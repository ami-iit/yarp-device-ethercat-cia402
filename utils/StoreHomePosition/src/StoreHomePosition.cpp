// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <CiA402/EthercatManager.h>
#include <CiA402/LogComponent.h>

#include <StoreHomePosition/StoreHomePosition.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace CiA402;
using namespace std::chrono_literals;

// ---- CiA-402 / Synapticon indices ----
static constexpr uint16_t IDX_CONTROLWORD = 0x6040; // uint16
static constexpr uint16_t IDX_STATUSWORD = 0x6041; // uint16
static constexpr uint16_t IDX_OPMODE = 0x6060; // int8  : 6 = Homing
static constexpr uint16_t IDX_POSITION_ACT = 0x6064; // int32 : for logging
static constexpr uint16_t IDX_HOMING_METHOD = 0x6098; // int8  : 35/37
static constexpr uint16_t IDX_HOME_OFFSET = 0x607C; // int32
static constexpr uint16_t IDX_STORE_PARAMS = 0x1010; // uint32: :01 = 'evas'

static constexpr uint16_t IDX_HOME_VENDOR = 0x2005; // Synapticon: :01 Home, :02 Restore-on-load

// ---- Statusword helpers ----
static inline bool swHomingAttained(uint16_t sw)
{
    return (sw & (1u << 12)) != 0;
}
static inline bool swHomingError(uint16_t sw)
{
    return (sw & (1u << 13)) != 0;
}

class StoreHome37::Impl
{
public:
    bool run(const std::string& ifname, int8_t hm, int32_t hoff, int timeoutMs, bool restore)
    {
        // Initialize EtherCAT master
        yCInfo(CIA402, "StoreHome37: initializing EtherCAT on %s", ifname.c_str());
        const auto rc = mgr.init(ifname);
        if (rc != EthercatManager::Error::NoError)
        {
            yCError(CIA402, "StoreHome37: init failed on %s (rc=%d)", ifname.c_str(), int(rc));
            return false;
        }

        // Discover slaves (1-based indices in SOEM)
        std::vector<int> slaves;
        for (int s = 1;; ++s)
        {
            auto name = mgr.getName(s);
            if (name.empty())
                break;
            yCInfo(CIA402, "StoreHome3537: found slave %d: %s", s, name.c_str());
            slaves.push_back(s);
        }
        if (slaves.empty())
        {
            yCError(CIA402, "StoreHome3537: no slaves found");
            return false;
        }

        bool allOk = true;
        for (int s : slaves)
            allOk &= homeAndPersistSingle(s, hm, hoff, timeoutMs, restore);
        return allOk;
    }

private:
    bool homeAndPersistSingle(int s, int8_t hm, int32_t hoff, int timeoutMs, bool restore)
    {
        // -- 1) OpMode = Homing (6)
        {
            const int8_t op = 6;
            if (mgr.writeSDO<int8_t>(s, IDX_OPMODE, 0x00, op) != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: write 0x6060=6 failed", s);
                return false;
            }
        }

        // -- 2) 0x6098 = 37 (or 35)
        if (hm != 35 && hm != 37)
        {
            yCWarning(CIA402, "s%02d: invalid homing method %d, using 37", s, int(hm));
            hm = 37;
        }
        if (mgr.writeSDO<int8_t>(s, IDX_HOMING_METHOD, 0x00, hm) != EthercatManager::Error::NoError)
        {
            yCError(CIA402, "s%02d: write 0x6098=%d failed", s, int(hm));
            return false;
        }

        // -- 3) Optional extra home offset
        if (hoff != 0)
        {
            if (mgr.writeSDO<int32_t>(s, IDX_HOME_OFFSET, 0x00, hoff)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: write 0x607C=%d failed", s, hoff);
                return false;
            }
        }

        // -- 4) Start homing: toggle Controlword bit4 (mode-specific "start")
        {
            uint16_t cw = 0;
            (void)mgr.readSDO<uint16_t>(s, IDX_CONTROLWORD, 0x00, cw); // best-effort read
            const uint16_t cwLow = static_cast<uint16_t>(cw & ~(1u << 4));
            const uint16_t cwHigh = static_cast<uint16_t>(cwLow | (1u << 4));
            if (mgr.writeSDO<uint16_t>(s, IDX_CONTROLWORD, 0x00, cwLow)
                    != EthercatManager::Error::NoError
                || mgr.writeSDO<uint16_t>(s, IDX_CONTROLWORD, 0x00, cwHigh)
                       != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: toggle 0x6040 bit4 failed", s);
                return false;
            }
        }

        // -- 5) Poll 0x6041: bit12 attained / bit13 error
        const auto t0 = std::chrono::steady_clock::now();
        while (true)
        {
            uint16_t sw = 0;
            if (mgr.readSDO<uint16_t>(s, IDX_STATUSWORD, 0x00, sw)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: read 0x6041 failed", s);
                return false;
            }
            if (swHomingError(sw))
            {
                yCError(CIA402, "s%02d: homing error (0x6041=0x%04X)", s, sw);
                return false;
            }
            if (swHomingAttained(sw))
            {
                int32_t pos = 0;
                (void)mgr.readSDO<int32_t>(s, IDX_POSITION_ACT, 0x00, pos);
                yCInfo(CIA402, "s%02d: homing attained (pos=%d, sw=0x%04X)", s, pos, sw);
                break;
            }
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0)
                    .count()
                > timeoutMs)
            {
                yCError(CIA402, "s%02d: timeout waiting homing attained", s);
                return false;
            }
            std::this_thread::sleep_for(10ms);
        }

        // -- 6) (Optional) read vendor Home Position 0x2005:01 for logging
        int32_t homeVal = 0;
        (void)mgr.readSDO<int32_t>(s, IDX_HOME_VENDOR, 0x01, homeVal);
        yCInfo(CIA402, "s%02d: vendor Home (0x2005:01) = %d", s, homeVal);

        // -- 7) Set restore-on-startup flag per request (0x2005:02)
        {
            const uint8_t flag = restore ? uint8_t{1} : uint8_t{0};
            if (mgr.writeSDO<uint8_t>(s, IDX_HOME_VENDOR, 0x02, flag)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: write 0x2005:02=%u failed", s, flag);
                return false;
            }
        }

        // -- 8) Save to flash: 0x1010:01 = 'evas' (0x65766173)
        {
            constexpr uint32_t EVAS = 0x65766173u;
            if (mgr.writeSDO<uint32_t>(s, IDX_STORE_PARAMS, 0x01, EVAS)
                != EthercatManager::Error::NoError)
            {
                yCError(CIA402, "s%02d: save 0x1010:01='evas' failed", s);
                return false;
            }
            yCInfo(CIA402,
                   "s%02d: configuration saved (home persisted, restoreOnStartup=%s)",
                   s,
                   restore ? "true" : "false");
        }

        return true;
    }

    EthercatManager mgr;
};

StoreHome37::StoreHome37()
{
    m_impl = std::make_unique<Impl>();
}

StoreHome37::~StoreHome37() = default;

bool StoreHome37::run(yarp::os::ResourceFinder& rf)
{
    const std::string ifname = rf.check("ifname") ? rf.find("ifname").asString()
                                                  : std::string("eth0");
    int methodTmp = 37;
    if (rf.check("method"))
    {
        methodTmp = rf.find("method").asInt32();
    }
    const int8_t method = static_cast<int8_t>(methodTmp);

    int32_t homeOffset = 0;
    if (rf.check("home-offset"))
    {
        homeOffset = rf.find("home-offset").asInt32();
    }

    int timeoutMs = 2000;
    if (rf.check("timeout-ms"))
    {
        timeoutMs = rf.find("timeout-ms").asInt32();
    }

    bool restoreOnBoot = true;
    if (rf.check("restore-on-boot"))
    {
        // Accept bool or int
        if (rf.find("restore-on-boot").isBool())
        {
            restoreOnBoot = rf.find("restore-on-boot").asBool();
        } else
        {
            restoreOnBoot = (rf.find("restore-on-boot").asInt32() != 0);
        }
    }

    yCInfo(CIA402,
           "StoreHome37: ifname=%s method=%d home-offset=%d timeout-ms=%d restore-on-boot=%s",
           ifname.c_str(),
           int(method),
           homeOffset,
           timeoutMs,
           restoreOnBoot ? "true" : "false");
    yCInfo(CIA402, "Do you want to proceed? (press ENTER to continue)");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return m_impl->run(ifname, method, homeOffset, timeoutMs, restoreOnBoot);
}
