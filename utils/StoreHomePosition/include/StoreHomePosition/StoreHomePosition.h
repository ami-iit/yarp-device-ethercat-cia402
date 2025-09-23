// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef STORE_HOME_POSITION_H
#define STORE_HOME_POSITION_H

#include <cstdint>
#include <memory>
#include <string>

namespace CiA402
{

/**
 * @brief Home on current position (CiA-402 method 37 or 35) and persist ONLY the home to flash.
 *
 * Sequence (per slave):
 *  1) Set OpMode=Homing (0x6060=6), write Homing method (0x6098=37/35), optional home offset
 * (0x607C). 2) Start homing by toggling Controlword bit 4 via SDO; poll Statusword bit 12 until
 * homing attained. 3) Set vendor flag 0x2005:02 = restoreOnStartup ? 1 : 0. 4) Trigger 0x1010:01 =
 * 0x65766173 ("evas") to save the changed home value/flag to flash.
 *
 * Notes:
 *  - Uses SDOs only; stays out of OP. No motion is commanded by this app.
 *  - restoreOnStartup=true means the drive boots already referenced (no homing at startup).
 */
class StoreHome37
{
public:
    StoreHome37();
    ~StoreHome37();

    StoreHome37(const StoreHome37&) = delete;
    StoreHome37& operator=(const StoreHome37&) = delete;

    /**
     * @param ifname            NIC name (e.g., "eth0").
     * @param homingMethod      37 (default) or 35.
     * @param homeOffset        Extra offset added to home (0x607C), default 0.
     * @param pollTimeoutMs     Timeout for homing attained polling, per slave.
     * @param restoreOnStartup  If true, writes 0x2005:02=1 so the saved home is applied at boot.
     *                          If false, writes 0x2005:02=0 (home saved but not auto-applied).
     * @return true if all slaves succeed; false otherwise.
     */
    bool run(const std::string& ifname,
             int8_t homingMethod = 37,
             int32_t homeOffset = 0,
             int pollTimeoutMs = 2000,
             bool restoreOnStartup = true);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace CiA402

#endif // STORE_HOME_POSITION_H