// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef STORE_HOME_POSITION_H
#define STORE_HOME_POSITION_H

#include <cstdint>
#include <memory>
#include <string>

#include <yarp/os/ResourceFinder.h>

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
     * @brief Run the homing and persistence procedure on all slaves found on the given interface.
     * @param rf ResourceFinder with configuration parameters (see below).
     * @return true on success (all slaves homed and persisted); false on any error.
     *
     * @note Expected keys in the ResourceFinder:
     * - ifname: string NIC name (default: "eth0")
     * - method: int 37 or 35 (default: 37)
     * - homeOffset: int32 offset (default: 0)
     * - timeoutMs: int polling timeout (default: 2000)
     * - restoreOnBoot: bool/int (default: 1)
     */
    bool run(yarp::os::ResourceFinder& rf);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace CiA402

#endif // STORE_HOME_POSITION_H