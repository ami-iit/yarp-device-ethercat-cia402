// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <string>

#include <CiA402/LogComponent.h>
#include <StoreHomePosition/StoreHomePosition.h>

#include <yarp/os/LogStream.h>

int main(int argc, char** argv)
{
    const std::string ifname = (argc > 1) ? argv[1] : "eth0";
    const int8_t method = (argc > 2) ? static_cast<int8_t>(std::stoi(argv[2])) : 37; // 37 or 35
    const int32_t homeOffset = (argc > 3) ? std::stoi(argv[3]) : 0;
    const int timeoutMs = (argc > 4) ? std::stoi(argv[4]) : 2000;
    const bool restoreOnBoot = (argc > 5) ? (std::stoi(argv[5]) != 0) : true; // default true

    CiA402::StoreHome37 app;
    const bool ok = app.run(ifname, method, homeOffset, timeoutMs, restoreOnBoot);
    if (!ok)
    {
        yCError(CIA402, "StoreHome37: FAILED");
        return 1;
    }
    yCInfo(CIA402, "StoreHome37: DONE");
    return 0;
}
