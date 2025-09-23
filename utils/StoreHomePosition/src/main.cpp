// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <string>

#include <CiA402/LogComponent.h>
#include <StoreHomePosition/StoreHomePosition.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char** argv)
{
    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("yarp-cia420-store-home.ini");
    rf.configure(argc, argv);

    CiA402::StoreHome37 app;
    const bool ok = app.run(rf);
    if (!ok)
    {
        yCError(CIA402, "StoreHome37: FAILED");
        return 1;
    }
    yCInfo(CIA402, "StoreHome37: DONE");
    return 0;
}
