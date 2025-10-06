// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "RealTimePeriodicThread.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <cerrno>
#include <cstring>

#ifdef __linux__
#include <sys/mman.h>
#endif

using namespace yarp::os;

namespace
{
// Anonymous logger used for every realtime tweak emitted by this helper class.
YARP_LOG_COMPONENT(RT_THREAD, "yarp.os.RealTimePeriodicThread");
} // namespace

RealTimePeriodicThread::RealTimePeriodicThread(double period,
                                               ShouldUseSystemClock useSystemClock,
                                               PeriodicThreadClock clockAccuracy)
    : yarp::os::PeriodicThread(period, useSystemClock, clockAccuracy)
    , m_parameters{}
{
    // Default-construct with the typical SCHED_FIFO + memory lock preset.
}

RealTimePeriodicThread::RealTimePeriodicThread(double period,
                                               const Parameters& parameters,
                                               ShouldUseSystemClock useSystemClock,
                                               PeriodicThreadClock clockAccuracy)
    : yarp::os::PeriodicThread(period, useSystemClock, clockAccuracy)
    , m_parameters(parameters)
{
    // Allow callers to provide a bespoke Parameters block upfront.
}

RealTimePeriodicThread::~RealTimePeriodicThread() = default;

void RealTimePeriodicThread::setRealTimeParameters(const Parameters& parameters)
{
    if (this->isRunning())
    {
        yCWarning(RT_THREAD,
                  "Realtime parameters updated while thread is running. Changes will take effect on"
                  " next start.");
    }
    // Cache the configuration for the next threadInit() invocation.
    m_parameters = parameters;
}

const RealTimePeriodicThread::Parameters&
RealTimePeriodicThread::realTimeParameters() const noexcept
{
    // Provide read-only access to the currently staged configuration.
    return m_parameters;
}

bool RealTimePeriodicThread::threadInit()
{
    // Run the base initialisation first; abort if it fails.
    if (!yarp::os::PeriodicThread::threadInit())
    {
        return false;
    }

    if (!m_parameters.enableRealtime)
    {
        yCInfo(RT_THREAD, "Realtime features disabled for this thread");
        return true;
    }

    // Attempt to apply realtime settings; log a warning but continue on failure.
    if (!configureRealtime())
    {
        yCWarning(RT_THREAD,
                  "Failed to configure realtime parameters. Running in best-effort mode");
    }

    return true;
}

void RealTimePeriodicThread::threadRelease()
{
    // Undo any realtime tweaks before delegating to the parent clean-up.
    restoreRealtime();
    yarp::os::PeriodicThread::threadRelease();
}

bool RealTimePeriodicThread::configureRealtime()
{
    bool success{true};

#ifdef __linux__
    const pthread_t threadHandle = pthread_self();

    if (m_parameters.lockMemory)
    {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0)
        {
            m_memoryLocked = true;
            yCInfo(RT_THREAD, "Locked process memory (MCL_CURRENT|MCL_FUTURE)");
        } else
        {
            yCWarning(RT_THREAD,
                      "mlockall failed (%s) — continuing without memory lock",
                      strerror(errno));
            success = false;
        }
    }

    // Adjust scheduler/priority if the user requested a change and the OS allows it.
    if (m_parameters.policy >= 0 || m_parameters.priority > 0)
    {
        struct sched_param targetParam{};
        if (pthread_getschedparam(threadHandle, &m_previousPolicy, &m_previousSchedParam) != 0)
        {
            yCWarning(RT_THREAD,
                      "Unable to query existing scheduling policy (%s)",
                      strerror(errno));
            m_previousPolicy = -1;
            success = false;
        }

        targetParam = m_previousSchedParam;

        int policyToSet = m_previousPolicy;
        if (m_parameters.policy >= 0)
        {
            policyToSet = m_parameters.policy;
        }
        if (m_parameters.priority > 0)
        {
            targetParam.sched_priority = m_parameters.priority;
        }

        if (policyToSet >= 0)
        {
            if (pthread_setschedparam(threadHandle, policyToSet, &targetParam) == 0)
            {
                m_schedApplied = true;
                yCInfo(RT_THREAD,
                       "Realtime scheduling enabled (policy=%d priority=%d)",
                       policyToSet,
                       targetParam.sched_priority);
            } else
            {
                yCWarning(RT_THREAD,
                          "Failed to set realtime scheduling (policy=%d priority=%d): %s",
                          policyToSet,
                          targetParam.sched_priority,
                          strerror(errno));
                success = false;
            }
        }
    }

    if (!m_parameters.cpuAffinity.empty())
    {
#ifdef __GLIBC__
        // Pin the control loop to the requested cores while keeping the previous mask for restore.
        cpu_set_t targetAffinity{};
        CPU_ZERO(&targetAffinity);
        for (int cpu : m_parameters.cpuAffinity)
        {
            if (cpu >= 0)
            {
                CPU_SET(static_cast<unsigned int>(cpu), &targetAffinity);
            }
        }

        if (pthread_getaffinity_np(threadHandle, sizeof(cpu_set_t), &m_previousAffinity) != 0)
        {
            yCWarning(RT_THREAD, "Unable to query current CPU affinity: %s", strerror(errno));
            success = false;
        } else
        {
            if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &targetAffinity) == 0)
            {
                m_affinityApplied = true;
                yCInfo(RT_THREAD, "CPU affinity applied");
            } else
            {
                yCWarning(RT_THREAD, "Failed to set CPU affinity: %s", strerror(errno));
                success = false;
            }
        }
#else
        yCWarning(RT_THREAD, "CPU affinity configuration not supported on this platform");
        success = false;
#endif
    }
#else
    return false;
#endif

    return success;
}

void RealTimePeriodicThread::restoreRealtime()
{
#ifdef __linux__
    const pthread_t threadHandle = pthread_self();

    if (m_parameters.restoreSchedulingOnStop && m_schedApplied && m_previousPolicy >= 0)
    {
        if (pthread_setschedparam(threadHandle, m_previousPolicy, &m_previousSchedParam) != 0)
        {
            yCWarning(RT_THREAD,
                      "Unable to restore previous scheduling policy (%s)",
                      strerror(errno));
        } else
        {
            yCInfo(RT_THREAD, "Restored previous scheduling policy");
        }
    }
    m_schedApplied = false;

#ifdef __GLIBC__
    if (m_parameters.restoreSchedulingOnStop && m_affinityApplied)
    {
        if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &m_previousAffinity) != 0)
        {
            yCWarning(RT_THREAD, "Unable to restore CPU affinity (%s)", strerror(errno));
        } else
        {
            yCInfo(RT_THREAD, "Restored previous CPU affinity");
        }
    }
    m_affinityApplied = false;
#endif

    if (m_memoryLocked)
    {
        // Release the memory lock so the process returns to its original state.
        if (munlockall() != 0)
        {
            yCWarning(RT_THREAD, "munlockall failed (%s)", strerror(errno));
        } else
        {
            yCInfo(RT_THREAD, "Unlocked process memory");
        }
    }
    m_memoryLocked = false;
#endif
}
