// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef YARP_OS_REALTIMEPERIODICTHREAD_H
#define YARP_OS_REALTIMEPERIODICTHREAD_H

#include <yarp/os/PeriodicThread.h>

#include <vector>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace yarp::os
{

/**
 * @brief Drop-in replacement for yarp::os::PeriodicThread with best-effort real-time helpers.
 *
 * The class preserves the standard PeriodicThread API while providing optional hooks to tune the
 * underlying POSIX thread (scheduler, priority, CPU affinity and memory locking). If a requested
 * feature cannot be applied (capabilities missing, platform unsupported, etc.) the thread keeps
 * running and a warning is reported on the YARP log.
 */
class RealTimePeriodicThread : public yarp::os::PeriodicThread
{
public:
    /**
     * @brief Tunable parameters for the real-time configuration.
     *
     * Defaults target a SCHED_FIFO thread with elevated priority and memory locking on Linux.
     * Non-Linux builds use placeholders and simply keep the compatibility layer active.
     */
    struct Parameters
    {
        /** Whether to attempt applying the real-time configuration in threadInit(). */
        bool enableRealtime{true};
#ifdef __linux__
        /** Scheduling policy used when enabling real-time (e.g., SCHED_FIFO, SCHED_RR). */
        int policy{SCHED_FIFO};
        /** POSIX priority to assign together with the policy. */
        int priority{80};
#else
        /** Scheduling policy placeholder for non-Linux builds. */
        int policy{-1};
        /** Priority placeholder for non-Linux builds. */
        int priority{0};
#endif
        /** Optional list of CPU cores to pin the thread to (empty = keep current affinity). */
        std::vector<int> cpuAffinity{};
        /** Lock process memory to minimise page faults during the control loop. */
        bool lockMemory{true};
        /** Restore original scheduling/affinity when stopping the thread. */
        bool restoreSchedulingOnStop{true};
    };

    /**
     * @brief Construct a new RealTimePeriodicThread with default Parameters.
     *
     * @param period Loop period in seconds.
     * @param useSystemClock Choose between system or network clock.
     * @param clockAccuracy Relative or absolute wake-up strategy.
     */
    explicit RealTimePeriodicThread(double period,
                                    ShouldUseSystemClock useSystemClock = ShouldUseSystemClock::No,
                                    PeriodicThreadClock clockAccuracy
                                    = PeriodicThreadClock::Relative);

    /**
     * @brief Construct a new RealTimePeriodicThread with a custom Parameters block.
     *
     * @param period Loop period in seconds.
     * @param parameters Real-time configuration to apply in threadInit().
     * @param useSystemClock Choose between system or network clock.
     * @param clockAccuracy Relative or absolute wake-up strategy.
     */
    RealTimePeriodicThread(double period,
                           const Parameters& parameters,
                           ShouldUseSystemClock useSystemClock = ShouldUseSystemClock::No,
                           PeriodicThreadClock clockAccuracy = PeriodicThreadClock::Relative);

    /** @brief Virtual destructor required by PeriodicThread. */
    ~RealTimePeriodicThread() override;

    /**
     * @brief Update the Parameter set that will be used at the next start().
     *
     * When invoked while the thread is running, the values are cached and a warning is emitted to
     * highlight that changes will only take effect after stop()/start().
     */
    void setRealTimeParameters(const Parameters& parameters);

    /** @brief Read back the cached Parameters currently configured. */
    const Parameters& realTimeParameters() const noexcept;

protected:
    /**
     * @brief Initialise the PeriodicThread and apply the real-time configuration if enabled.
     *
     * The base class threadInit() is executed first; afterwards configureRealtime() is called to
     * tweak scheduler/affinity/memory as requested by the user.
     */
    bool threadInit() override;

    /** @brief Ensure temporary scheduling tweaks are reverted before the thread stops. */
    void threadRelease() override;

private:
    /**
     * @brief Helper that performs the best-effort real-time configuration.
     *
     * @return true if all features were applied successfully, false otherwise.
     */
    bool configureRealtime();

    /** @brief Undo the adjustments performed by configureRealtime(). */
    void restoreRealtime();

    Parameters m_parameters;
    bool m_memoryLocked{false};
    bool m_schedApplied{false};
    bool m_affinityApplied{false};

#ifdef __linux__
    int m_previousPolicy{-1};
    struct sched_param m_previousSchedParam{};
    cpu_set_t m_previousAffinity{};
#endif
};

} // namespace yarp::os

#endif // YARP_OS_REALTIMEPERIODICTHREAD_H
