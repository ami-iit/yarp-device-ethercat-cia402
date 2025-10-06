# yarp-device-ethercat-CiA402 🚀

## Overview 🌟
This repository provides a YARP device plugin for EtherCAT CiA402 drives.

## Compilation 🛠️

### Prerequisites 📋
Ensure the following dependencies are installed:
- [CMake](https://cmake.org/) (version 3.8 or higher)
- [YARP](https://www.yarp.it/)
- [SOEM](https://github.com/OpenEtherCATsociety/SOEM)

> **Note**: This device has been tested only on Linux systems.

### Build Steps 🧩

1. Clone the repository:
   ```bash
   git clone https://github.com/ami-iit/yarp-device-ethercat-cia402.git
   cd yarp-device-ethercat-cia402
   ```
2. Create a build directory and navigate into it:
   ```bash
   mkdir build && cd build
   ```

3. Configure the project with CMake:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install
   ```

4. Build the project:
   ```bash
   make
   ```

5. Install the plugin:
   ```bash
   make install
   ```

## Usage 🚀

### Running the Plugin 🏃‍♂️

After building the project, the plugin can be loaded into a YARP-based application. Make sure the `YARP_DATA_DIRS` environment variable includes the path to the plugin's configuration files:
```bash
export YARP_DATA_DIRS=/path/to/install:$YARP_DATA_DIRS
```

### Configuration ⚙️
The plugin requires a configuration file defining the EtherCAT network and device parameters. An example can be found at: [`config/robot/template_1_motor/config.xml`](config/robot/template_1_motor/config.xml)

#### Real-time thread tuning ⏱️
When running on a Linux system with a real-time kernel you can opt into deterministic scheduling for the EtherCAT loop by adding the following optional keys to the device configuration:

- `rt_enable` *(bool, default `true`)* — disable to fall back to standard `PeriodicThread` behaviour.
- `rt_priority` *(int, default `80`)* — POSIX thread priority used when the scheduler supports it.
- `rt_policy` *(string/int)* — scheduling policy (`fifo`, `rr`, `other`, or a numeric policy value). Only honoured on Linux.
- `rt_cpu_affinity` *(list of int)* — pin the control loop to one or more CPU cores.
- `rt_lock_memory` *(bool, default `true`)* — call `mlockall(MCL_CURRENT|MCL_FUTURE)` to minimise page faults.
- `rt_restore_policy` *(bool, default `true`)* — restore the previous scheduler and affinity when the thread stops.

The defaults target a SCHED_FIFO thread with priority 80 and memory locking. If the kernel refuses a setting (for example due to missing capabilities) the device logs a warning and keeps running in best-effort mode.

### Setting Up `yarprobotinterface` 🛠️
To ensure that the `yarprobotinterface` binary has the correct permissions and can locate its dependencies, execute:

```console
patchelf --add-rpath $(dirname $(dirname $(which yarprobotinterface)))/lib $(which yarprobotinterface)
sudo setcap cap_net_raw,cap_net_admin+ep $(which yarprobotinterface)
```

### Example 💡
To run the plugin with a specific configuration:
```bash
yarprobotinterface --config config/robot/template_1_motor/config.xml
```

## Supported Drives 🛠️
This plugin has been primarily tested with Synapticon drives. While it may be compatible with other EtherCAT drive models or manufacturers, some modifications might be necessary to ensure proper functionality. This is due to the plugin’s use of a custom Process Data Object (PDO) mapping, which extends beyond the standard CiA402 specification.

If you're looking to adapt the plugin for different hardware, we encourage you to open an issue or contribute improvements.

### Additional notes 📝
For more details, see:
- Protocol map — PDOs, SDOs, and conversion formulas: [doc/protocol_map.md](./doc/protocol_map.md)
- Modes and setpoints — available control modes and targets: [doc/modes_and_setpoints.md](./doc/modes_and_setpoints.md)
- Dual encoder handling — mounts, sources, and transformations: [doc/dual_encoder_handling.md](./doc/dual_encoder_handling.md)


## License 📜
This project is licensed under the BSD-3-Clause License. See the [`LICENSE`](LICENSE) file for details.
