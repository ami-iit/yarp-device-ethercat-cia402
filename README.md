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
   git clone https://github.com/your-repo/yarp-device-ethercat-ds402.git
   cd yarp-device-ethercat-ds402
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
The plugin requires a configuration file defining the EtherCAT network and device parameters. An example can be found at: [`config/robot/adj8/config.xml`](config/robot/adj8/config.xml)

### Setting Up `yarprobotinterface` 🛠️
To ensure that the `yarprobotinterface` binary has the correct permissions and can locate its dependencies, execute:

```bash
sudo setcap cap_net_raw,cap_net_admin+ep $(which yarprobotinterface)
patchelf --add-rpath $(dirname $(dirname $(which yarprobotinterface)))/lib $(which yarprobotinterface)
```

### Example 💡
To run the plugin with a specific configuration:
```bash
yarprobotinterface --config config/robot/adj8/config.xml
```

## Supported Drives 🛠️
This plugin has been primarily tested with Synapticon drives. While it may be compatible with other EtherCAT drive models or manufacturers, some modifications might be necessary to ensure proper functionality. This is due to the plugin’s use of a custom Process Data Object (PDO) mapping, which extends beyond the standard CiA402 specification.

If you're looking to adapt the plugin for different hardware, we encourage you to open an issue or contribute improvements.

### Note on PDO Mapping 📝
The plugin includes a custom mapping of the **Safe Torque Off (STO)** and **Safe Brake Control (SBC)** signals into PDOs. This design choice enables the EtherCAT master to access real-time data on these critical safety features, enhancing runtime monitoring and safety state management.

Although this approach diverges from strict CiA402 compliance, it brings practical advantages: improved fault detection, smoother safety transitions, and more robust motion control. The mapping is handled by the `configurePDOMapping` method in the `EthercatManager` class, ensuring that STO and SBC statuses are continuously updated and readily available for real-time decision-making.

## License 📜
This project is licensed under the BSD-3-Clause License. See the [`LICENSE`](LICENSE) file for details.
