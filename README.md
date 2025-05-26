# yarp-device-ethercat-CiA402 🚀

## Overview 🌟
This repository provides a YARP device plugin for EtherCAT CiA402 drives. It enables seamless integration of EtherCAT-based motion control devices into YARP-based robotic systems.

## Features ✨
- Implements the CiA402 state machine for motion control.
- Provides real-time communication with EtherCAT devices.
- Custom PDO mapping for enhanced fault management and control transitions.

## Compilation 🛠️

### Prerequisites 📋
Ensure the following dependencies are installed:
- [CMake](https://cmake.org/) (version 3.8 or higher)
- [YARP](https://www.yarp.it/)
- [SOEM](https://github.com/OpenEtherCATsociety/SOEM)

### Steps 🧩
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/yarp-device-ethercat-ds402.git
   cd yarp-device-ethercat-ds402
   ```
2. Create a build directory and navigate to it:
   ```bash
   mkdir build && cd build
   ```
3. Configure the project using CMake:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release
   ```
4. Build the project:
   ```bash
   make
   ```
5. Install the plugin (optional):
   ```bash
   make install
   ```

## Usage 🚀

### Running the Plugin 🏃‍♂️
After building the project, the plugin can be loaded into a YARP-based application. Ensure the `YARP_DATA_DIRS` environment variable includes the path to the plugin's configuration files. For example:
```bash
export YARP_DATA_DIRS=/path/to/build/share/yarp:$YARP_DATA_DIRS
### Configuration ⚙️
The plugin requires a configuration file to define the EtherCAT network and device parameters. An example configuration file is located in `config/robot/adj8/config.xml`. Key parameters include:
- **EtherCAT network interface**: Specify the network interface to use (e.g., `eth0`).
- **PDO mapping**: Define the custom PDO mapping for the devices.

### Example 💡
To run the plugin with a specific configuration:
```bash
yarprobotinterface --config config/robot/adj8/config.xml
```

## Note on PDO Mapping 📝

The plugin maps **Safe Torque Off (STO)** and **Safe Brake Control (SBC)** signals into Process Data Objects (PDOs). This mapping allows the real-time status of these safety-critical features to be directly accessible by the EtherCAT master. By integrating these signals into the PDOs, the system can monitor and manage safety states more effectively during runtime.

Although this approach deviates from strict CiA402 compliance, it provides significant advantages in fault management and control transitions. Specifically, it simplifies the process of detecting and responding to safety-related events, ensuring smoother and more reliable operation of the motion control system.

The `configurePDOMapping` method in the `EthercatManager` class is responsible for setting up this mapping, ensuring that the STO and SBC statuses are continuously updated and available for real-time decision-making.

## License 📜
This project is licensed under the BSD-3-Clause License. See the [`LICENSE`](LICENSE) file for details.
