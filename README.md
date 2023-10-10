# CU Robotics Khadas-Teensy Serial Communication
*Temporary repo until permanent repo for Khadas software is created
#### Contains C++ firmware & python testing script for Khadas-side comms firmware:
- Team members may view the protocol specification [here](https://docs.google.com/document/d/1gBOFfmUhH06k1NlOR8c_m2aSDpMiyN7HGMBWxrBApJw/edit?usp=sharing)
- To test packet bounceback on the Teensy: run the .py file with CL argument `test-phys` (must have Teensy connected to test packet bounceback)
- OR with no CL arguments to inspect raw packet binary (Teensy may be disconnected)
- To compile the firmware: `g++ main.cpp SerialComms.cpp` (must have Teensy connected to run the executable)
