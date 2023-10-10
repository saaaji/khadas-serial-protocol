# CU Robotics Khadas-Teensy Serial Communication
*Temporary repo until permanent repo for Khadas software is created
#### Contains C++ firmware & python testing script for Khadas-side comms firmware:
- To test packet bounceback on the Teensy: run the .py file with CL argument `test-phys` (must have Teensy connected to test packet bounceback)
- OR with no CL arguments to inspect raw packet binary (Teensy may be disconnected)
- To compile the firmware: `g++ main.cpp SerialComms.cpp` (must have Teensy connected to run the executable)
