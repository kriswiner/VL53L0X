# VL53L0X

Basic Arduino sketch intended to run on the Teensy 3.2 (but easily modified to run on any Arduino MCU). The register definitions are taken from ST Microelectronics' API since they will not publish the register map. The sketch reads the WHO_AM_I register and revision ID, then configures the sensor for 1 shot, repeated mode which is essentially continuous mode output. The data is read and counts as well as distance in mm is output to serial. I tried to get the interrupt to work but I am missing something still.

Update: Better to use the [Arduino library](https://github.com/pololu/vl53l0x-arduino) created by Pololu based on ST's API. Several sketches in this repository do, including properly configuring the interrupt and changing the data modes between long-range, lower-accuracy and shorter-range , high-accuracy modes. Even at 2 meters, the VL53L0X is accurate to within 1 or 2 mm. Pretty amazing!

The VL53L0X breakout [board](https://www.tindie.com/products/onehorse/vl53l0x-time-of-flight-ranging-sensor/) I use can be purchased on Tindie.
