# VL53L0X

Basic Arduino sketch intended to run on the Teensy 3.2 (but easily modified to run on any Arduino MCU). The register definitions are taken from ST Microelectronics' API since they will not publish the register map. The sketch reads the WHO_AM_I register and revision ID, then configures the sensor for 1 shot, repeated mode which is essentially continuous mode output. The data is read and counts as well as distance in mm is output to serial. I tried to get the interrupt to work but I am missing something still.

The VL53L0X breakout [board}(https://www.tindie.com/products/onehorse/vl53l0x-time-of-flight-ranging-sensor/) I use can be purchased on Tindie.
