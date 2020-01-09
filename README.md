# ICM20948_DMP_Arduino
Invensense 20948 chip working with DMP code for Arduino environment

Copy the ICM20948 directory to your Arduino libraries folder
open and build the ino file.

It talks to 4 IMU's, selected by the selectMPU() call, with IO lines changing the I2C address using 4 IO pins in AD0_PINS array.
Set PIN Low and others HIGH to talk to a particular 20948
