# BMP280-STM32F4xx
Firmware for STM32F4xx, to operate the BMP280/BME280 via I2C.
Also, added a library for BMP280, so it can be ported to any other STM32 by changing just two include files.


# 1 - Build the code
Use STMCubeIDE or any other Toolchain to build. For another ST MCU, change the HAL library and device library header in the baroLib.h/.c and the userHW.h/.c files. 

# 2 - Use Pullups in your wiring
Use external I2C pullups in your wiring, it doesn't work without them. The BMP on it's own will show constant values and wrong ones.

# 3 - Follow
