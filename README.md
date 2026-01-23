# BNO055 STM32 HAL Library

This library is designed to interface the Bosch BNO055 9-axis Absolute Orientation Sensor (IMU) with **STM32** microcontrollers using the **HAL (Hardware Abstraction Layer)** library.

It features a simple, modular, and easy-to-use architecture. It supports reading raw Accelerometer, Gyroscope, and Magnetometer data, as well as fused Euler Angles and Quaternion data.

##  Features

* **Easy Initialization:** Single function setup and mode selection (`BNO_Init`).
* **Multi-Mode Support:** Supports all operation modes (NDOF, ACCONLY, MAGONLY, IMUPLUS, etc.).
* **Full Data Access:**
    * Raw Accelerometer, Gyroscope, Magnetometer data.
    * Fused Euler Angles (Heading, Roll, Pitch).
    * Quaternion data.
    * Linear Acceleration and Gravity vectors.
    * Temperature data.
* **Calibration Management:** Functions to read and restore calibration offsets (essential for retaining calibration across resets).
* **Axis Configuration:** Remap axes based on the sensor's mounting position (Axis Remap).
* **External Crystal:** Option to enable/disable external crystal usage for better accuracy.

##  Requirements

* STM32 Microcontroller (F1, F4, G4, H7, etc.)
* STM32 HAL Library
* I2C Connectivity

## Important Notes
HAL Delay: The library uses HAL_Delay() for timing requirements during mode switches. Be cautious when calling these functions inside Interrupt Service Routines (ISRs).
I2C Timeout: The default timeout is 100ms. If you experience I2C communication errors, consider increasing BNO055_I2C_TIMEOUT in bno055.h.
## License
This project is open-source. Feel free to use and modify it as needed.

