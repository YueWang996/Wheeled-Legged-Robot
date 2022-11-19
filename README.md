# Wheeled Legged Robot
A low cost wheeled legged robot driven by BLDC motor and Feetech serial servos.
## Project Introduction
There are two branches in this project--_Main_ and _MotorDriver_

- Branch [Main](https://github.com/YueWang996/Wheeled-Legged-Robot/tree/main) contains code for the main control unit. It contains an ESP32 board and an MPU6050 6-DOF IMU. 
- Brance [MotorDriver](https://github.com/YueWang996/Wheeled-Legged-Robot/tree/MotorDriver) contains code based on SimpleFOC for the motor control board. It is a dual-channel BLDC motor controller which can control two BLDC motors at the same time. The onboard CAN transceiver receives instructions from the main control unit so that the two motors can move as desired. 

## Environment
- ESP-IDF 4.2
- Clion 2021.2.3
- Python 3.9

Maybe you can refer to [this link (Chinese version)](https://www.bilibili.com/read/cv15226500) to get more information about how to configure and use ESP-IDF toolchain in Clion. 

## Major Hardwares
- ESP32-WROOM-32D
- Feetech ST3215 serial servo * 4
- Feetech servo control board
- GY-521 (MPU6050)

## Project Structure
All source code is placed in the [src](https://github.com/YueWang996/Wheeled-Legged-Robot/tree/main/src).
All model files are stored in the directory [Models](https://github.com/YueWang996/Wheeled-Legged-Robot/tree/main/Models).
