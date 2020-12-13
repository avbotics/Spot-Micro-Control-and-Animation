# Spot-Micro-Control-and-Animation
Inverse Kinematics based control and animation of Spotmicro developped in Python for Raspeberry Pi

This first version is a "simulator" of Spot Micro that runs under Python 3 and requires the Pygame Library in order to display animation and use the xbox one (or any similar) controller

The SW does not include any function to control the servos PWM. I chose to do so as this is very dependant on the HW set-up (type of µC board, servos type, ...)
However, the SW includes all the functions that enable Spot Micro to walk, sit, give the paws, lie, twist and even lift the hind leg...
All these features are based upon Inverse Kinematics.

Line 57 to 72 








My own Spot Micro version uses:
-Raspberry Pi 4
-PCA9685 shield (I2c) modified in order to separate servos power supply (6V) and 5V logic supply
-12 30 kg.cm servos
-MPU 6050 (I2c) IMU
-ADS 1015 (I2c) for voltage measurement (with a simpe resistors voltage divider)
-2x16 characters LCD display (I2c)
-2 HC-SR04 untrasonic range sensors
-20 A SBEC supply (servos) set at 6V
-5 A SBEC supply for the RPi and shields logic circuits
-2S 4000 mAh LiPo Battery



