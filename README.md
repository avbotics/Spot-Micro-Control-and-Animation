# Spot-Micro-Control-and-Animation
Inverse Kinematics based control and animation of Spotmicro developped in Python for Raspeberry Pi

This first version is a "simulator" of Spot Micro that runs under Python 3 and requires the Pygame Library in order to display animation and use the xbox one (or any similar) controller

The SW does not include any function to control the servos PWM. I chose to do so as this part is very dependent on the HW set-up (type of µC board, servos type, ...)
However, the SW includes all the functions that enable Spot Micro to walk, sit, give the paws, lie, twist and even lift the hind leg...
All these features are based upon Inverse Kinematics.

In the main program Spot_Micro_Control_v01.py:

-Line 57 to 72: set-up of the XBOX One (or similar) controller. You can use the attached utility Essai_Joystick_01.py to identify the parameters.

-Line 565: This is the best place where you could place the call to your own servo control function.
 
Based upon the paws to body position, the Inverse Kinematics functions calculates 4 sets of 3 angles (x shoulder, y shoulder, y elbow/knee): thetalf, thetarf, thetarr, thetalr:

-lf: left front leg
-rf : right front leg
-rr : right rear leg
-lr : left rear leg

I expect that it is not necessary to understand exactly how the SW works providing that finally only these generated angles are necessary to control Spot Micro.
When adapting to your own servos, please pay attention to zero setting of the servos and the tuning of the angles range so that actual angles correspond to the calculated ones.

I attached a pdf file that gives some details about how inverse kinematics is built-up and correspondance between calculated angles and servos angles.

Attached Libraries:

-Spotmicro_Inverse_Kinematics_and_Position_Library_v01.py:
  This file contains a class of paramaters and functions to calculate Spot micro position.
  It includes :
  
   -The main dimensions of Spot micro
   -The center of gravity position and weight of each spotmicro limbs and body
   -The forward kinematics function
   -The forward kinematics functions
   -The function that generates walking positions
   -The function that generates positions from known start and end positionsThis library contains a class of functions to calculate the center of gravity position
    and distance to the support polygon edge


-Spotmicro_Gravity_Center_Library_v01.py:
  This file contains a class of functions to calculate the center of gravity position
  and distance to the support polygon edge


-Spotmicro_Animation_Library_v01.py:
  This file contains a class of functions to generate the animation frames

Please make sure that all these files are located in the same folder.

-Essai_Joystick_01.py:

  This utility was copied from https://www.pygame.org/docs/ref/joystick.htm. It is helpfull to identify the controller / joystick parameters
  
I will release soon a full version corresponding to my HW for information or for persons willing to apply the same of very close set-up.
Actual S/W includes the control of IMU, voltage sensor, LCD display and ultrasonic range sensors
It also includes features in order to toggle animation display and servos motion. This is very usefull to test movements prior to apply them in real life.

I connect to the Raspberry Pi with VNC viewer through Wifi (and sometimes LAN Cable). 

My own Spot Micro version uses:

-Raspberry Pi 4

-PCA9685 shield (I2c) modified in order to separate servos power supply (6V) and 5V logic supply

-12 x 30 kg.cm servos

-MPU 6050 (I2c) IMU

-ADS 1015 (I2c) for voltage measurement (with a simpe resistors voltage divider)

-2x16 characters LCD display (I2c)

-2 HC-SR04 untrasonic range sensors

-20 A SBEC supply (servos) set at 6V

-5 A SBEC supply for the RPi and shields logic circuits

-2S 4000 mAh LiPo Battery

I hope you will enjoy playing with this software as much as I had writing it ! 

Arnaud Villeneuve

December 2020




