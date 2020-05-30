# RoboticFlyingSquirrel

This code is for the control of the SCRAM (Soaring and Climbing Rainforest Robot for Aerial Monitoring) robotic flying squirrel.
`Tail_control.ino` uses PID control to adjust the angle of the robot's tail (as controlled by a servomotor) in response to the glider's pitch (measured using an IMU).
`Motor_control.ino` rotates the DC motor to move the robot's legs.

## Tail_control.ino
Two libraries are necessary for the code to function, servo.h from Marko Margolis and [MPU9250.h](https://github.com/bolderflight/MPU9250) from Brian Taylor of BolderFlight.

## Leg_control.ino
No additional libraries are necessary.

## Setup
The code has been designed to run from a Teensy 3.2 microcontroller, so require some adjustments before it can run from other microcontrollers. The necessary components for the robot and how to connect them together are detailed below:

![Electronics Diagram](/images/Electronics_Diagram.png)
