# HKC_minicheetah_modifed_f28379d

Hardware setup:
BLDC motor: Written for a U8IIkv100 T_motor. If you use another motor, you should change motor_config.h
[Any 3-Phase motor]

Magnetic Encoder AS5147P
[Position Sense]

Motor Driver Boostxl-drv8323rs
[Current Sense, Commutation]

Microcontroller Launchxl-f28379d
[Digital Controller (Logic)]


Software setup:
Code Composer Studio
C2000Ware_4_02_00_00

Embed this program to flash memory.
It is not perfect yet, there are no calibration mode and setup mode. It works in motor mode.

Please see the Simulink model(Another repository) for CAN communication for Torque, Position, and Impedance commands.
