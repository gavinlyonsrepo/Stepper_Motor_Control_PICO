[![Website](https://img.shields.io/badge/Website-Link-blue.svg)](https://gavinlyonsrepo.github.io/)  [![Rss](https://img.shields.io/badge/Subscribe-RSS-yellow.svg)](https://gavinlyonsrepo.github.io//feed.xml)  [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/paypalme/whitelight976)

# Stepper_Motor_Control_PICO

## Overview

* Title: Stepper Motor Control PICO 
* Description: 

A Raspberry pi Pico SDK C++ stepper motor library.
The components supported are some of the most widely used by the electronic hobbyist community. This is a partial port of my 'rpiMotorLib' a python raspberry pi library. Not are components supported yet.

* URL to 'rpiMotorLib': [URL LINK](https://github.com/gavinlyonsrepo/RpiMotorLib)


## Supported Components

Note: the help file links are to the aforementioned raspberry pi python port.
The theory of operation, hardware setup and much of the software interface is the same. There is an example file for each motor in examples. To build the one you want, edit the CMakeLists.txt file add_executable(${PROJECT_NAME} section, 
comment in one example file path and one only. 
Code is commented for Doxygen API generation. 
Code for an optional push button to ground which can be used as software 
stop on motor movement is included in the example files. 

### Stepper motors

| Motor tested | Motor controller| example file |Help File URL |
| ----- | ----- | ----- | ---- |
|Bipolar Nema| A3967 Easy Driver| A3967_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11A3967Easy.md)| 
|Unipolar 28BYJ-48| ULN2003 driver module| BYJ_48|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/28BYJ.md)| 
|Bipolar Nema| TB6612FNG Dual Driver Carrier| TB6612FNG_NEMA |[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11TB6612FNG.md) |
|Bipolar Nema| L298N H-Bridge controller module| L298N_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11L298N.md) |
|Bipolar Nema| MX1508 Motor controller module| MX1508_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11MX150X.md) |
|Bipolar Nema| A4988 Motor controller module| A4988_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11A4988.md) |
|Bipolar Nema| DRV8825 Motor controller module| DRV8825_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11DRV8825.md) |
|Bipolar Nema| LV8729Motor controller module| LV8729_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11LV8729.md) |
|Bipolar Nema(untested)| DV8833 Motor controller module| TODO| TODO |



## Output 

If verbose output is enabled. Data reports and any errors are sent to console at 38400 baud via USB. 

![ op ](https://github.com/gavinlyonsrepo/Stepper_Motor_Control_PICO/blob/main/extra/images/output.png)





