# Stepper_Motor_Control_PICO

## Overview

* Title: Stepper Motor Control PICO 
* Description: 

A Raspberry pi Pico SDK C++ stepper motor library.
The components supported are some of the most widely used by electronic hobbyists community.
This is a partial port of my 'rpiMotorLib' a python raspberry pi library.
Not are components supported yet.

* URL to source port: [URL LINK](https://github.com/gavinlyonsrepo/RpiMotorLib)


## Supported Components

Note: the help file links are to the aforementioned raspberry pi python port.
The theory of operation, hardware setup and most of the software setup is the same.
There is an example file for each motor in examples. Select which one is complied  
The example files are in example folder. To build the one you want, 
edit the Cmaketxt file add_executable(${PROJECT_NAME} section, 
comment in one example file path and one only. 
Code is commented for Doxygen API generation. 

### Stepper motors

| Motor tested | Motor controller| example file |Help File URL |
| ----- | ----- | ----- | ---- |
|Unipolar 28BYJ-48| ULN2003 driver module| BYJ_48|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/28BYJ.md)| 
|Bipolar Nema| TB6612FNG Dual Driver Carrier| TB6612FNG_NEMA |[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11TB6612FNG.md) |
|Bipolar Nema| L298N H-Bridge controller module| L298N_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11L298N.md) |
|Bipolar Nema| MX1508 Motor controller module| MX1508_NEMA|[Link](https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/extra/Documentation/Nema11MX150X.md) |
|Bipolar Nema(untested)| DV8833 Motor controller module| TODO| TODO |



## Output 

If verbose output is enabled. Data reports and any errors are sent to console at 38400 baud via USB. 

![ op ](https://github.com/gavinlyonsrepo/Stepper_Motor_Control_PICO/blob/main/extra/images/output.png)





