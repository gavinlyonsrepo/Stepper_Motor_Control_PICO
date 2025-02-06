/**
 * @file stepper_motor_control.hpp
 * @brief Stepper motor control class definition for Raspberry Pi Pico.
 */

# pragma once

#include "pico/stdlib.h"
#include <vector>
#include <iostream>
#include <algorithm>  // std::reverse and std::find
#include <iomanip> // Set floating point precision 

/**
 * @class StepMotorControl
 * @brief Controls a stepper motor using GPIO pins on Raspberry Pi Pico.
 */
class StepMotorControl 
{
public:

	/**
	 * @enum MotorType_e
	 * @brief Enum representing different motor types.
	 */
	enum MotorType_e : uint8_t 
	{
		BYJ48,  ///< 28BYJ-48 Stepper Motor
		LN298   ///< L298-based Stepper Motor
	};

	/**
	 * @enum StepMode_e
	 * @brief Enum representing different stepping modes.
	 */
	enum StepMode_e : uint8_t 
	{
		fullStep, ///< Full stepping mode
		halfStep, ///< Half stepping mode
		waveDrive ///< Wave drive stepping mode
	};

	StepMotorControl(MotorType_e motor_type = BYJ48, std::vector<uint> gpiopins = {0 ,1 ,2 ,3});
	void motorInit(void);
	void motorDeInit(void);
	void motorStop(void);
	void motorRun(float wait = 0.001, int steps = 512,
			   bool ccwise = false, bool verbose = false, StepMode_e steptype = halfStep,
			   float initdelay = 0.001);
	
	bool getMotorStop(void);
	void setMotorStop(bool);

private:
	/**
	 * @brief Sets the degree value based on motor type.
	 */
	void displayDegree(void);

	float _degreeValue = 0.0F;   ///< Degree per step value
	MotorType_e  _motorType;     ///< Motor type (BYJ48 or LN298)
	bool _stopMotor;             ///< Motor stop flag
	std::vector<uint> _gpioPins; ///< GPIO pins used for motor control
};


