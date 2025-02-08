/**
 * @file stepper_motor_control.hpp
 * @brief Stepper motor control class definition for Raspberry Pi Pico.
 * @details URL :: https://github.com/gavinlyonsrepo/Motor_Control_PICO.git
 */

# pragma once

// Include libraries section
#include "pico/stdlib.h"
#include <vector>
#include <iostream>
#include <algorithm>  // std::reverse and std::find
#include <iomanip> // Set floating point precision 
#include <map>

// Debug marco flag , comment in to enable debug 
//#define debug_StepMtrCtrlLib 

// ################################################
// ### StepMotorControlCommon class ###
// ################################################

/*!
	@brief Class to hold common code for all sub classes,
		 inherited by sub classes.
*/
class StepMotorControlCommon
{
	public:
		/*! Enum to define a standard return code for most functions that return failures*/
		enum StepMotorControlError_e : uint8_t
		{
			StepMotorControl_Success,       /**< Success, Function ran without defined Error :)*/
			StepMotorControl_StepValue,     /**< Step Value incorrect, Bad input step number by user */
			StepMotorControl_UnknownError,  /**< Unknown Exception was occurred */
			StepMotorControl_StepModeType,  /**< Unknown Step mode requested */
			StepMotorControl_MotorStopFlag  /**< The motor stop flag was been set true externally */
		};

		StepMotorControlCommon();
		bool getMotorStop(void);
		void setMotorStop(bool);
	protected:
		float degree_calc(int steps, const std::string& steptype); 
		bool _stopMotor;  ///< Motor stop flag
	private:
};

// ################################################
// ### StepMotorControl class ###
// ################################################

/**
 * @class StepMotorControl
 * @brief Controls a stepper motor using GPIO pins on Raspberry Pi Pico.
 * @details Supported devices by this class
 * 	-# 28BYJ-48 bipolar motor driven by ULN2003
 * 	-# Nema bipolar motor driven by L298N 
 * 	-# Nema bipolar motor driven byMX1508 
 * 	-# Nema bipolar motor driven by TB6612FNG 
 */
class StepMotorControl : public StepMotorControlCommon
{
public:

	/**
	 * @enum MotorType_e
	 * @brief Enum representing different motor types.
	 */
	enum MotorType_e : uint8_t 
	{
		BYJ_48,  ///< 28BYJ-48 unipolar Motor (stepper motor controller ULN2003 )
		L298N_NEMA,   ///< Bipolar Nema motor, L298N -based Stepper Motor controller
		MX1508_NEMA, ///< Bipolar Nema motor, MX1508 based Stepper Motor controller
		TB6612FNG_NEMA, ///< Bipolar Nema motor, TB6612FNG based Stepper Motor controller
		UNKNOWN ///< Unknown motor type
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

	StepMotorControl(MotorType_e motor_type = BYJ_48, std::vector<uint> gpiopins = {0 ,1 ,2 ,3});
	void motorInit(void);
	void motorDeInit(void);
	void motorStop(void);
	StepMotorControlError_e motorRun(uint32_t stepDelay = 10, int steps = 512,
			   bool ccwise = false, bool verbose = false, StepMode_e steptype = halfStep,
			   uint32_t initdelay = 1);

private:
	void displayDegree(int steps);
	void displayGPIO(void);

	float _degreeValue = 0.0F;   ///< Degree per step value
	MotorType_e  _motorType;     ///< Motor type
	std::vector<uint> _gpioPins; ///< GPIO pins used for motor control
};


//################################################
// ### StepMotorControlEasy class ###
//################################################

/**
 * @class StepMotorControlEasy
 * @brief Controls a stepper motor using GPIO pins on Raspberry Pi Pico.
 * @details Supported devices by this class
 * 	-# Bipolar Nema Stepper motor A3967 Easy Driver 
 */
class StepMotorControlEasy : public StepMotorControlCommon
{
	public:
		StepMotorControlEasy(uint dirPin, uint stepPin, int msPins[2]);
		StepMotorControlEasy(uint dirPin, uint stepPin);

		void motorInit(void);
		void motorDeInit(void);
		void motorStop(void);

		StepMotorControlError_e motorMove(uint32_t stepdelay = 10, int steps = 200, bool clockwise = false,
				    bool verbose = false, std::string steptype = "Full", uint32_t initdelay = 1);
	
	protected:

	private:
		uint _directionPin;   /**< GPIO for direction pin  */
		uint _stepPin;        /**< GPIO for direction pin  */
		int _modePins[2];     /**< Microstep Resolution MS1-MS2 -> GPIO Pins */
		bool _modePinsEnable; /**< Microstep resolution enabled  */
		
		std::map<std::string, std::pair<bool, bool>> resolution = {
			{"Full", {0, 0}},
			{"Half", {1, 0}},
			{"1/4", {0, 1}},
			{"1/8", {1, 1}}
		}; /**<  Step mode type mapped to MSX GPIO pins */

};

// ### EOF ###