/**
 * @file stepper_motor_control.hpp
 * @brief Stepper motor control class definition for Raspberry Pi Pico.
 * @details URL :: https://github.com/gavinlyonsrepo/Motor_Control_PICO.git
 */

# pragma once

// Include libraries section
#include "pico/stdlib.h"
#include <iostream>   // for cout
#include <algorithm>  // for reverse and find
#include <iomanip>    // Set floating point precision 
#include <vector>
#include <array>
#include <map>
#include <set>

// Debug marco flag , comment in to enable debug 
//#define debug_StepMtrCtrlLib 

// ################################################
// ### StepMotorControlCommon class section ###
// ################################################

/*!
	@brief Class to hold common code for all sub classes,
		 inherited by sub classes.
*/
class StepMotorControlCommon
{
	public:
		/*! Enum to define a standard return code for most functions that return failures*/
		enum ReturnCode_e : uint8_t
		{
			Success,               /**< Success, Function ran without defined Error :)*/
			StepValueIncorrect,    /**< Step Value incorrect, Bad input step number by user */
			UnknownException,      /**< Unknown Exception occurred */
			UnknownStepModeType,   /**< Unknown Step mode requested */
			MotorStopFlagSet,      /**< The motor stop flag set externally */
			GenericError,          /**< Unknown generic error */
			UnknownMotorType       /**< Unsupported motor type */
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
// ### StepMotorControl class section ###
// ################################################

/**
 * @class StepMotorControl
 * @brief Controls a stepper motor using GPIO pins on Raspberry Pi Pico.
 * @details Supported devices by this class
 * 	-# 28BYJ-48 bipolar motor driven by ULN2003
 * 	-# Nema bipolar motor driven by L298N 
 * 	-# Nema bipolar motor driven by MX1508 
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
		BYJ_48,      ///< 28BYJ-48 unipolar Motor (stepper motor controller ULN2003 )
		L298N_NEMA,     ///< Bipolar Nema motor, L298N -based Stepper Motor controller
		MX1508_NEMA,    ///< Bipolar Nema motor, MX1508 based Stepper Motor controller
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
	ReturnCode_e motorRun(uint32_t stepDelay = 10, int steps = 512,
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
// ### StepMotorControlEasy class section ###
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
		StepMotorControlEasy(uint8_t dirPin, uint8_t stepPin, std::array<int, 2> msPins);
		StepMotorControlEasy(uint8_t dirPin, uint8_t stepPin);

		void motorInit(void);
		void motorDeInit(void);
		void motorStop(void);

		ReturnCode_e motorMove(uint32_t stepdelay = 10, int steps = 200, bool clockwise = false,
				    bool verbose = false, std::string steptype = "Full", uint32_t initdelay = 1);
	
	protected:

	private:
		uint8_t _directionPin;   /**< GPIO for direction pin  */
		uint8_t _stepPin;        /**< GPIO for step pin  */
		std::array<int, 2> _modePins;     /**< Microstep Resolution MS1-MS2 -> GPIO Pins */
		bool _modePinsEnable; /**< Microstep resolution enabled  */
		
		std::map<std::string, std::pair<bool, bool>> resolution = {
			{"Full", {0, 0}},
			{"Half", {1, 0}},
			{"1/4", {0, 1}},
			{"1/8", {1, 1}}
		}; /**<  Step mode type mapped to MSX GPIO pins */

};

//################################################
// ### StepMotorControlMicro class section ###
//################################################

/**
 * @class StepMotorControlMicro
 * @brief Controls a stepper motor using GPIO pins on Raspberry Pi Pico.
 * @details Supported devices by this class
 * 	-# Nema bipolar motor driven by A4988 
 * 	-# Nema bipolar motor driven by DRV8825 
 * 	-# Nema bipolar motor driven by LV8729 
 */
class StepMotorControlMicro : public StepMotorControlCommon
{
	public:
		StepMotorControlMicro(uint8_t directionPin, uint8_t stepPin, std::vector<int> mode_pins, std::string motor_type = "A4988");
		StepMotorControlMicro(uint8_t directionPin, uint8_t stepPin, std::string motor_type = "A4988");

		void motorInit(void);
		void motorDeInit(void);
		void motorStop(void);
		ReturnCode_e motorGo(bool clockwise = false, const std::string& steptype = "Full", int steps = 200, uint32_t stepdelay = 5, bool verbose = false, uint32_t initdelay = 1);

	protected:

	private:

		ReturnCode_e resolutionSet(const std::string& steptype);

		uint8_t _directionPin;      /**< GPIO for direction pin  */
		uint8_t _stepPin;           /**< GPIO for step pin  */
		std::vector<int> _modePins; /**< Microstep Resolution MS1-MS3 -> GPIO Pins */
		std::string _motorType;     /**< Name of a supported motor */
		bool _modePinsEnable;       /**< Microstep resolution enabled  */
};

// ### EOF ###