
/**
 * @file stepper_motor_control.cpp
 * @brief Stepper motor control implementation for Raspberry Pi Pico.
 * @details URL :: https://github.com/gavinlyonsrepo/Motor_Control_PICO.git
 */

#include "../../include/stepper_motor_control/stepper_motor_control.hpp"

// ################################################
// ### StepMotorControlCommon class ###
// ################################################

StepMotorControlCommon::StepMotorControlCommon(){
	// Blank constructor
}

/*!
 * @brief Calculates the size of a turn in degrees based on the number of steps and step type.
 * 
 * @param steps The number of steps taken by the motor.
 * @param steptype The step resolution mode (e.g., "Full", "Half", "1/4", etc.).
 * @return float The equivalent rotation in degrees.
 * 
 * @note The function assumes that `steptype` is a valid key in the `degree_value` map.
 */
float StepMotorControlCommon::degree_calc(int steps, const std::string& steptype) 
{
	std::map<std::string, float> degree_value = {
		{"Full", 1.8},
		{"Half", 0.9},
		{"1/4", 0.45},
		{"1/8", 0.225},
		{"1/16", 0.1125},
		{"1/32", 0.05625},
		{"1/64", 0.028125},
		{"1/128", 0.0140625}
	};
	return steps * degree_value[steptype];
}

/**
 * @brief Retrieves the motor stop flag.
 * @return True if the motor is stopped, false otherwise.
 */
bool StepMotorControlCommon::getMotorStop(void){return _stopMotor;}

/**
 * @brief Sets the motor stop flag.
 * @param value Boolean indicating stop state.
 */
void StepMotorControlCommon::setMotorStop(bool value){_stopMotor = value;}


// ################################################
// ### StepMotorControl class ###
// ################################################

/**
 * @brief Constructor for StepMotorControl.
 * @param motorType The type of motor.
 * @param gpioPins A vector of GPIO pins used for motor control.
 */
StepMotorControl::StepMotorControl(MotorType_e motorType, std::vector<uint> gpioPins)
{
	_motorType = motorType; 
	_gpioPins = gpioPins;
	_stopMotor = false;
	_degreeValue = 0.0F;
}

/**
 * @brief Initializes the motor by setting up GPIO pins.
 */
void StepMotorControl::motorInit(void)
{
	for (auto pin : _gpioPins) 
	{
		gpio_init(pin);
		gpio_set_dir(pin, GPIO_OUT);
		gpio_put(pin, 0);
	}
}

/**
 * @brief Deinitialize the motor by resetting GPIO pins.
 */
void StepMotorControl::motorDeInit(void)
{
	motorStop();
	for (auto pin : _gpioPins) 
	{
		gpio_deinit(pin);
	}
}

/**
 * @brief Stops the motor by setting all GPIO pins to low.
 */
void StepMotorControl::motorStop() 
{
	for (auto pin : _gpioPins) 
	{
		gpio_put(pin, 0);
	}
	_stopMotor = true;
}

/*!
 * @brief Runs the stepper motor with specified parameters.
 * @param stepDelay Time delay (in mS) between steps.
 * @param steps Number of steps to move the motor.
 * @param ccwise Direction of rotation (true for counterclockwise, false for clockwise).
 * @param verbose Enables detailed output if true.
 * @param steptype Step mode (half-step, full-step, or wave drive).
 * @param initdelay Initial delay (in mS) before starting movement.
 * @return  StepMotorControlError_e enum 
 *		-# StepMotorControl_Success for success.
 *		-# StepMotorControl_MotorStopFlag  stopMotor flag was set true during motor loop.
 *		-# StepMotorControl_UnknownErrorAn unknown exception occurred.
 *		-# StepMotorControl_StepValue Bad input step number by user.
 *		-# StepMotorControl_StepModeType Unknown Step mode type.
 */
StepMotorControlCommon::StepMotorControlError_e StepMotorControl::motorRun(uint32_t stepDelay, int steps,
		   bool ccwise, bool verbose, StepMode_e steptype,
		   uint32_t initdelay) {
	// Ensure that step count is valid
	if (steps < 0) {
		std::cout << "Error 1: motorRun: Step number must be greater than 0" << std::endl; 
		return StepMotorControl_StepValue;
	}

	// Reset stop flag and apply initial delay before movement starts
	_stopMotor = false;
	sleep_ms(initdelay);

	// Define the step sequence based on the selected step mode
	// stepSequence is a vector of vectors.
	std::vector<std::vector<uint>> stepSequence;
	try{
		/* Step sequence setup */
		switch (steptype)
		{
		case halfStep: 
			// Half-step mode: Alternates between single-coil and dual-coil activation
			stepSequence = {{_gpioPins[0]}, {_gpioPins[0], _gpioPins[1]}, {_gpioPins[1]}, {_gpioPins[1], _gpioPins[2]},
					{_gpioPins[2]}, {_gpioPins[2], _gpioPins[3]}, {_gpioPins[3]}, {_gpioPins[3], _gpioPins[0]}};
		break ;
		case fullStep: 
			// Full-step mode: Two adjacent coils are always activated
			stepSequence = {{_gpioPins[0], _gpioPins[1]}, {_gpioPins[1], _gpioPins[2]},
					{_gpioPins[2], _gpioPins[3]}, {_gpioPins[0], _gpioPins[3]}};
		break;
		case waveDrive: 
			// Wave drive mode: Only one coil is activated at a time
			stepSequence = {{_gpioPins[0]}, {_gpioPins[1]}, {_gpioPins[2]}, {_gpioPins[3]}};
		break;
		default: {
			// Invalid step mode error
			std::cout << "Error 2: motorRun : Unknown step type (half, full, or wave)" << std::endl; 
			return StepMotorControl_StepModeType;
		}
		}
		// Reverse the sequence if counterclockwise rotation is requested
		if (ccwise) 
		{
			reverse(stepSequence.begin(), stepSequence.end());
		}
		// Execute step sequence for the specified number of steps, The outer loop iterates through steps.
		for (int stepsRemaining = steps; stepsRemaining > 0 && !_stopMotor; --stepsRemaining) 
		{
			// The inner loop iterates through stepSequence, setting the appropriate GPIOs HIGH or LOW.
			for (auto& pinList : stepSequence) 
			{
				if (_stopMotor) break; // Stop execution if motor stop flag is set
				// Activate only the required GPIO pins for the current step
				for (auto pin : _gpioPins) 
				{
					// std::find(start, end, value)
					gpio_put(pin, std::find(pinList.begin(), pinList.end(), pin) != pinList.end());	
				}
				// Wait for the specified step duration
				sleep_ms(stepDelay);
			}
			#ifdef debug_StepMtrCtrlLib
			std::cout << "Steps count: " << stepsRemaining << "\r";
			std::cout.flush();
			#endif
		}
		// If stop flag was triggered externally, stop the motor and display message
		if (_stopMotor == true)
		{
			motorStop();
			std::cout << "Stop Button pressed." << std::endl;
			return StepMotorControl_MotorStopFlag;
		} 
	}
	catch(const std::exception& e)
	{
		// Handle any exceptions, stop the motor, and display the error
		motorStop();
		std::cerr << "An exception has occurred in MotorRun : " << e.what() << '\n';
		return StepMotorControl_UnknownError;
	}

	// Ensure the motor is stopped after completing the step sequence
	motorStop();

	// Verbose output
	if (verbose) {
		std::cout << "Motor Run Finished:  Details:" << std::endl;
		std::cout << "Motor type enum no = " << +_motorType  << std::endl; 
		displayGPIO();
		std::cout << std::defaultfloat << std::setprecision(6);
		std::cout << "Initial delay mS = " << initdelay  << std::endl; 
		std::cout << "Step Delay time mS = " << stepDelay  << std::endl; 
		std::cout << "Number of step sequences = " << steps  << std::endl; 
		std::cout << "Size of step sequence = " << stepSequence.size()  << std::endl; 
		std::cout << "Number of steps = " << steps * stepSequence.size()  << std::endl; 
		displayDegree(steps);
		std::cout << "Counter clockwise = " << (ccwise ? "true" : "false")  << std::endl; 
		std::cout << "Verbose = " << (verbose ? "true" : "false")  << std::endl; 
		std::cout << "Steptype enum no = " << +steptype  << std::endl; 
	}
	return StepMotorControl_Success;
}


/*!
 * @brief Displays degree values based on motor type.
 * @param steps Number of steps motor moved
 * @details for Calculation size of turn in degrees for verbose reporting
 */
void StepMotorControl::displayDegree(int steps)
{
	switch (_motorType)
	{
	case BYJ_48: 
		_degreeValue = 1.422222F; 
		std::cout << "Size of turn in degrees = " << ((float)steps / _degreeValue) << std::endl;

	break;
	case L298N_NEMA: 
	case MX1508_NEMA: 
	case TB6612FNG_NEMA:
		_degreeValue = 7.20F; 
		std::cout << "Size of turn in degrees = " << ((float)steps * _degreeValue) << std::endl;
	break;
	case UNKNOWN:
	default:{
		std::cout << "Info : displayDegree : Unknown Motor Type"  << _motorType << std::endl;
		_degreeValue = 0.0F;
		}
	}
}

/**
 * @brief Displays motor GPIO pins 
 * @details used for verbose reporting
 */
void StepMotorControl::displayGPIO(void)
{
	std::cout << "GPIO pins = ";
	for (auto pin : _gpioPins) std::cout << pin << " ";
	std::cout  << std::endl; 
}

//################################################
// ### StepMotorControlEasy class ###
//################################################

/*!
 * @brief Constructor for StepMotorControlEasy.
 * @param dirPin GPIO for direction control
 * @param stepPin  GPIO for step control
 */
StepMotorControlEasy::StepMotorControlEasy(uint dirPin, uint stepPin)
{
	_directionPin = dirPin;
	_stepPin = stepPin;
	_stopMotor = false;
	_modePinsEnable = false;
}

/**
 * @brief Constructor for StepMotorControlEasy.
 * @param dirPin GPIO for direction control
 * @param stepPin  GPIO for step control
 * @param msPins GPIO for resolution contorl 2 off MS1 and MS2
 */
StepMotorControlEasy::StepMotorControlEasy(uint dirPin, uint stepPin, int msPins[2])
{
	_directionPin = dirPin;
	_stepPin = stepPin;
	_stopMotor = false;
	_modePinsEnable = true;
	if (_modePinsEnable)
	{
		_modePins[0] = msPins[0];
		_modePins[1] = msPins[1];
	}
}

/**
 * @brief Initializes the motor by setting up GPIO pins.
 */
void StepMotorControlEasy::motorInit(void)
{
	gpio_init(_directionPin);
	gpio_set_dir(_directionPin, GPIO_OUT);
	gpio_init(_stepPin);
	gpio_set_dir(_stepPin, GPIO_OUT);
	if (_modePinsEnable) 
	{
		gpio_init(_modePins[0]);
		gpio_set_dir(_modePins[0], GPIO_OUT);
		gpio_init(_modePins[1]);
		gpio_set_dir(_modePins[1], GPIO_OUT);
	}
}

/**
 * @brief Deinitialize the motor by resetting GPIO pins to default
 */
void StepMotorControlEasy::motorDeInit(void)
{
	motorStop();	
	gpio_deinit(_directionPin);
	gpio_deinit(_stepPin);
	if (_modePinsEnable) 
	{
		gpio_deinit(_modePins[0]);
		gpio_deinit(_modePins[1]);
	}
}

/**
 * @brief Stops the motor by setting all GPIO pins to low.
 */
void StepMotorControlEasy::motorStop() 
{
	gpio_put(_directionPin, 0);
	gpio_put(_stepPin, 0);
	if (_modePinsEnable) 
	{
		gpio_put(_modePins[0], 0);
		gpio_put(_modePins[1], 0);
	}
	_stopMotor = true;
}

/*!
 * @brief Runs the stepper motor with specified parameters.
 * @param stepdelay Time delay (in mS) between steps.
 * @param steps Number of steps to move the motor.
 * @param ccwise Direction of rotation (true for counterclockwise, false for clockwise).
 * @param verbose Enables detailed output if true.
 * @param steptype Step mode Full half 1/4 1/8
 * @param initdelay Initial delay (in mS) before starting movement.
 * @return  StepMotorControlError_e enum 
 *		-# StepMotorControl_Success for success.
 *		-# StepMotorControl_MotorStopFlag  stopMotor flag was set true during motor loop.
 *		-# StepMotorControl_UnknownError An unknown exception occurred.
 *		-# StepMotorControl_StepValue Bad input step number by user.
 *		-# StepMotorControl_StepModeType invalid Step mode type.
 */
 StepMotorControlCommon::StepMotorControlError_e StepMotorControlEasy::motorMove(uint32_t stepdelay , int steps, bool clockwise,
				    bool verbose, std::string steptype, uint32_t  initdelay) 
{
	// 1. User error checks
	// 1-1 Ensure that step count is valid
	if (steps < 0) {
		std::cout << "Error 1: motorMove: Step number must be greater than 0" << std::endl; 
		return StepMotorControl_StepValue;
	}
	// 1.2 Ensure resolution type is valid
	if (resolution.find(steptype) == resolution.end()) 
	{
		std::cerr << "Error 2: motorMove : Invalid steptype: " << steptype << " Available options: ";
		for (const auto& res : resolution) 
		{
			std::cerr << res.first << " ";
		}
		std::cerr << std::endl;
		return StepMotorControl_StepModeType;
	}


	// 2. Setup motor
	_stopMotor = false;
	try 
	{
		gpio_put(_directionPin, clockwise);
		if (_modePinsEnable)  
		{
			gpio_put(_modePins[0], resolution[steptype].first);
			gpio_put(_modePins[1], resolution[steptype].second);
		}
		sleep_ms(initdelay);
		// 3. Step the motor
		for (int i = 0; i < steps; i++) 
		{
			if (_stopMotor) break;

			gpio_put(_stepPin, false);
			sleep_ms(stepdelay);
			gpio_put(_stepPin, true);
			sleep_ms(stepdelay);
			#ifdef debug_StepMtrCtrlLib
				std::cout << "Steps count: " << i + 1 << "\r";
				std::cout.flush();
			#endif
		}
		// If stop flag was triggered externally, stop the motor and display message
		if (_stopMotor)
		{
			motorStop();
			std::cout << "Info : motorMove: Stop Button pressed." << std::endl;
			return StepMotorControl_MotorStopFlag;
		}  
		std::cout <<  std::endl;
	}	
	catch(const std::exception& e)
	{
		// Handle any exceptions, stop the motor, and display the error
		motorStop();
		std::cerr << "Error: motorMove: An exception has occurred: " << e.what() << '\n';
		return StepMotorControl_UnknownError;
	}
	// finish happy path
	motorStop();
	if (verbose)
	{
		std::cout << "Motor Move finished, Details:." << std::endl;
		std::cout << "Clockwise = " << (clockwise ? "true" : "false") << std::endl;
		std::cout << "Step Type = " << steptype << std::endl;
		std::cout << "Number of steps = " << steps << std::endl;
		std::cout << "Step Delay mS = " << stepdelay << std::endl;
		std::cout << "Initial delay mS = " << initdelay << std::endl;
		std::cout << "Size of turn in degrees = " << degree_calc(steps, steptype) << std::endl;
	}
	return StepMotorControl_Success;
}


// EOF