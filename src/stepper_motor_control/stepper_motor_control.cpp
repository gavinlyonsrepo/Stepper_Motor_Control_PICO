
/**
 * @file stepper_motor_control.cpp
 * @brief Stepper motor control implementation for Raspberry Pi Pico.
 * @details URL :: https://github.com/gavinlyonsrepo/Motor_Control_PICO.git
 */

#include "../../include/stepper_motor_control/stepper_motor_control.hpp"

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
 * @brief Deinitializes the motor by resetting GPIO pins.
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

/**
 * @brief Runs the stepper motor with specified parameters.
 * @param wait Time delay (in seconds) between steps.
 * @param steps Number of steps to move the motor.
 * @param ccwise Direction of rotation (true for counterclockwise, false for clockwise).
 * @param verbose Enables detailed output if true.
 * @param steptype Step mode (half-step, full-step, or wave drive).
 * @param initdelay Initial delay (in seconds) before starting movement.
 * @return 
 *		-#  0 for success.
 *		-# -1 stopMotor flag was set true during motor loop.
 *		-# -2 An unknown exception occurred.
 *		-# -3 Bad input step number by user.
 *		-# -4 Unknown Step mode type.
 */
int StepMotorControl::motorRun(float wait, int steps,
		   bool ccwise, bool verbose, StepMode_e steptype,
		   float initdelay) {
	// Ensure that step count is valid
	if (steps < 0) {
		std::cout << "Error 1: motorRun: Step number must be greater than 0" << std::endl; 
		return -3;
	}

	// Reset stop flag and apply initial delay before movement starts
	_stopMotor = false;
	sleep_ms(initdelay * 1000);

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
			return -4;
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
				sleep_ms(wait * 1000);
			}
		}

		// If stop flag was triggered, stop the motor and display message
		if (_stopMotor == true)
		{
			motorStop();
			std::cout << "Stop Button pressed." << std::endl;
			return -1;
		} 
	}
	catch(const std::exception& e)
	{
		// Handle any exceptions, stop the motor, and display the error
		motorStop();
		std::cerr << "An exception has occurred in MotorRun : " << e.what() << '\n';
		return -2;
	}

	// Ensure the motor is stopped after completing the step sequence
	motorStop();

	/* Verbose output */
	if (verbose) {
		std::cout << "\nVerbose Details::" << std::endl;
		std::cout << "Motor type enum no = " << +_motorType  << std::endl; 
		displayGPIO();
		std::cout << std::defaultfloat << std::setprecision(6);
		std::cout << "Initial delay = " << initdelay  << std::endl; 
		std::cout << "Wait time = " << wait  << std::endl; 
		std::cout << "Number of step sequences = " << steps  << std::endl; 
		std::cout << "Size of step sequence = " << stepSequence.size()  << std::endl; 
		std::cout << "Number of steps = " << steps * stepSequence.size()  << std::endl; 
		displayDegree(steps);
		std::cout << "Counter clockwise = " << (ccwise ? "true" : "false")  << std::endl; 
		std::cout << "Verbose = " << (verbose ? "true" : "false")  << std::endl; 
		std::cout << "Steptype enum no = " << +steptype  << std::endl; 
	}
	return 0;
}


/**
 * @brief Displays degree values based on motor type. 
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
	case MC1508_NEMA: 
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

/**
 * @brief Retrieves the motor stop flag.
 * @return True if the motor is stopped, false otherwise.
 */
bool StepMotorControl::getMotorStop(void){return _stopMotor;}

/**
 * @brief Sets the motor stop flag.
 * @param value Boolean indicating stop state.
 */
void StepMotorControl::setMotorStop(bool value){_stopMotor = value;}
