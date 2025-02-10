/**
 * @file main.cpp
 * @brief test file Stepper motor control library for Raspberry Pi Pico.
			28BYJ-48 unipolar Motor, ULN2003 stepper motor controller.
 */
#include <vector>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stepper_motor_control/stepper_motor_control.hpp"

// motor setup 
// Define variables for motor control
std::vector<uint> motorPins = {2, 3, 4, 5}; // GPIO pins list
// Create motor instance
StepMotorControl motor(StepMotorControl::BYJ_48, motorPins);

// Emergency stop pushbutton, optional.
#define BUTTON_PIN 15  // GPIO pin for button

void Setup(void);
void EndTest(void);
void button_callback(uint gpio, uint32_t events) ; // Emergency stop pushbutton, optional.


int main() 
{
	Setup();
	// Run motor test 1
	// Define variables for motor run test: 360 turn clockwise
	uint32_t stepDelay = 2;
	int steps = 512;
	bool ccwise = false;
	bool verbose = true;
	uint32_t initDelay = 10;
	std::cout << "Test 1. 360 degree turn, half step." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::halfStep, initDelay);

	// Run motor test 2
	// Define variables for motor run: 180 turn counter clockwise and 2 second init delay
	stepDelay = 2;
	steps = 256;
	ccwise = true;
	verbose = true;
	initDelay = 2000;
	std::cout << "Test 2 180 degree turn, half step, counter clockwise, init delay 2." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::halfStep, initDelay);

	// Run motor test 3
	// Define variables for motor run test 3: 180 degree turn step delay slower
	stepDelay = 10;
	steps = 256;
	ccwise = true;
	verbose = true;
	initDelay = 10;
	std::cout << "Test 3 180 degree turn, half step, counter clockwise,  slower." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::halfStep, initDelay);

	// Run motor test 4
	// Define variables for motor run test:  360 turn full step
	stepDelay = 10;
	steps = 512;
	ccwise = false;
	verbose = true;
	initDelay = 10;
	std::cout << "Test 4. 360 degree turn , full step." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::fullStep, initDelay);

	// Run motor test 5
	// Define variables for motor run test: 360 turn , wave drive
	stepDelay = 10;
	steps = 512;
	ccwise = false;
	verbose = true;
	initDelay = 10;
	std::cout << "Test 5. 360 degree turn , wave drive." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::waveDrive, initDelay);

	EndTest();
	return 0;
}

void EndTest(void)
{
	// Stop motor after test
	motor.motorStop();
	//de- init motor 
	motor.motorDeInit();
	gpio_deinit(BUTTON_PIN);
	std::cout << "Motor test completed." << std::endl; 
}

void Setup(void)
{
	// Initialize standard I/O for debugging
	stdio_init_all(); // usb 38400 baud
	busy_wait_ms(1000);
	std::cout << "Motor test Started." << std::endl; 
	//init motor 
	motor.motorInit();

	//init pushbutton emergency stop, optional.
	gpio_init(BUTTON_PIN);
	gpio_set_dir(BUTTON_PIN, GPIO_IN);
	gpio_pull_up(BUTTON_PIN);  // Enable pull-up resistor
	// Attach interrupt on falling edge (button press)
	gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL , true, &button_callback);
}

// Interrupt callback function,  optional 
void button_callback(uint gpio, uint32_t events) 
{
	motor.setMotorStop(true);
}
