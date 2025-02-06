/**
 * @file main.cpp
 * @brief test file Stepper motor control lbrary for Raspberry Pi Pico.
 */
#include <vector>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stepper_motor_control/stepper_motor_control.hpp"

// motor setup 
// Define varibles for motor control
std::vector<uint> motorPins = {0, 1, 2, 3}; // GPIO pins list
// Create motor instance
StepMotorControl motor(StepMotorControl::BYJ48, motorPins);

// Emergency stop pushbutton, optional.
#define BUTTON_PIN 15  // GPIO pin for button

void Setup(void);
void EndTest(void);
void button_callback(uint gpio, uint32_t events) ; // Emergency stop pushbutton, optional.


int main() 
{
	// Run motor test 1
	// Define varibles for motor run test 360 turn
	float stepDelay = 0.002F;
	int steps = 512;
	bool ccwise = false;
	bool verbose = true;
	float initDelay = 0.001F;
	std::cout << "Test 1." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::halfStep, initDelay);

	// Run motor test 2
	// Define varibles for motor run 180 turn + test counter cloclkwise and init delay
	stepDelay = 0.002F;
	steps = 256;
	ccwise = true;
	verbose = true;
	initDelay = 2.0F;
	std::cout << "Test 2." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::halfStep, initDelay);

	// Run motor test 3
	// Define varibles for motor run test 3 180 degree turn step delay slower
	stepDelay = 0.1F;
	steps = 256;
	ccwise = true;
	verbose = true;
	initDelay = 0.001F;
	std::cout << "Test 3." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::halfStep, initDelay);

	// Run motor test 4
	// Define varibles for motor run test 360 turn full step
	stepDelay = 0.01F;
	steps = 512;
	ccwise = false;
	verbose = true;
	initDelay = 0.01F;
	std::cout << "Test 4." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::fullStep, initDelay);

	// Run motor test 5
	// Define varibles for motor run test 360 turn , wave drive
	stepDelay = 0.01F;
	steps = 512;
	ccwise = false;
	verbose = true;
	initDelay = 0.01F;
	std::cout << "Test 5." << std::endl;
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
