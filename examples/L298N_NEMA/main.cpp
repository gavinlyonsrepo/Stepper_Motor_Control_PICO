/**
 * @file main.cpp
 * @brief test file Stepper motor control library for Raspberry Pi Pico.
			Bipolar Nema stepper motor with L298N
 */
#include <vector>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stepper_motor_control/stepper_motor_control.hpp"

// motor setup 
// Define variables for motor control
std::vector<uint> motorPins = {0, 1, 2, 3}; // GPIO pins list
// Create motor instance
StepMotorControl motor(StepMotorControl::L298N_NEMA, motorPins);

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
	uint32_t stepDelay = 100;
	int steps = 50;
	bool ccwise = false;
	bool verbose = true;
	uint32_t  initDelay = 1000;
	std::cout << "Test 1. 360 degree turn, full step." << std::endl;
	motor.motorRun(stepDelay, steps, ccwise, verbose, StepMotorControl::fullStep, initDelay);

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
	std::cout << "Motor test L298N controller, bipolar Nema motor, Started." << std::endl; 
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
