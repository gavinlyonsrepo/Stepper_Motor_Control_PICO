/**
 * @file main.cpp
 * @brief test file Stepper motor control library for Raspberry Pi Pico.
		Bipolar Nema Stepper motor A3967 Easy Driver 
	@details code for an optional push button to gnd which can be used as software 
		stop on motor movement included. 
 */
#include <vector>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stepper_motor_control/stepper_motor_control.hpp"

// motor setup 
// Define variables for motor control
const uint8_t direction = 0 ; //Direction -> GPIO Pin
const uint8_t  step = 1 ;     //Step -> GPIO Pin
int msPins[2] = {2, 3};       //Microstep Resolution MS1-MS2 -> GPIO Pins

// Create motor instance
StepMotorControlEasy motor(direction, step, msPins);
//StepMotorControlEasy motor(direction, step); // without MS PINS GPIO , hardwire

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
	float stepDelay = 5;
	int steps = 200;
	bool ccwise = false;
	bool verbose = true;
	std::string resolution = "Full";
	float initDelay = 1;
	std::cout << "Test 1. 360 degree turn, full step." << std::endl;
	motor.motorMove(stepDelay, steps, ccwise, verbose, resolution, initDelay);

	// run motor test 2
	// Define variables for motor run test: 180 turn 
	steps = 100;
	ccwise = true;
	initDelay = 2000;
	std::cout << "Test 2. 180 degree turn, counter clockwise, full step." << std::endl;
	motor.motorMove(stepDelay, steps, ccwise, verbose, resolution, initDelay);

	// run motor test 3
	// Define variables for motor run test: 180 turn 
	stepDelay = 50;
	std::cout << "Test 3. Same as two but slower step." << std::endl;
	motor.motorMove(stepDelay, steps, ccwise, verbose, resolution, initDelay);

	// run motor test 4
	// Define variables for motor run test:180 turn 1/2 step resolution
	steps = 200;  // full 1/2 turn 400
	ccwise = false;
	resolution = "Half";
	std::cout << "Test 4. 180 degree turn, step resolution 1/2 " << std::endl;
	motor.motorMove(stepDelay, steps, ccwise, verbose, resolution, initDelay);

	// run motor test 5
	// Define variables for motor run test:90 turn 1/4 step resolution
	steps = 200;  // full 1/4 turn 800
	resolution = "1/4"; 
	std::cout << "Test 5. 90 degree turn, step resolution 1/4" << std::endl;
	motor.motorMove(stepDelay, steps, ccwise, verbose, resolution, initDelay);

	// run motor test 6
	// Define variables for motor run test:90 turn 1/8 step resolution
	steps = 400;  // full 1/4 turn 1600
	resolution = "1/8";
	std::cout << "Test 6. 90 degree turn, step resolution 1/8" << std::endl;
	motor.motorMove(stepDelay, steps, ccwise, verbose, resolution, initDelay);

	EndTest();
	return 0;
}

void EndTest(void)
{
	motor.motorStop(); 	// Stop motor after test
	motor.motorDeInit(); //de- init motor 
	gpio_deinit(BUTTON_PIN); //de- init stop button
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

// Interrupt callback function, [[optional 
void button_callback(uint gpio, uint32_t events) 
{
	motor.setMotorStop(true);
}
