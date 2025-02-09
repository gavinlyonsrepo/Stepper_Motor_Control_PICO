/**
 * @file main.cpp
 * @brief test file Stepper motor control library for Raspberry Pi Pico.
		Bipolar Nema Stepper motor DRV8825 controller
	@details code for an optional push button to gnd which can be used as software
		stop on motor movement included.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stepper_motor_control/stepper_motor_control.hpp"

// motor setup
// Define variables for motor control
uint8_t direction = 0; //Direction -> GPIO Pin
uint8_t  step = 1;     //Step -> GPIO Pin
std::vector<int> msPins = {2, 3, 4}; //Microstep Resolution MS1-MS3 -> GPIO Pins
std::string MotorType = "DRV8825";
// Create motor instance
StepMotorControlMicro motor(direction, step, msPins, MotorType);
//StepMotorControlMirco motor(direction, step, MotorType); // without MS PINS GPIO , hardwire

// Emergency stop pushbutton, optional.
#define BUTTON_PIN 15  // GPIO pin for button

void Setup(void);
void EndTest(void);
void button_callback(uint gpio, uint32_t events) ; // Emergency stop pushbutton, optional.


int main()
{
	Setup();
	//motorGo(clockwise, steptype", steps, stepdelay, verbose, initdelay)
	std::cout << " 1 Full 180 turn Test1" << std::endl;
	motor.motorGo(false, "Full", 100, 10, true, 50); //~ 2 seconds
	busy_wait_ms(1000);
	std::cout << " 2 Full 180 clockwise Test2" << std::endl;
	motor.motorGo(true, "Full", 100, 10, true, 50); // ~2 seconds
	busy_wait_ms(1000);
	std::cout << " 3 1/8 90 Test3" << std::endl;
	motor.motorGo(false, "1/8" , 400, 5, true, 50); // ~4 seconds
	busy_wait_ms(1000);
	std::cout << " 4  1/32 90 clockwise Test4" << std::endl;
	motor.motorGo(true, "1/32" , 1600, 5, true, 50); // ~16 seconds
	busy_wait_ms(1000);

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
	std::cout << std::endl;
	std::cout << "Motor test Started. DRV8825" << std::endl;
	//init motor
	motor.motorInit();

	//init pushbutton emergency stop, optional.
	gpio_init(BUTTON_PIN);
	gpio_set_dir(BUTTON_PIN, GPIO_IN);
	gpio_pull_up(BUTTON_PIN);  // Enable pull-up resistor
	// Attach interrupt on falling edge (button press)
	gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL , true, &button_callback);
}

// Interrupt callback function, optional
void button_callback(uint gpio, uint32_t events)
{
	motor.setMotorStop(true);
}
