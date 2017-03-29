#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "../bgc_io.h"
#include "blmotor.h"
#include "../config.h"

// control state variables of the motor
volatile MotorControlState motorControlState;

volatile int8_t currentPhase = 0;

//rotation speed for turning
int8_t rot_Speed = 0;

volatile uint8_t tick = 0;

volatile uint8_t idx = 0; // index for load array

// acceleration function (similar to RC load curve), access with  pgm_read_byte(&percent[i]);
//const uint8_t percent[] PROGMEM = { 10, 18, 26, 33, 39, 45, 50, 55, 59, 63, 67,
//		70, 73, 75, 78, 80, 82, 83, 85, 86, 88, 89, 90, 91, 92, 93, 93, 94, 94,
//		95, 95, 96, 96, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99,
//		99, 100, 100 }; // 50 Schritte -> 49 entspricht ungefähr 5*tau -> weiterschalten in tau/10 (z. B. 10 ms)
const uint8_t percent[] PROGMEM = { 50, 54, 58, 61, 63, 64, 65, 66, 67, 68,	69,  // 33, 35, 38, 40, 42, 45, 50, 55, 59, 63, 67,
		70, 73, 75, 78, 80, 82, 83, 85, 86, 88, 89, 90, 91, 92, 93, 93, 94, 94, 95, 95, 96, 96, 97, 97, 97, 98, 98, 98,
		98, 98, 99, 99, 99, 99, 99, 99, 99, 100, 100 };

// oder const __flash uint8_t percent[] = ... testen
// -std=gnu99 o.ä. notwendig, s. https://www.mikrocontroller.net/articles/AVR-GCC-Tutorial#Flash_mit_flash_und_Embedded-C
// const __flash uint8_t percent[] ={ 10,18,26,33,39,45,50,55,59,63,67,70,73,75,78,80,82,83,85,86,88,89,90,91,92,93,93,94,94,95,95,96,96,97,97,97,98,98,98,98,98,99,99,99,99,99,99,99,100,100};

const uint8_t timerUs = 4; // Ttimer in microseconds
const uint16_t max_speed_delay = MAX_SPEED_DELAY;
const uint16_t max_speed_ocr_val = MAX_SPEED_OCR_VAL * 1.2f;

const long int factor = (250000L * 60) / ((6 * POLENUMBER));

volatile uint8_t accelFlag = 0;
volatile uint8_t decelFlag = 0;

//--------------------------------------------------------------------
//Motor control stuff-------------------------------------------------
//--------------------------------------------------------------------
void bl_setup() {
	// set initial state
	motorControlState.direction = 1;
	motorControlState.desiredDirection = 1;
	motorControlState.speed = 0;
	motorControlState.desiredSpeed = 0;
	motorControlState.enabled = 0;

	// Set Motor Ports to output
	MOT0_DDR |= (1 << MOT0A_BIT) | (1 << MOT0B_BIT) | (1 << MOT0C_BIT);
//	MOT1_DDR |= (1 << MOT1A_BIT) | (1 << MOT1B_BIT) | (1 << MOT1C_BIT);

	cli();
	// F(timer)=16000000/64 = 250000Hz, CTC mode
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
	// initialize counter
	TCNT1 = 0;
	// initialize compare value
	OCR1A = (uint16_t) (max_speed_ocr_val * (101 - pgm_read_byte(&percent[0])));
	TIMSK1 |= (1 << OCIE1A);
	// disable arduino standard timer interrupt
//	TIMSK0 &= ~(1 << TOIE1);
	sei();

	// switch off PWM Power
	motorPowerOff();
}

// switch off motor power
void motorPowerOff() {
	MOT0_PORT &= ~((1 << MOT0A_BIT) | (1 << MOT0B_BIT) | (1 << MOT0C_BIT));
	//TODO: correct?
}

void setMotorControl(MotorControlState control) {
	motorControlState.desiredDirection = control.desiredDirection;
	motorControlState.desiredSpeed = control.desiredSpeed;
	motorControlState.direction = control.direction;
	motorControlState.speed = control.speed;
	motorControlState.enabled = control.enabled;
	motorControlState.brake = control.brake;
}

void commutate() {
	if (motorControlState.direction > 0) {
		currentPhase++;
		if (currentPhase < 0)
			currentPhase = 0;
		if (currentPhase > 5)
			currentPhase = 0;
	} else {
		currentPhase--;
		if (currentPhase < 0)
			currentPhase = 5;
		if (currentPhase > 5)
			currentPhase = 5;
	}

	switch (currentPhase) {
	case 0: // 100
		MOT0_PORT |= (1 << MOT0A_BIT);
		MOT0_PORT &= ~((1 << MOT0B_BIT) | (1 << MOT0C_BIT));
		break;
	case 1: // 110
		MOT0_PORT |= (1 << MOT0A_BIT) | (1 << MOT0B_BIT);
		MOT0_PORT &= ~(1 << MOT0C_BIT);
		break;
	case 2: // 010
		MOT0_PORT |= (1 << MOT0B_BIT);
		MOT0_PORT &= ~((1 << MOT0A_BIT) | (1 << MOT0C_BIT));
		break;
	case 3: // 011
		MOT0_PORT |= (1 << MOT0B_BIT) | (1 << MOT0C_BIT);
		MOT0_PORT &= ~(1 << MOT0A_BIT);
		break;
	case 4: // 001
		MOT0_PORT |= (1 << MOT0C_BIT);
		MOT0_PORT &= ~((1 << MOT0A_BIT) | (1 << MOT0B_BIT));
		break;
	case 5: // 101
		MOT0_PORT |= (1 << MOT0A_BIT) | (1 << MOT0C_BIT);
		MOT0_PORT &= ~(1 << MOT0B_BIT);
		break;
	default: // 000
		MOT0_PORT &= ~((1 << MOT0A_BIT) | (1 << MOT0B_BIT) | (1 << MOT0C_BIT));
	}
}

void motorControl() {
//	if (!motorControlState.enabled) {
//		motorPowerOff();
//		idx = 0;
//		motorControlState.speed = 0;
//
//	} else {
//		if (!motorControlState.brake) {
//			motorControlState.speed = factor / OCR1A;
//			if (motorControlState.speed < motorControlState.desiredSpeed) {
//				accelFlag = 1;
//			} else if (motorControlState.speed
//					> motorControlState.desiredSpeed) {
//				decelFlag = 1;
//			}
//		} else {
//			idx = 0;
//			motorControlState.speed = 0;
//		}
//	}
}

/* Motor Control Loop */
ISR (TIMER1_COMPA_vect) {
	if (!motorControlState.enabled) {
		motorPowerOff();
		idx = 0;
		motorControlState.speed = 0;
		OCR1A = (uint16_t) (max_speed_ocr_val * (101 - pgm_read_byte(&percent[0])));
	} else {
		if (!motorControlState.brake) {
			commutate();
			motorControlState.speed = factor / OCR1A;
			if (motorControlState.speed < motorControlState.desiredSpeed) {
				if (idx < 49) {
					//	accelerate
					idx++;
					OCR1A = (uint16_t) (max_speed_ocr_val * (101 - pgm_read_byte(&percent[idx])));
				}
			} else if (motorControlState.speed > motorControlState.desiredSpeed) {
				if (idx > 0) {
					//	decelerate
					idx--;
					OCR1A = (uint16_t) (max_speed_ocr_val * (101 - pgm_read_byte(&percent[idx])));
				}
			}
		} else {
			idx = 0;
			motorControlState.speed = 0;
			OCR1A = (uint16_t) (max_speed_ocr_val * (101 - pgm_read_byte(&percent[0])));
		}
	}

//	if (tick %2 == 0) {
//		commutate();
//	}

//	tick++;
}

void printMotorState() {
	printf("Motor state:\n");
	printf("\tmotorControlState.enabled: %i\n", motorControlState.enabled);
	printf("\tmotorControlState.brake: %i\n", motorControlState.brake);
	printf("\tspeed: %.i\n", motorControlState.speed);
	printf("\tdirection: %i\n", motorControlState.direction);
	printf("\tdesiredSpeed: %i\n", motorControlState.desiredSpeed);
	printf("\tdesiredDirection: %i\n", motorControlState.desiredDirection);
}

