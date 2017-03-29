/*
 * bgc_io.h
 *
 *  Created on: 08.03.2017
 *      Author: Tobias Ellermeyer
 */

#ifndef BGC_IO_H_
#define BGC_IO_H_

//--------- LED -----------
#define LEDG_PORT    PORTB
#define LEDG_DDR     DDRB
#define LEDG_BIT     PB5

//--------- MOTORS --------------
// Motor 0 PWM
#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B

// Motor 1 PWM
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

// Motor IO Ports
// OC1 -> PB1 PB2
// OC2 -> PB3 PD3
// OC0 -> PD6 PD5
#define MOT0_PORT   PORTD
#define MOT0_DDR    DDRD
#define MOT1_PORT   PORTB
#define MOT1_DDR   DDRB

#define MOT0A_BIT   PD6
#define MOT0B_BIT   PD5
#define MOT0C_BIT   PD3

#define MOT1A_BIT   PB3
#define MOT1B_BIT   PB2
#define MOT1C_BIT   PB1

#endif /* BGC_IO_H_ */
