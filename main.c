#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "stdlib.h"
#include "math.h"
#include "uart/uart.h"
#include "bgc_io.h"
#include "bldc/blmotor.h"
#include "mpu6050/mpu6050registers.h"
#include "mpu6050/mpu6050.h"
#include "config.h"

#define 	F_CPU   16000000UL
#define UART_BAUD_RATE 57600

volatile int freqCounter = 0;
volatile int msCounter = 0;

void timer2_setup();

int main() {
	uart_stdio();
//	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	bl_setup();
	timer2_setup();

	mpu6050_init();
//
//	uint8_t testConnection = mpu6050_testConnection();
//
//	printf("mpu6050 test: %i\n", testConnection);
//	fflush(stdin);

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    double axg = 0;
    double ayg = 0;
    double azg = 0;
    double gxds = 0;
    double gyds = 0;
    double gzds = 0;

	while (1) {
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
				mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
//		printf("mpu6050 data\n: axg: %.4f\tayg: %.4f\tazg: %.4f\tgxds: %.2f\tgyds: %.2f\tgzds: %.2f \n", axg, ayg, azg, gxds, gyds, gzds);
//		printf("%.4f %.4f\r", axg, gxds);
//		_delay_ms(10);

		char itmp[10];
				ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
				ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
				ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
				ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
				ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
				ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
				uart_puts("\r\n");

				dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
				dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
				dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
				dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
				dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
				dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
				uart_puts("\r\n");

				uart_puts("\r\n");

				_delay_ms(1000);
	}

	motorControlState.enabled = 1;
	motorControlState.brake = 0;
	motorControlState.desiredDirection = 1;
	motorControlState.desiredSpeed = 1000;
	motorControlState.direction = 1;
	motorControlState.speed = 0;

	int i = 0;
	do {
		if (i < 1) {
			_delay_ms(1500);
			printMotorState();

			_delay_ms(1500);
			motorControlState.brake = 1;
			printMotorState();
			_delay_ms(200);
			motorControlState.enabled = 0;
			_delay_ms(300);
			motorControlState.enabled = 1;
			motorControlState.brake = 0;
			printMotorState();

			//
			_delay_ms(3000);
			motorControlState.enabled = 0;
			printMotorState();

			_delay_ms(2000);
			motorControlState.direction = 0;
			motorControlState.enabled = 1;
//			printMotorState();
//			printf("Reverse\n");
//			setMotorData(0, 1200);
			_delay_ms(3000);
			motorControlState.enabled = 0;
			i++;
		}

//		if (i == 1) {
//			setMotorData(0, 0);
//			motorPowerOff();
//		}

	} while (1);

	return 0;
}

void timer2_setup() {
	cli();
	TCNT2 = 0;
	TCCR2A |= (1 << WGM21); // Configure timer 2 for CTC mode
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024 (different for timer2)
	TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt
	OCR2A = 156 - 1; // 100Hz, 10ms Timer
	sei();
}

/* Main Control Loop every 10ms */
ISR(TIMER2_COMPA_vect) {
//	freqCounter++;
//	if (freqCounter % 2 == 0) {
//		motorControl();
//		freqCounter = 0;
//	}
//	motorControl();
}

///********************************/
///* Control IRQ Routine    */
///********************************/
//// is called every 31.5us (32)
//// minimize interrupt code (20 instructions)
//ISR( TIMER1_OVF_vect ) {
//	freqCounter++;
//
////	if (freqCounter >= (PWM_FACTOR * 1000 / MOTORUPDATE_FREQ)) {
////		freqCounter = 0;
////
////		// motor control at MOTORUPDATE_FREQ rate
////		doMotorControl();
////
//////    PWM_A_MOTOR0 = pwm_a_motor0;
//////    PWM_B_MOTOR0 = pwm_b_motor0;
//////    PWM_C_MOTOR0 = pwm_c_motor0;
//////
//////    PWM_A_MOTOR1 = pwm_a_motor1;
//////    PWM_B_MOTOR1 = pwm_b_motor1;
//////    PWM_C_MOTOR1 = pwm_c_motor1;
//////
//////    // update event
//////    motorUpdate = true;
////	}
//
//	if ((freqCounter & 0x01f) == 0 && msCounter % 1000 == 0) { // every 1000ms
//		msCounter = 0;
//		printMotorState();
//	}
//	// care for standard timers every 1 ms
//	if ((freqCounter & 0x01f) == 0) { // every ms
//		msCounter++;
////    TIMER0_isr_emulation();
//	}
//
//}

