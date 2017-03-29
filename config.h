/*
 * config.h
 *
 *  Created on: 24.03.2017
 *      Author: Wohn
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define MOTORUPDATE_FREQ 15625 // 31250, 15625, 10417
#define POLENUMBER 7
#define KV 80L
#define VOLTAGE 13
#define MAX_RPM (KV*VOLTAGE) // unloaded! TODO: reduce
#define TIMER_US 4
#define COM_STEPS_PER_TURN (POLENUMBER * 6L) // 6 phase commutation
#define MAX_SPEED_COM_FREQ ((MAX_RPM*COM_STEPS_PER_TURN) / 60)
#define MAX_SPEED_DELAY (1000000L / MAX_SPEED_COM_FREQ) // in microseconds
#define MAX_SPEED_OCR_VAL ((MAX_SPEED_DELAY / TIMER_US) - 1)

#define TICK_DIVIDER_MAX 200

#endif /* CONFIG_H_ */
