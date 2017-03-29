#ifndef BLMOTOR_H
#define BLMOTOR_H
#endif

typedef struct {
	int8_t enabled;
	int8_t brake;
	int8_t direction;
	int8_t desiredDirection;
	int16_t speed;
	int16_t desiredSpeed;
} MotorControlState;

extern volatile MotorControlState motorControlState;

void MoveMotorsPhase(uint8_t motorNumber, uint8_t phase, uint16_t power, int8_t direction);
void motorPowerOff();
void bl_setup();
void setMotorControl(MotorControlState control);
void motorControl();
void printMotorState();

