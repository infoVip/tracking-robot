#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <wiringPi.h>

class StepperMotor {
public:
	enum SteppingMethod {WAVE_DRIVE, FULL_STEP, HALF_STEP};

	StepperMotor(const int pinA, const int pinB, const int pinC,
			const int pinD, const int stepDuration, const SteppingMethod steppingMethod = HALF_STEP);

	void setStepDurartion(const unsigned int stepDuration);

	void fullRotationCW(const unsigned int noOfRotations);
	void fullRotationCCW(const unsigned int noOfRotations);

private:
	static const bool WAVE_DRIVE_MOTOR_SEQUENCE[][4];
	static const bool FULL_STEP_MOTOR_SEQUENCE[][4];
	static const bool HALF_STEP_MOTOR_SEQUENCE[][4];

	int stepDuration;
	int inputPins[4];
	SteppingMethod steppingMethod;

	void stepCW(const int noOfSteps);
	void stepCCW(const int noOfSteps);

	void writeSequence(const unsigned int sequenceNo);
};

#endif
