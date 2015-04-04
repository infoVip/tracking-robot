#include "StepperMotor.h"
#include <iostream>

const bool StepperMotor::WAVE_DRIVE_MOTOR_SEQUENCE[][4] = {
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW },
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW }
};

const bool StepperMotor::FULL_STEP_MOTOR_SEQUENCE[][4] = {
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW },
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW }
};

const bool StepperMotor::HALF_STEP_MOTOR_SEQUENCE[][4] = {
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW },
		{ HIGH, LOW,  LOW,  HIGH }
};

StepperMotor::StepperMotor(const int pinA, const int pinB, const int pinC,
		const int pinD, const int stepDuration, const SteppingMethod steppingMethod)
{
	inputPins[0] = pinA;
	inputPins[1] = pinB;
	inputPins[2] = pinC;
	inputPins[3] = pinD;

	for (int i = 0; i < 4; i++) {
		pinMode(inputPins[i], OUTPUT);
	}

	this->stepDuration = stepDuration;
	this->steppingMethod = steppingMethod;
}

void StepperMotor::fullRotationCW(const unsigned int noOfRotations) {
	for (unsigned int i = 0; i < noOfRotations; i++) {
		switch (steppingMethod) {
			case HALF_STEP:
				stepCW(8 * 512);
				break;
			default:
				stepCW(4 * 512);
				break;
		}
	}
}

void StepperMotor::fullRotationCCW(const unsigned int noOfRotations) {
	for (unsigned int i = 0; i < noOfRotations; i++) {
		switch (steppingMethod) {
			case HALF_STEP:
				stepCCW(8 * 512);
				break;
			default:
				stepCCW(4 * 512);
				break;
		}
	}
}

void StepperMotor::setStepDurartion(const unsigned int stepDuration) {
	this->stepDuration = stepDuration;
}

void StepperMotor::stepCW(const int noOfSteps) {
	for (int currentStep = noOfSteps; currentStep > 0; currentStep--) {
		int currentSequenceNo = currentStep % 8;
		writeSequence(currentSequenceNo);
	}
}

void StepperMotor::stepCCW(const int noOfSteps) {
	for (int currentStep = 0; currentStep < noOfSteps; currentStep++) {
		int currentSequenceNo = currentStep % 8;
		writeSequence(currentSequenceNo);
	}
}

void StepperMotor::writeSequence(const unsigned int sequenceNo) {
	for (int i = 0; i < 4; i++) {
		switch(steppingMethod) {
			case WAVE_DRIVE:
				digitalWrite(inputPins[i], WAVE_DRIVE_MOTOR_SEQUENCE[sequenceNo][i]);
				break;
			case FULL_STEP:
				digitalWrite(inputPins[i], FULL_STEP_MOTOR_SEQUENCE[sequenceNo][i]);
				break;
			default:
				digitalWrite(inputPins[i], HALF_STEP_MOTOR_SEQUENCE[sequenceNo][i]);
				break;
		}
	}
	delay(stepDuration);
	for (int i = 0; i < 4; i++) {
		digitalWrite(inputPins[i], LOW);
	}
}
