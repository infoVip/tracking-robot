#include "StepperMotor.h"
#include <iostream>
#include <ctime>

using namespace std;

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

StepperMotor::StepperMotor(const unsigned int pinA, const unsigned int pinB, const unsigned int pinC,
		const unsigned int pinD, const unsigned int stepDuration, const SteppingMethod steppingMethod)
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

void StepperMotor::fullRotation(const int noOfRotations) {
	halfRotation(2*noOfRotations);
}

void StepperMotor::halfRotation(const int noOfHalfRotations) {
	int factor = noOfHalfRotations < 0 ? -1 : 1;
	for (unsigned int i = 0; i < abs(noOfHalfRotations); i++) {
		switch (steppingMethod) {
			case HALF_STEP:
				step(4 * 512 * factor);
				break;
			default:
				step(2 * 512 * factor);
				break;
		}
	}
}

void StepperMotor::setStepDurartion(const unsigned int stepDuration) {
	this->stepDuration = stepDuration;
}

void StepperMotor::setSteppingMethod(const SteppingMethod steppingMethod) {
	this->steppingMethod = steppingMethod;
}

void StepperMotor::step(const int noOfSteps) {
	if (noOfSteps > 0) {
		for (int currentStep = noOfSteps; currentStep > 0; currentStep--) {
			int currentSequenceNo = currentStep % 8;
			writeSequence(currentSequenceNo);
		}
	} else {
		for (int currentStep = 0; currentStep < abs(noOfSteps); currentStep++) {
			int currentSequenceNo = currentStep % 8;
			writeSequence(currentSequenceNo);
		}
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

void StepperMotor::performDemo() {
	cout << "Full rotation clockwise in wave drive method with stepDuration=3ms... " << endl;
	clock_t begin = clock();
	setSteppingMethod(WAVE_DRIVE);
	setStepDurartion(3);
	fullRotation(1);
	clock_t end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Full rotation counterclockwise in full step method with stepDuration=3ms... " << endl;
	begin = clock();
	setSteppingMethod(FULL_STEP);
	setStepDurartion(3);
	fullRotation(-1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Full rotation clockwise in half step method with stepDuration=3ms... " << endl;
	begin = clock();
	setSteppingMethod(HALF_STEP);
	setStepDurartion(3);
	fullRotation(1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Half rotation counterclockwise in ful step method with stepDuration=3ms... " << endl;
	begin = clock();
	setSteppingMethod(FULL_STEP);
	setStepDurartion(3);
	halfRotation(-1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;
}
