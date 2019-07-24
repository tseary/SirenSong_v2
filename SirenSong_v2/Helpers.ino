
/*******************************************************************************
 * Helpers
 * This file is part of Siren Song, created by Thomas Seary (subatomicsushi), April 2015.
 * Free for any use at own risk.
 *******************************************************************************/

const int FREQUENCY_SETTLING_PERIOD = 100;         // The period of checks for settling in millis
const float FREQUENCY_SETTLING_TOLERANCE = 1.001;  // The ratio between consecutive frequency checks at which the motor is considered to have settled
const byte FREQUENCY_SETTLING_COUNT = 3;           // Settling is complete after this many consecutive calls to isFrequencySettled() return true

// Calculates the audio frequency and stores the result in audioFrequency.
// The previous value is moved to previousAudioFrequency.
void calculateAudioFrequency() {
	if (micros() > getLastTickMicros() + SPEED_STALE_MICROS ||
		tachoDeltaMicros < TACHO_DELTA_MICROS_MIN) {
		// The speed is stale, the motor is stopped
		audioFrequency = 0.0;
	} else {
		audioFrequency = TACHO_TO_AUDIO_FREQ *
			TACHOMETER_BUFFER_SIZE * (1000000.0f / tachoDeltaMicros);                // Audio frequency
	}
}

void setMotorPowerDouble(double power) {
	if (isnan(power)) power = 0.0;
	setMotorPower((uint8_t)round(max(0.0, min(power, 255.0))),
		power < MOTOR_BRAKE_THRESHOLD);
}

// If brakeOn is true, power level is forced to zero
void setMotorPower(uint8_t power, bool brakeOn) {
	if (brakeOn) {
		analogWrite(MOTOR_PWM_PIN, 0);
		digitalWrite(MOTOR_BRAKE_PIN, HIGH);
	} else {
		digitalWrite(MOTOR_BRAKE_PIN, LOW);
		power = min(power, MOTOR_POWER_MAX);  // Clamp motor speed
		analogWrite(MOTOR_PWM_PIN, power);
	}
}

// Calculates a rolling average.
double smooth(double smoothing, double average, double last) {
	return smoothing * average + (1.0 - smoothing) * last;
}

// Determines if the motor is at either maximum or minimum speed.
/*boolean isSaturated() {
  return abs(motorSpeed - MOTOR_SPEED_MIN) < 0.5 || abs(motorSpeed - MOTOR_SPEED_MAX) < 0.5;
}*/

// Returns true if the given ratio is within the given tolerance, above or below.
// e.g. For a tolerance of 1.10, ratios from 0.91 to 1.10 return true.
boolean isRatioWithinTolerance(float ratio, float tolerance) {
	return max(ratio, 1.0 / ratio) <= tolerance;
}

// Sets up fast PWM on the motor pin
void setPwmFrequency(int pin, int divisor) {
	byte mode;
	if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
		switch (divisor) {
		case 1: mode = 0x01; break;
		case 8: mode = 0x02; break;
		case 64: mode = 0x03; break;
		case 256: mode = 0x04; break;
		case 1024: mode = 0x05; break;
		default: return;
		}
		if (pin == 5 || pin == 6) {
			TCCR0B = TCCR0B & 0b11111000 | mode;
		} else {
			TCCR1B = TCCR1B & 0b11111000 | mode;
		}
	} else if (pin == 3 || pin == 11) {
		switch (divisor) {
		case 1: mode = 0x01; break;
		case 8: mode = 0x02; break;
		case 32: mode = 0x03; break;
		case 64: mode = 0x04; break;
		case 128: mode = 0x05; break;
		case 256: mode = 0x06; break;
		case 1024: mode = 0x7; break;
		default: return;
		}
		TCCR2B = TCCR2B & 0b11111000 | mode;
	}
}
