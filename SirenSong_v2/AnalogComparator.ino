
/*******************************************************************************
 * Analog Comparator / Tachometer
 * This file is part of Siren Song, created by Thomas Seary (subatomicsushi), April 2015.
 * Free for any use at own risk.
 *******************************************************************************/

void initAnalogComparator() {
	cbi(ACSR, ACD);
	cbi(ACSR, ACBG);
	cbi(ACSR, ACIE);
	cbi(ACSR, ACIC);

	// ACIS1 | ACIS0 | Interrupt Mode
	//   0       0       Toggle
	//   0       1       Reserved.
	//   1       0       Falling
	//   1       1       Rising
	sbi(ACSR, ACIS1);  // Interrupt mode 1
	sbi(ACSR, ACIS0);  // Interrupt mode 0

	sbi(DIDR1, AIN1D);
	sbi(DIDR1, AIN0D);
}

// Start the analog comparator to enable interrupts.
void startAnalogComparator() {
	sbi(ACSR, ACIE);
}

// Stop the analog comparator to disable interrupts.
void stopAnalogComparator() {
	cbi(ACSR, ACIE);
}

// The interrupt for the analog comparator. This is called every time the tachometer toggles.
// tachometerDeltaMicros is the difference of the current time and the oldest value in the buffer.
// We set it here, and not in the main loop, to avoid synchronization errors.
ISR(ANALOG_COMP_vect) {
	// Ignore if it's too soon after the last tick
	if (micros() - getLastTickMicros() < TACHO_DELTA_MICROS_MIN) {
		return;
	}

	// Next index (faster than mod)
	if (++tachoBufferIndex >= TACHOMETER_BUFFER_SIZE) {
		tachoBufferIndex = 0;
	}

	// tachoDeltaMicros is difference of newest value and oldest value.
	// Newest value is guaranteed >= oldest value, except during rollover.
	tachoDeltaMicros = micros() - tachoBuffer[tachoBufferIndex];
	tachoBuffer[tachoBufferIndex] = micros();

	// DEBUG
	static bool ledState = false;
	digitalWrite(LED_BUILTIN, ledState = !ledState);
}

// Gets the absolute time of the last tachometer tick.
uint32_t getLastTickMicros() {
	return tachoBuffer[tachoBufferIndex];
}
