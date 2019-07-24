/*******************************************************************************
 * Siren Song v2
 * This file is part of Siren Song, created by Thomas Seary,
 * April 2015, updated July 2019.
 * Free for any use at own risk.
 *
 * This project features a mechanical rotary siren (i.e. an air-raid siren),
 * retro-fitted with an optical tachometer. Closed loop speed control is used
 * to vary the siren's speed and thus the audible frequency. By changing the
 * target frequency at the appropriate times, the siren can be made to play music.
 * Tunes written for bagpipes are especially suitable, since both the air-raid
 * siren and the bagpipes are scary noise-makers of war.
 *
 * The control algorithm is optimized for a fast-settling step response, since
 * this improves the minimum note length that can be achieved.
 *
 * The controller is based on the Arduino Duemilanove. The analog signal from
 * the tachometer is put through a 1000 Hz low-pass RC filter to pin 7, and the
 * same signal is put through a 1 Hz low-pass RC filter to pin 6. This means that
 * the analog comparator toggles whenever the tachometer signal crosses its own
 * average. The time between toggles is measured and this provides the frequency
 * feedback.
 *******************************************************************************/

#include "Music.h"
#include <PID_v1.h>

 // Macros for setting bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Motor control
const byte MOTOR_PWM_PIN = 9;  // PWN on pin 9 is controlled by timer 1, which we set for 3.9 kHz
const byte MOTOR_BRAKE_PIN = 8;  // This controls an N-ch MOSFET to short the motor terminals
const byte MOTOR_POWER_MAX = 255;
const double MOTOR_BRAKE_THRESHOLD = -50.0;	// Use the brake if the PID controller requests a value less than this

// Main loop period
const int CYCLE_PERIOD_MILLIS = 1;

// The times of the most recent tachometer ticks are stored in microseconds in this buffer.
// This provides better accuracy for the audible frequency calculation.
const byte TACHOMETER_BUFFER_SIZE = 2;  // It is recommended to make this a multiple of the tachometer ticks per rotation to negate encoder jitter.
volatile uint32_t tachoBuffer[TACHOMETER_BUFFER_SIZE];
volatile byte tachoBufferIndex = 0;  // The index of the oldest sample
volatile uint32_t tachoDeltaMicros;  // The difference of the current time and the oldest sample in the buffer.

// Frequency feedback
// See initAnalogComparator() to determine if this is on rising edge or on toggle
const float TACHO_TO_AUDIO_FREQ = 4.0f;	// Audio cycles per tachometer tick
float audioFrequency = 0.0f;

// Equivalent to 24 kHz max audio frequency
const uint32_t TACHO_DELTA_MICROS_MIN = 42 * TACHO_TO_AUDIO_FREQ;
const uint32_t SPEED_STALE_MICROS = 250000;  // If it's been this long with no ticks, the motor is stopped.

// PID Control
// Define variables we'll be connecting to
double
pidInAudioFreq = 0.0,
pidOutMotorPWM = 0.0,
pidRefAudioFreq = 0.0;
const double
K_P = 2.0,
K_I = 0.1,
K_D = 0.1;
PID sirenPID(&pidInAudioFreq, &pidOutMotorPWM, &pidRefAudioFreq, K_P, K_I, K_D, DIRECT);

uint16_t noteIndex = 0;	// Counter to change notes
const bool REPEAT_SONG = false;
const uint16_t REST_MILLIS = 50;	// Cut this much off the end of each note
bool resting = false;

/*******************************************************************************
 * Setup
 *******************************************************************************/

 // Setup starts the serial port, tachometer interrupts etc. and PID controller.
void setup() {
	// Use high-speed serial to reduce the effect on timing
	Serial.begin(500000);

	// Set the LED pin as output
	pinMode(LED_BUILTIN, OUTPUT);

	// Start the analog comparator to read the tachometer
	initAnalogComparator();
	startAnalogComparator();

	// Set up the motor
	pinMode(MOTOR_BRAKE_PIN, OUTPUT);
	digitalWrite(MOTOR_BRAKE_PIN, LOW);  // Brakes off by default
	setPwmFrequency(MOTOR_PWM_PIN, 8);  // (31250 / 8 = 3906 Hz)

	// Initialize PID control
	sirenPID.SetOutputLimits(-255.0, 255.0);
	sirenPID.SetSampleTime(1);	// Fastest possible
	sirenPID.SetMode(AUTOMATIC);

	// Set first note
	pidRefAudioFreq = 440.0;
}

/*******************************************************************************
 * Main Loop
 *******************************************************************************/

 // The main loop converts the tachometer reading to an audible frequency.
 // It calculates the error between the actual frequency and the target
 // frequency, then adjusts the motor speed to compensate.
void loop() {
	// Change the setpoint to the next note
	const float OCTAVE = 2.0f;	// Play higher
	const float TEMPO = 1.5f;	// Play faster
	static uint32_t noteChangeMillis = NOTE_LENGTHS[noteIndex];
	if (millis() >= noteChangeMillis) {
		// Set the note frequency and duration
		if (resting) {
			// Next note
			if (++noteIndex >= NOTE_COUNT) {
				if (REPEAT_SONG) {
					// Go back to the beginning
					noteIndex = 0;
				} else {
					// Turn off the motor and stop
					setMotorPower(0, false);
					while (true);
				}
			}

			// New note
			pidRefAudioFreq = OCTAVE * NOTE_FREQS[noteIndex];
			noteChangeMillis += (NOTE_LENGTHS[noteIndex] - REST_MILLIS) / TEMPO;
			resting = false;
		} else {
			// Rest at the end of a note
			pidRefAudioFreq = 0.0;
			noteChangeMillis += REST_MILLIS / TEMPO;
			resting = true;
		}

		// DEBUG Print note
		Serial.print(noteIndex);
		Serial.print('\t');
		Serial.print(pidRefAudioFreq);
		Serial.print('\t');
		Serial.println(noteChangeMillis - millis());
	}

	calculateAudioFrequency();

	// Set the speed per the PID controller
	pidInAudioFreq = audioFrequency;
	sirenPID.Compute();
	setMotorPowerDouble(pidOutMotorPWM);

	// Timeout
	/*if (millis() > 4500) {
		setMotorPower(0, false);
		while (true);
	}*/
}
