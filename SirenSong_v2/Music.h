#pragma once

// Musical frequencies (H = sharp, b = flat, n = natural)
const float
	A3n = 220.000,
	B3n = 246.942,
	C4H = 277.183,
	D4n = 293.665,
	E4n = 329.628,
	F4H = 369.994,
	G4n = 391.995,
	A4n = 440.000;

// Song (Hundred Pipers)
const float NOTE_FREQS[] = {  // n = natural, H = sharp, b = flat
	  A4n, G4n,
	  F4H, A3n, A3n, B3n, A3n,
	  B3n, D4n, D4n, A4n,
	  A4n, F4H, F4H, E4n, D4n,
	  E4n, D4n, E4n,
	  F4H, A3n, A3n, B3n, A3n,
	  B3n, D4n, D4n, A4n,
	  A4n, F4H, E4n, F4H, E4n,
	  D4n,
	  0.000  // Rest at the end
};

const uint16_t NOTE_LENGTHS[] = {
	  500,   500,
	  1000,  500,  750,  250,  500,
	  1000,  500, 1000,  500,
	  1000,  500,  750,  250,  500,
	  2000,  500,  500,
	  1000,  500,  750,  250,  500,
	  1000,  500, 1000,  500,
	  1000,  500,  750,  250,  500,
	  2000,
	  3000  // Rest at the end
};

const int NOTE_COUNT = sizeof(NOTE_FREQS) / sizeof(float);
