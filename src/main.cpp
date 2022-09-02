#include <Arduino.h>
#include <CircularBuffer.h>

// #define DEBUG
#define RELEASE

#define SIG A6 // input signal
#define THR A5 // speaker enabler
#define TX1 A4 // transmit of speaker
#define TX2 A3 // other transmite of speaker

#define N_PULSE 8 // number of pulses to expect
#define S_NUM 1000 // number of samples to correlate
#define K_SZ 4 // size of correlation kernel

const float snd_spd = 343/2; // speed of sound, divided by 2 since covering double the distance

CircularBuffer<int, S_NUM> buff; // store samples
CircularBuffer<int, S_NUM> tims; // store timing of samples
int out[S_NUM]; // store results of correlation

#ifdef DEBUG
#define TST_N 1000
int tst[1000];
int tms[1000];
#endif

int kernel[K_SZ] = {0, 250, 375, 375}; // roughly what the impulse looks like

void setup() {
	// pair for triggering transmit speaker
	pinMode(TX1, OUTPUT);
	pinMode(TX2, OUTPUT);
	// transmit enable
	pinMode(THR, OUTPUT);

	digitalWrite(TX1, LOW);
	digitalWrite(TX2, LOW);
	digitalWrite(THR, LOW);

	// indictor LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	Serial.begin(115200); // for printing distance
}

void loop() {
	memset(out, 0, sizeof(out));
	digitalWrite(THR, HIGH); // enabled the speaker
	digitalWrite(LED_BUILTIN, LOW); // turn on indicator LED
	for (int i = 0; i < N_PULSE; i++) { // create transmit pulse by toggling speaker 8 times
		// switched to port manipulation instead of arduino API. appreciably faster on occiliscope.
		GPIOA->BSRR |= (1 << 4);
		GPIOA->BRR |= (1 << 3);
		delayMicroseconds(12);
		GPIOA->BRR |= (1 << 4);
		GPIOA->BSRR |= (1 << 3);
		delayMicroseconds(12);
	}
	digitalWrite(LED_BUILTIN, HIGH); // turn off indicator LED
	digitalWrite(THR, LOW); // disable the speaker

	int start = micros();

	GPIOA->BSRR |= (1 << 4) | (1 << 3); // make sure the speaker is off

	for (int i = 0; i < S_NUM; i++) {
		buff.push(analogRead(SIG));
		tims.push(micros());
		delayMicroseconds(6); // delay to make it 80KHz (takes into account delay created by reading sample and time)
	}

	#ifdef RELEASE
	// Correlate input
	for (int i = 0; i < buff.size() - K_SZ; i++) { // we do not start from edge of kernel since it is extremely unlikely to have the echo we are looking for to appear there.
		for (int j = 0; j < K_SZ && buff.size() <= K_SZ; j++) {
			out[i] += buff[j] * kernel[j]; // apply kernel
		}
		buff.shift(); // remove first element so we apply kernel on the next set
	}

	int max = 0;
	int max_idx = 0;
	for (int i = 0; i < S_NUM; i++) { // search for the biggest peak of the correlation
		if (out[i] > max) {
			max_idx = i; // save index of peak
		}
	}

	float dist = (tims[max_idx] - start) / snd_spd; // get distance by dividing time of flight by speed of sound

	Serial.print(dist); // prints distance
	Serial.print(" mm\n"); // print units
	#endif

	#ifdef DEBUG
	for (int i = 0; i < S_NUM; i++) {
		Serial.print(tms[i]);
		Serial.print(',');
		Serial.print(buff[i]);
		Serial.print('\n');
	}
	#endif

	delay(500);
}
