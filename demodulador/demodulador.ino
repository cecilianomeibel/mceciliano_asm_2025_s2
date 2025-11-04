//DEMODULADOR

#include <arduino.h>
#include <arduinoFFT.h>


const uint16_t samples = 1024;
const double samplingFrequency = 4000;
const int inputPin = 26; // ADC0 en Pico
const int readyPin = 14; // Pin de salida para handshake

double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

void setup() {
  pinMode(readyPin, OUTPUT);
  digitalWrite(readyPin, LOW);
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("FSK Demodulator iniciado");
}

void loop() {
  // Indica al modulador que está listo
  digitalWrite(readyPin, HIGH);

  for (uint16_t i = 0; i < samples; i++) {
    vReal[i] = analogRead(inputPin);
    vImag[i] = 0;
    delayMicroseconds(1000000 / samplingFrequency);
  }

  // Indica al modulador que terminó
  digitalWrite(readyPin, LOW);

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double maxMag = 0;
  int maxIndex = 0;
  for (uint16_t i = 1; i < samples / 2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      maxIndex = i;
    }
  }
  double dominantFreq = (maxIndex * samplingFrequency) / samples;
  Serial.print("Frecuencia dominante: ");
  Serial.print(dominantFreq);
  Serial.println(" Hz");

  delay(500);
}
 