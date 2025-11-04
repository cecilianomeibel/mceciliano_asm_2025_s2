//MODULADOR

#include <arduino.h>

const int outputPin = 15;
const int handshakePin = 16; // Pin de entrada para handshake
const int freq0 = 100;
const int freq1 = 1000;
const int bitDuration = 100;

int bitSequence[] = {1, 0, 0, 1, 0};
int numBits = sizeof(bitSequence) / sizeof(bitSequence[0]);

void setup() {
  pinMode(outputPin, OUTPUT);
  pinMode(handshakePin, INPUT);
  Serial.begin(115200);
  Serial.println("FSK Modulator iniciado");
}

void loop() {
  // Espera a que el demodulador esté listo (handshakePin en HIGH)
  while (digitalRead(handshakePin) == LOW) {
    // Espera pasiva
  }

  for (int i = 0; i < numBits; i++) {
    int freq = bitSequence[i] ? freq1 : freq0;
    Serial.print("Bit: ");
    Serial.print(bitSequence[i]);
    Serial.print(" Frecuencia: ");
    Serial.println(freq);

    unsigned long t0 = millis();
    while (millis() - t0 < bitDuration) {
      digitalWrite(outputPin, HIGH);
      delayMicroseconds(500000 / freq);
      digitalWrite(outputPin, LOW);
      delayMicroseconds(500000 / freq);
    }
  }

  // Espera a que el demodulador termine (handshakePin en LOW)
  while (digitalRead(handshakePin) == HIGH) {
    // Espera pasiva
  }
}