//MODULADOR FSK PARA RASPBERRY PI PICO

// Incluye la librería principal de Arduino
#include <arduino.h>

// Pin de salida para la señal FSK
const int outputPin = 15;
// Pin de entrada para handshake con el demodulador
const int handshakePin = 16; // Pin de entrada para handshake
// Frecuencia para el bit 0 (Hz)
const int freq0 = 100;
// Frecuencia para el bit 1 (Hz)
const int freq1 = 1000;
// Duración de cada bit en milisegundos
const int bitDuration = 100;


// Secuencia de bits a transmitir
int bitSequence[] = {1, 0, 0, 1, 0};
// Número de bits en la secuencia
int numBits = sizeof(bitSequence) / sizeof(bitSequence[0]);

// Configuración inicial del microcontrolador
void setup() {
  pinMode(outputPin, OUTPUT); // Configura el pin de salida
  pinMode(handshakePin, INPUT); // Configura el pin de handshake como entrada
  Serial.begin(115200); // Inicializa la comunicación serial
  Serial.println("FSK Modulator iniciado");
}

// Bucle principal de transmisión FSK
void loop() {
  // Espera a que el demodulador esté listo (handshakePin en HIGH)
  while (digitalRead(handshakePin) == LOW) {
    // Espera pasiva hasta que el demodulador indique estar listo
  }

  // Envía cada bit de la secuencia
  for (int i = 0; i < numBits; i++) {
    int freq = bitSequence[i] ? freq1 : freq0; // Selecciona la frecuencia según el bit
    Serial.print("Bit: ");
    Serial.print(bitSequence[i]);
    Serial.print(" Frecuencia: ");
    Serial.println(freq);

    // Genera la señal FSK durante la duración del bit
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
    // Espera pasiva hasta que el demodulador indique que terminó
  }
}