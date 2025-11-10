//DEMODULADOR FSK PARA RASPBERRY PI PICO

// Incluye las librerías necesarias para Arduino y FFT
#include <arduino.h>
#include <arduinoFFT.h>

// Número de muestras para la FFT
const uint16_t samples = 1024;
// Frecuencia de muestreo en Hz
const double samplingFrequency = 4000;
// Pin de entrada analógica (ADC0 en Raspberry Pi Pico)
const int inputPin = 26; // ADC0 en Pico
// Pin de salida digital para handshake con el modulador
const int readyPin = 14; // Pin de salida para handshake

// Arreglos para almacenar las partes real e imaginaria de la señal
double vReal[samples];
double vImag[samples];
// Instancia de la clase ArduinoFFT para realizar la transformada
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// Configuración inicial del microcontrolador
void setup() {
  pinMode(readyPin, OUTPUT); // Configura el pin de handshake como salida
  digitalWrite(readyPin, LOW); // Inicializa el pin en bajo
  Serial.begin(115200); // Inicializa la comunicación serial
  analogReadResolution(12); // Configura la resolución del ADC a 12 bits
  Serial.println("FSK Demodulator iniciado");
}

// Bucle principal de adquisición y procesamiento
void loop() {
  // Indica al modulador que el demodulador está listo para recibir datos
  digitalWrite(readyPin, HIGH);

  // Adquisición de muestras analógicas
  for (uint16_t i = 0; i < samples; i++) {
    vReal[i] = analogRead(inputPin); // Lee el valor analógico
    vImag[i] = 0; // Parte imaginaria en cero
    delayMicroseconds(1000000 / samplingFrequency); // Espera para mantener la frecuencia de muestreo
  }

  // Indica al modulador que terminó la adquisición
  digitalWrite(readyPin, LOW);

  // Aplica ventana de Hamming para reducir el efecto de fugas espectrales
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  // Calcula la FFT
  FFT.compute(FFT_FORWARD);
  // Convierte los resultados complejos a magnitud
  FFT.complexToMagnitude();

  // Busca la frecuencia dominante en el espectro
  double maxMag = 0;
  int maxIndex = 0;
  for (uint16_t i = 1; i < samples / 2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      maxIndex = i;
    }
  }
  // Calcula la frecuencia dominante detectada
  double dominantFreq = (maxIndex * samplingFrequency) / samples;
  Serial.print("Frecuencia dominante: ");
  Serial.print(dominantFreq);
  Serial.println(" Hz");

  // Espera antes de la siguiente adquisición
  delay(500);
}
 