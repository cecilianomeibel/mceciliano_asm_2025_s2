// MODULADOR FSK CON PWM – Raspberry Pi Pico W
#include <Arduino.h>
#include "hardware/pwm.h"

const int pwmPin = 15;       // Pin de salida PWM
const int clockPin = 16;     // Pin de clock externo para sincronización
bool sincronizacion = true;

// Buffer para la palabra recibida
#define LONGITUD_MAX 32
char palabra[LONGITUD_MAX + 1];
int longitudPalabra = 0;
bool palabraLista = false;

const float freq0 = 100.0;   // Frecuencia del bit 0
const float freq1 = 1000.0;  // Frecuencia del bit 1
const float bitDuration_ms = 50;  // Duración del bit (en milisegundos)

uint slice;  // canal PWM
uint channelNum;

void setup() {
  Serial.begin(115200);

  // Configurar pin PWM
  pinMode(pwmPin, OUTPUT);

  // asignar función PWM al pin
  gpio_set_function(pwmPin, GPIO_FUNC_PWM);

  slice = pwm_gpio_to_slice_num(pwmPin); // Obtiene el canal PWM interno
  channelNum = pwm_gpio_to_channel(pwmPin);

  // Configurar pin de clock
  pinMode(clockPin, OUTPUT);
  digitalWrite(clockPin, LOW);
}

void setPWMFrequency(float freq) {
  // Frecuencia base del PWM de la Pico: 125 MHz
  const float pwmClock = 125000000.0;

  // Queremos una señal cuadrada → duty = 50%
  // Calcular el divisor y el tope del contador
  float divider = 1.0;
  uint32_t top = (uint32_t)(pwmClock / (freq * divider)) - 1;

  // Si el valor de top es mayor al máximo permitido, ajusta el divisor
  while (top > 65535) {
    divider *= 2;
    top = (uint32_t)(pwmClock / (freq * divider)) - 1;
  }

  if (top < 1) top = 1; // evitar wrap=0

  // aplicar divisor y top
  pwm_set_clkdiv(slice, divider);        // divisor float
  pwm_set_wrap(slice, (uint16_t)top);    // wrap (TOP)

  // canal A o B: establecemos nivel = 50% (TOP/2)
  uint32_t level = ((uint32_t)top) / 2u;
  pwm_set_chan_level(slice, channelNum, (uint16_t)level);
}

void transmitirPalabraFSK(const char* palabra, int longitud) {
  // Secuencia de sincronización (4 bits '1')
  for (int i = 0; i < 4; i++) {
    setPWMFrequency(freq1);
    digitalWrite(clockPin, LOW);
    pwm_set_enabled(slice, true);
    delay((int)bitDuration_ms);
    pwm_set_enabled(slice, false);
    digitalWrite(clockPin, HIGH);
  }
  // Transmitir cada carácter como 8 bits
  for (int i = 0; i < longitud; i++) {
    char c = palabra[i];
    for (int b = 7; b >= 0; b--) {
      int bit = (c >> b) & 1;
      setPWMFrequency(bit ? freq1 : freq0);
      digitalWrite(clockPin, LOW);
      pwm_set_enabled(slice, true);
      delay((int)bitDuration_ms);
      pwm_set_enabled(slice, false);
      digitalWrite(clockPin, HIGH);
    }
  }
}

void loop() {
  // Leer palabra desde Serial
  if (!palabraLista) {
    if (Serial.available() > 0) {
      longitudPalabra = Serial.readBytesUntil('\n', palabra, LONGITUD_MAX);
      palabra[longitudPalabra] = '\0';
      if (longitudPalabra > 0) {
        palabraLista = true;
        Serial.print("Palabra recibida: ");
        Serial.println(palabra);
      }
    }
    return;
  }

  // Transmitir la palabra por FSK
  transmitirPalabraFSK(palabra, longitudPalabra);
  Serial.println("Transmisión FSK finalizada. Ingrese otra palabra:");
  palabraLista = false;
}

 