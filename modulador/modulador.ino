// MODULADOR FSK CON PWM – Raspberry Pi Pico W
#include <Arduino.h>
#include "hardware/pwm.h"

const int pwmPin = 15;       // Pin de salida PWM
const int clockPin = 16;     // Pin de clock externo para sincronización
bool sincronizacion = true;

// Texto 'HOLA'
const int bitSequence[] = {0,1,0,0,1,0,0,0,
                          0,1,1,0,1,1,1,1,
                          0,1,1,0,1,1,0,0,
                          0,1,1,0,0,0,0,1};
const int numBits = sizeof(bitSequence) / sizeof(bitSequence[0]);

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

void loop() {

  if (digitalRead())

  if (sincronizacion == true){
    for (int i = 0; i<4; i++){
      setPWMFrequency(freq1);
      digitalWrite(clockPin, LOW); // Se baja pues se va enviar un bit (no se puede hacer FFT aún)
      pwm_set_enabled(slice, true);
      delay((int)bitDuration_ms);
      pwm_set_enabled(slice,false);
      digitalWrite(clockPin, HIGH);
    }
    sincronizacion = false;
  }
  else {
    for (int i = 0; i < numBits; i++) {
      float freq = (bitSequence[i] == 1) ? freq1 : freq0;
      setPWMFrequency(freq);
      digitalWrite(clockPin, LOW); // Se baja pues se va enviar un bit (no se puede hacer FFT aún)
      pwm_set_enabled(slice, true);
      delay((int)bitDuration_ms);
      pwm_set_enabled(slice,false);
      digitalWrite(clockPin, HIGH);   // Indica que ya se terminó de enviar un bit (se puede hacer FFT sobre ese)
    }
  }
}
