#include <Arduino.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pines para el clock del modulador y el input de la señal
const int PIN_RX = 14;           // Entrada digital FSK
const int CLOCK_PIN = 16;        // Pin de clock del modulador

// Configuración de la modulación FSK
const double freq0 = 100.0;
const double freq1 = 1000.0;
const double T_bit = 0.05; //1.0 / freq0;
const double Fs = 2560.0;
const uint16_t N = 128; //(uint16_t)(Fs * T_bit); // muestras por bit

// Configuración de los Buffers de muestreos y el circular
const int BUFFER_SIZE = 1024;
volatile uint16_t circularBuffer[BUFFER_SIZE];
volatile uint16_t writeIndex = 0;
volatile bool blockReady = false;

// Variables para la FFT
double vReal[N], vImag[N];
ArduinoFFT<double> FFT(vReal, vImag, N, Fs);

// Buffer de bits
byte bitBuffer[256];
int head = 0, tail = 0;

// Variables para el control de la LCD
const int LCD_COLS = 16;  // Columnas de la LCD
const int LCD_ROWS = 2;   // Filas de la LCD
int currentCol = 0;
int currentRow = 0;
String currentMessage = "";
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Sincronización de inicio
short countBitsInicio = 0;
const short num_bits_sinc = 4;


// =======================
// ISR: leer señal FSK continuamente (subrutina para no perder muestras)
// =======================
bool sample_callback(struct repeating_timer *t) {
  circularBuffer[writeIndex] = digitalRead(PIN_RX);
  writeIndex = (writeIndex + 1) % BUFFER_SIZE;
  return true;
}

// =======================
// ISR: flanco de clock -> procesar un bit
// =======================
void clockISR() {
  blockReady = true;
}

// =======================
// FFT y detección de bit
// =======================
int processFFT(double *vReal, double *vImag) {
  double mean = 0;
  for (int i=0;i<N;i++){
    mean += vReal[i];
  }
  mean /= N;
  for (int i=0;i<N;i++) {
    vReal[i] -= mean;
  }

  FFT.windowing(FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double maxMag = 0;
  int maxIndex = 0;
  for (int i=1; i<N/2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      maxIndex = i;
    }
  }

  // Se obtiene la frecuencia dominante
  double dominantFreq = (maxIndex * Fs) / N;

  short bitDetected;
  short incertidumbre = 80; // las frecuencias se pueden encontrar sobre o por debajo 100 Hz de la frecuencia nominal
  
  // Se realiza un filtrado de las frecuencias aceptables para este receptor
  if (freq0 - 60 < dominantFreq && dominantFreq < freq0 + incertidumbre){
    bitDetected = 0;
  }
  else if (freq1 - incertidumbre < dominantFreq && dominantFreq < freq1 + incertidumbre) {
    bitDetected = 1;
  }
  else {
    bitDetected = -1; // El mensaje de dicha frecuencia se omite
  }

  return bitDetected;
}

// =======================
// Buffer de bits
// =======================
void pushBit(int bit) {
  bitBuffer[head] = bit;
  head = (head + 1) % 256;
  if (head == tail) tail = (tail + 1) % 256;
}

bool getByte(char &c) {
  if ((head - tail + 256) % 256 < 8) return false;
  byte val = 0;
  for (int i=0;i<8;i++) {
    val = (val << 1) | bitBuffer[tail];
    tail = (tail + 1) % 256;
  }
  c = (char)val;
  return true;
}

// =======================
// Funciones para inicializar de LCD
// =======================
void initializeLCD() {
  Wire.begin();
  lcd.init();
  lcd.begin(LCD_COLS, LCD_ROWS); // Confirmación de tamaño
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  
  currentCol = 0;
  currentRow = 0;
  currentMessage = "";
}

// Función para escribir un caracter en la LCD
void displayCharOnLCD(char c) {
  // Si llegamos al final de una fila, pasar a la siguiente
  if (currentCol >= LCD_COLS) {
    currentCol = 0;
    currentRow++;
  }

  // Si la pantalla está llena, limpiar y comenzar de nuevo
  if (currentRow > LCD_ROWS) {
    lcd.clear();
    currentCol = 0;
    currentRow = 0;
    currentMessage = "";
  }
  
  // Mostrar el carácter en la posición actual
  lcd.setCursor(currentCol, currentRow);
  lcd.print(c);
  
  // Actualizar posición y mensaje actual
  currentCol++;
  currentMessage += c;
}


// =======================
// Setup
// =======================
struct repeating_timer timer;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RX, INPUT);
  pinMode(CLOCK_PIN, INPUT);

  // Inicializar LCD
  initializeLCD();

  // ISR de muestreo rápido
  add_repeating_timer_us(-(int)(1e6 / Fs), sample_callback, NULL, &timer);

  // ISR de clock externo
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), clockISR, RISING);
}

// =======================
// Loop principal
// =======================
void loop() {
  
  // Entra aquí cuando se termina de enviar un bit (un bloque de 128 muestras, buffer)
  if (blockReady) {
    blockReady = false;

    // Copiar bloque FFT alineado con el clock
    uint16_t idx = (writeIndex + BUFFER_SIZE - N) % BUFFER_SIZE;
    for (int i=0; i<N; i++) {
      vReal[i] = (double)circularBuffer[(idx + i) % BUFFER_SIZE];
      vImag[i] = 0.0;
    }

    // Si el bit se encuentra dentro de las frecuencias aceptadas, se guarda
    int bit = processFFT(vReal, vImag);
    if (bit != -1) {
      // Comparación para sincronización
      if (countBitsInicio < 4 && bit==1){
        countBitsInicio++; // cuando sea igual a 4, lo que sea que sigue ya es el inicio del mensaje
      } 
      else if (countBitsInicio >= 4){
        pushBit(bit);
      }
    }

    // Se consulta si hay alguna letra (ASCII) completada 
    char c;
    if (getByte(c)) {
      if (c >= 32 && c <= 126) {  // Caracteres ASCII imprimibles
        displayCharOnLCD(c);
      }
    }
  }
}
