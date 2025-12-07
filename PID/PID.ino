
// Parámetros del PID
float Kp = 1.2;
float Ki = 0.4;
float Kd = 0.05;

// Variables internas PID
float errorPrevio = 0;
float integral = 0;

// Frecuencia de muestreo propuesta: 10 ms = 100 Hz 
unsigned long T = 10;      
unsigned long lastTime = 0;

// Entrada, salida y referencia (setpoint)
float referencia = 100.0;  // RPM simulados
float medida = 0;          // lectura del "sensor"
float salidaPID = 0;

// Pin del motor (se prueba con un led)
int pwmPin = 9;

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  
}

void loop() {

  unsigned long ahora = millis();
  if (ahora - lastTime >= T) {
    lastTime = ahora;
    
    // 1. Parpadear LED para verificar ejecución del PID
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    //Se siguen los pasos de sección 3 de la tarea 2

    // 2. Leer "velocidad" simulada
    medida = leerVelocidadMotor();  //retorna valores aleatorios 80–120

    // 3. Calcular error
    float error = referencia - medida;

    // 4. Cálculo integral
    integral += error * (T / 1000.0);

    // 5. Cálculo derivada
    float derivada = (error - errorPrevio) / (T / 1000.0);

    // 6. PID (ecuación de control)
    salidaPID = Kp * error + Ki * integral + Kd * derivada;

    // Saturación del PWM
    if (salidaPID > 255) salidaPID = 255;
    if (salidaPID < 0) salidaPID = 0;

    // 7. Aplicar PWM simulado al LED en este caso
    analogWrite(pwmPin, (int)salidaPID);

    // 8. Guardar error previo
    errorPrevio = error;

    // 9. Se printean los datos
    // Comportamiento esperado:
    // Conforme cambia la medida debe cambiar el PWM
    // Si medida<referencia -> PWM debería subir   (LED más brillante)
    // Si medida>referencia -> PWM debería bajar   (LED se atenua)
    
    Serial.print("Medida: ");
    Serial.print(medida);
    Serial.print("  PWM: ");
    Serial.println(salidaPID);
  }
}

// =====================================================
// Función simulada del sensor de velocidad
// =====================================================
float leerVelocidadMotor() {
  return random(80, 120);  // simula lecturas cercanas a 100 RPM
}

