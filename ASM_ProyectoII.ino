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
float referencia = 5.0;  // RPM simulados
float medida = 0;          // lectura del "sensor"
float salidaPID = 0;

// Pin del motor (se prueba con un led)
int pwmPin = 9; // PWM ahora en pin 9

// Pines sensor ultrasónico 1
const int trigPin = 13;
const int echoPin = 12;
// Pines sensor ultrasónico 2
const int trigPin2 = 11;
const int echoPin2 = 10;

float distanciaActual = 0; // Variable global para la distancia del primer sensor
float distanciaActual2 = 0; // Variable global para la distancia del segundo sensor

// Pin de dirección L293D
int IN1 = 6; // Entrada 1 L293D
int IN2 = 2; // Entrada 2 L293D

// Pines DIP switch
const int dipPin1 = 8; // DIP switch 1 en pin 8
const int dipPin2 = 7; // DIP switch 2 en pin 7

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(dipPin1, INPUT_PULLUP);
  pinMode(dipPin2, INPUT_PULLUP);
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - lastTime >= T) {
    lastTime = ahora;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Leer estado de DIP switches
    bool dip1Activo = digitalRead(dipPin1) == LOW;
    bool dip2Activo = digitalRead(dipPin2) == LOW;

    if (dip1Activo) {
      // Sensor ultrasónico 1 activo, motor gira a la derecha
      distanciaActual = leerDistanciaCM();
      Serial.print("Distancia: ");
      Serial.println(distanciaActual);

      // Control PID para velocidad del motor con sensor 1
      float setpoint = 2.0;
      float error = distanciaActual - setpoint;
      integral += error * (T / 1000.0);
      float derivada = (error - errorPrevio) / (T / 1000.0);
      float salidaPID = Kp * error + Ki * integral + Kd * derivada;
      errorPrevio = error;

      int offset = 80;
      int pwm = (int)salidaPID + offset;
      if (pwm > 255) pwm = 255;
      if (pwm < offset) pwm = offset;
      if (abs(error) < 2) {
        pwm = 0;
      }

      analogWrite(pwmPin, pwm);
      digitalWrite(IN1, HIGH); // derecha
      digitalWrite(IN2, LOW);
      Serial.print("PWM aplicado (sensor 1): ");
      Serial.println(pwm);
    } else if (dip2Activo) {
      // Sensor ultrasónico 2 activo, motor gira a la izquierda
      distanciaActual2 = leerDistanciaCM2();
      Serial.print("Distancia sensor 2: ");
      Serial.println(distanciaActual2);

      // Control PID para velocidad del motor con sensor 2
      float setpoint2 = 2.0;
      float error2 = distanciaActual2 - setpoint2;
      static float errorPrevio2 = 0;
      static float integral2 = 0;
      integral2 += error2 * (T / 1000.0);
      float derivada2 = (error2 - errorPrevio2) / (T / 1000.0);
      float salidaPID2 = Kp * error2 + Ki * integral2 + Kd * derivada2;
      errorPrevio2 = error2;

      int offset = 80;
      int pwm2 = (int)salidaPID2 + offset;
      if (pwm2 > 255) pwm2 = 255;
      if (pwm2 < offset) pwm2 = offset;
      if (abs(error2) < 2) {
        pwm2 = 0;
      }

      analogWrite(pwmPin, pwm2);
      digitalWrite(IN1, LOW); // izquierda
      digitalWrite(IN2, HIGH);
      Serial.print("PWM aplicado (sensor 2): ");
      Serial.println(pwm2);
    } else {
      // Ningún DIP activo: motor detenido
      analogWrite(pwmPin, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      Serial.println("Motor detenido (ningun DIP activo)");
    }
  }
}

// =====================================================
// Función para leer distancia del sensor ultrasónico
// =====================================================
float leerDistanciaCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  float distancia = duracion * 0.034 / 2;
  return distancia;
}

// =====================================================
// Función para leer distancia del segundo sensor ultrasónico
// =====================================================
float leerDistanciaCM2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  long duracion = pulseIn(echoPin2, HIGH, 30000); // timeout 30ms
  float distancia = duracion * 0.034 / 2;
  return distancia;
}


