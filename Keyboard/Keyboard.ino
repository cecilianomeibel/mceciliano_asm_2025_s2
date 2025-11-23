#include <Keypad.h>

//Configuración del teclado
const byte FILAS = 4;
const byte COLUMNAS = 4;

char teclas[FILAS][COLUMNAS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte pinesFilas[FILAS] = {9,8,6,7};
byte pinesColumnas[COLUMNAS] = {5,4,3,2};

Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

//Variables globales
char bufferHex[3];  // 2 dígitos hex + '\0'
byte indice = 0;

// Funciones
bool esHexValido(char c);
void leerHexDesdeTeclado();

void setup() {
  Serial.begin(9600);
  Serial.println("Ingrese 2 dígitos en hexadecimal (0-9, A-F), y presione '#' para convertir:");
}

void loop() {
  leerHexDesdeTeclado();  //llamada a la función principal
}

//Función que lee y procesa la entrada desde el teclado
void leerHexDesdeTeclado() {
  char tecla = teclado.getKey();

  if (!tecla) return; // si no hay tecla presionada, salir

  if (tecla == '#') {
    if (indice == 2) {
      bufferHex[2] = '\0';
      byte valor = strtol(bufferHex, NULL, 16);
      Serial.print("\nHex: ");
      Serial.print(bufferHex);
      Serial.print(" -> ASCII: ");
      Serial.println((char)valor);
    } else {
      Serial.println("\nDebe ingresar exactamente 2 dígitos hex antes de presionar '#'");
    }
    indice = 0;
  }
  else if (tecla == '*') {
    indice = 0;
    Serial.println("\nEntrada borrada.");
  }
  else if (esHexValido(tecla)) {
    if (indice < 2) {
      bufferHex[indice++] = tecla;
      Serial.print(tecla);
    } else {
      Serial.println("\nYa se ingresaron 2 dígitos. Presione '#' para convertir o '*' para reiniciar.");
    }
  }
}

//Valida si un carácter es un dígito hexadecimal ---
bool esHexValido(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'F') ||
         (c >= 'a' && c <= 'f');
}


