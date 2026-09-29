/*
  Teclado matricial 4x4 con ESP32
  ---------------------------------
  Lee un teclado matricial 4x4 y envia por Serial (a 115200 baudios)
  las teclas A, B, C, D (para mover el enjambre) y # / * (para
  cerrar o reiniciar la simulacion) cuando se presionan. El script
  de Python (pid_ABC.py) escucha ese puerto serial y controla el
  enjambre de drones en consecuencia.

  Libreria necesaria: "Keypad" de Mark Stanley y Alexander Brevig
  En Arduino IDE: Herramientas > Administrar bibliotecas > buscar "Keypad" > Instalar

  Conexion:
  ---------
  El teclado matricial 4x4 tiene 8 pines (4 filas + 4 columnas).
  Conecta cada pin del teclado a un GPIO del ESP32 segun la tabla
  de abajo. Estos GPIOs se eligieron por ser seguros para uso
  general (evitan los pines de arranque 0, 2, 4, 15 y los pines de
  solo lectura 34-39).

  Teclado      ESP32 GPIO
  -------      ----------
  Fila 1   ->  13
  Fila 2   ->  12
  Fila 3   ->  14
  Fila 4   ->  27
  Columna 1 -> 26
  Columna 2 -> 25
  Columna 3 -> 33
  Columna 4 -> 32
*/

#include <Keypad.h>

const byte ROWS = 4;
const byte COLS = 4;

//// Distribucion estandar de un teclado matricial 4x4 ////
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

//// Pines seguros recomendados para ESP32 ////
byte rowPins[ROWS] = {13, 12, 14, 27};   // Filas del teclado
byte colPins[COLS] = {26, 25, 33, 32};   // Columnas del teclado

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(115200);   // Debe coincidir con baud_rate en pid_ABC.py
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    //// Nos interesan las letras A, B, C, D y los simbolos # y * ////
    //// (los numeros del 0-9 se ignoran) ###############################
    if (key == 'A' || key == 'B' || key == 'C' || key == 'D' ||
        key == '#' || key == '*') {
      Serial.println(key);
    }
  }
}
