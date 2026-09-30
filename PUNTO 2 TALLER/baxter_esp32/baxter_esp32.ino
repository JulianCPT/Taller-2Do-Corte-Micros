// Consola ESP32 para Baxter (PyBullet)
// Envia por USB serial (~50 Hz):  jx,jy,pot,rot,arm,grip,mode
//   jx, jy : joystick, de -1 a 1 (0 = centro)
//   pot    : potenciometro, de 0 a 1 (posicion absoluta)
//   rot    : botones de giro: -1 (izq), 0, +1 (der)
//   arm    : 0 = brazo izquierdo, 1 = derecho      (boton brazo, conmuta)
//   grip   : 0/1, cambia cada vez que pulsas       (boton pinza, conmuta)
//   mode   : 0 = modo 1 (brazo), 1 = modo 2 (muneca)  (click del joystick, conmuta)
//
// Placa: "ESP32 Dev Module"  |  Velocidad: 115200
//
// Conexiones:
//   Joystick  : VRx -> 34 | VRy -> 35 | SW (click) -> 33 | +5V/VCC -> 3V3 | GND -> GND
//   Potenciometro: pin central -> 32, extremos a 3V3 y GND
//   Boton brazo : 25 -> GND
//   Boton pinza : 26 -> GND
//   Giro izq    : 27 -> GND
//   Giro der    : 14 -> GND

const int PIN_JX   = 34;
const int PIN_JY   = 35;
const int PIN_POT  = 32;
const int BTN_MODE = 33;   // click del joystick (pin SW)
const int BTN_ARM  = 25;
const int BTN_GRIP = 26;
const int BTN_ROT_L = 27;
const int BTN_ROT_R = 14;

// Orientacion del joystick (ajusta segun como lo tengas puesto)
const bool SWAP_XY  = false;   // true si VRx resulta ser el eje vertical
const bool INVERT_X = true;    // izquierda/derecha invertido -> true
const bool INVERT_Y = true;    // arriba/abajo invertido -> true

int centroJX, centroJY;
float potFilt = 0;

// Botones que conmutan (con antirrebote)
struct Toggle {
  int pin;
  bool state;
  bool last;
  unsigned long t;
};

Toggle tArm  = {BTN_ARM,  false, HIGH, 0};
Toggle tGrip = {BTN_GRIP, false, HIGH, 0};
Toggle tMode = {BTN_MODE, false, HIGH, 0};

void updateToggle(Toggle &b) {
  bool now = digitalRead(b.pin);
  unsigned long ms = millis();
  if (b.last == HIGH && now == LOW && ms - b.t > 250) {
    b.state = !b.state;
    b.t = ms;
  }
  b.last = now;
}

float normaliza(int lectura, int centro) {
  int v = lectura - centro;
  float rango = (v >= 0) ? (4095 - centro) : centro;
  float n = v / rango;
  if (n > 1) n = 1;
  if (n < -1) n = -1;
  return n;
}

int promedio(int pin) {
  long s = 0;
  for (int k = 0; k < 64; k++) { s += analogRead(pin); delay(2); }
  return s / 64;
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_ARM, INPUT_PULLUP);
  pinMode(BTN_GRIP, INPUT_PULLUP);
  pinMode(BTN_ROT_L, INPUT_PULLUP);
  pinMode(BTN_ROT_R, INPUT_PULLUP);
  analogReadResolution(12);
  delay(500);                       // NO toques el joystick al arrancar
  centroJX = promedio(PIN_JX);
  centroJY = promedio(PIN_JY);
  potFilt = analogRead(PIN_POT) / 4095.0;
}

void loop() {
  updateToggle(tArm);
  updateToggle(tGrip);
  updateToggle(tMode);

  float ax = normaliza(analogRead(PIN_JX), centroJX);
  float ay = normaliza(analogRead(PIN_JY), centroJY);
  float jx = SWAP_XY ? ay : ax;
  float jy = SWAP_XY ? ax : ay;
  if (INVERT_X) jx = -jx;
  if (INVERT_Y) jy = -jy;

  // potenciometro: filtro suave para quitar el ruido del ADC del ESP32
  potFilt += 0.2 * (analogRead(PIN_POT) / 4095.0 - potFilt);

  int rot = (digitalRead(BTN_ROT_R) == LOW ? 1 : 0) - (digitalRead(BTN_ROT_L) == LOW ? 1 : 0);

  Serial.printf("%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
                jx, jy, potFilt, rot,
                tArm.state ? 1 : 0, tGrip.state ? 1 : 0, tMode.state ? 1 : 0);
  delay(20);
}
