// Consola de mandos ESP32 -> Atlas (Boston Dynamics) en PyBullet
// Envia por serial a 50 Hz:  X,Y,POT,SW,B1,B2,B3,B4
//   X,Y  : -100..100 (0 = centro, con zona muerta)
//   POT  : 0..100
//   SW,B : 1 = pulsado
//
// Cableado (ESP32 DevKit):
//   Joystick VRx -> GPIO34 | VRy -> GPIO35 | SW -> GPIO32 | 3V3 y GND
//   Potenciometro (pin central) -> GPIO33
//   Botones (el otro extremo a GND): B1 GPIO25, B2 GPIO26, B3 GPIO27, B4 GPIO14

const int PIN_X = 34, PIN_Y = 35, PIN_SW = 32, PIN_POT = 33;
const int PIN_B[4] = {25, 26, 27, 14};

// Si el joystick sale invertido, cambia estos valores
const bool INVERT_X = true;
const bool INVERT_Y = true;

const int DEADZONE = 250;       // en cuentas del ADC (0..4095)
const unsigned long PERIODO_MS = 20;

int cx = 2048, cy = 2048;       // centro calibrado al arrancar
float potFiltro = 0;

int leerPromedio(int pin, int n = 16) {
  long s = 0;
  for (int i = 0; i < n; i++) { s += analogRead(pin); delay(2); }
  return s / n;
}

int normalizar(int raw, int centro, bool invertir) {
  int d = raw - centro;
  if (abs(d) < DEADZONE) return 0;
  int v;
  if (d > 0) v = (long)(d - DEADZONE) * 100 / (4095 - centro - DEADZONE);
  else       v = (long)(d + DEADZONE) * 100 / (centro - DEADZONE);
  v = constrain(v, -100, 100);
  return invertir ? -v : v;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_SW, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) pinMode(PIN_B[i], INPUT_PULLUP);
  delay(300);
  cx = leerPromedio(PIN_X);     // no muevas el joystick al encender
  cy = leerPromedio(PIN_Y);
  potFiltro = analogRead(PIN_POT);
}

void loop() {
  static unsigned long t0 = 0;
  if (millis() - t0 < PERIODO_MS) return;
  t0 = millis();

  int x = normalizar(analogRead(PIN_X), cx, INVERT_X);
  int y = normalizar(analogRead(PIN_Y), cy, INVERT_Y);
  potFiltro += 0.2f * (analogRead(PIN_POT) - potFiltro);   // filtro suave
  int pot = constrain((int)(potFiltro * 100 / 4095), 0, 100);
  int sw = !digitalRead(PIN_SW);

  Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", x, y, pot, sw,
                !digitalRead(PIN_B[0]), !digitalRead(PIN_B[1]),
                !digitalRead(PIN_B[2]), !digitalRead(PIN_B[3]));
}
