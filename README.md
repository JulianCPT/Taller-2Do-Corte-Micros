<div align="center">

![Header](https://capsule-render.vercel.app/api?type=waving&color=0:0B3D2E,50:14683F,100:1F8A4C&height=220&section=header&text=Taller%20Segundo%20Corte&fontSize=44&fontColor=ffffff&animation=fadeIn&fontAlignY=38&desc=Real-to-Sim%3A%20consolas%20ESP32%20controlando%20robots%20en%20PyBullet&descAlignY=58&descSize=16)

*Ingeniería Mecatrónica · Universidad Militar Nueva Granada*

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=20&duration=3000&pause=800&color=3DDC84&center=true&vCenter=true&width=640&lines=%22Teclado+matricial+%E2%86%92+enjambre+de+4+drones%22;%22Joystick+%2B+pot+%E2%86%92+Baxter+con+7+ejes%22;%22Click+del+joystick+%E2%86%92+Atlas+camina+o+mueve+brazos%22" alt="Typing SVG" />

<br/>

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![PyBullet](https://img.shields.io/badge/PyBullet-Simulación%20física-1F8A4C?style=for-the-badge&logo=python&logoColor=white)
![Tkinter](https://img.shields.io/badge/Tkinter-Panel%20en%20vivo-A0522D?style=for-the-badge&logo=python&logoColor=white)
![ESP32](https://img.shields.io/badge/ESP32-Arduino-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
![Status](https://img.shields.io/badge/estado-académico-6E40C9?style=for-the-badge)

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## ✨ ¿Qué hace este proyecto?

Taller del **segundo corte** de Microcontroladores: tres ejercicios de **real-to-sim**, es decir,
un **circuito físico con una ESP32** que se convierte en la "consola de mandos" de un robot
que vive dentro de **PyBullet**. La ESP32 lee los botones, el joystick, el potenciómetro o el
teclado matricial y envía los datos por **UART (USB)**; un script de **Python** los recibe y
mueve el robot en la simulación, mientras un **panel de Tkinter** muestra el estado en vivo.

<div align="center">

| Punto | 🤖 Robot simulado | 🎛️ Consola física | 🎮 Qué se controla |
|:---:|:---|:---|:---|
| **1** | Enjambre de **4 drones** Crazyflie (`gym-pybullet-drones`) | Teclado matricial 4×4 | Despegar, volar y aterrizar en los puntos **A, B, C, D** |
| **2** | **Baxter** (`toms_baxter.urdf`) | Joystick + potenciómetro + 4 botones | Los 7 ejes de cada brazo y la pinza, para llevar 3 objetos a una zona verde |
| **3** | **Atlas** de Boston Dynamics | Joystick + potenciómetro + 4 botones | Caminar, girar, saltar y mover los brazos |

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 📑 Contenido

- [🧭 Resumen de los tres puntos](#-resumen-de-los-tres-puntos)
- [📸 Capturas](#-capturas)
- [🎥 Videos y GIFs de funcionamiento](#-videos-de-funcionamiento)
- [📐 Arquitectura general](#-arquitectura-general)
- [🔎 Análisis del proyecto](#-análisis-del-proyecto)
- [📁 Estructura del repositorio](#-estructura-del-repositorio)
- [⚙️ Requisitos](#️-requisitos)
- [▶️ Paso a paso: cómo correrlo](#️-paso-a-paso-cómo-correrlo)
- [📡 Protocolos de comunicación](#-protocolos-de-comunicación)
- [🧩 Explicación del código, bloque por bloque](#-explicación-del-código-bloque-por-bloque)
- [🧠 Conceptos clave](#-conceptos-clave)
- [🛠️ Solución de problemas](#️-solución-de-problemas)
- [👤 Autor](#-autor)

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 🧭 Resumen de los tres puntos

### 🚁 Punto 1 — Enjambre de drones con teclado matricial

Cuatro drones despegan juntos formando un **cuadrado** (separados 18 cm) y vuelan como un solo
cuerpo. Hay cuatro puntos en el piso: **A (0,0)**, **B (3,0)**, **C (4,3)** y **D (1,3)** metros.
El enjambre empieza aterrizado en A; al presionar una letra en el teclado matricial,
**despega, vuela en línea recta a ese punto y aterriza** (navegación libre, en cualquier orden).

<div align="center">

| Tecla | Acción |
|:---:|:---|
| `A` `B` `C` `D` | Vuela al punto elegido (solo si el enjambre está aterrizado) |
| `*` | **Reinicia** la simulación (el enjambre vuelve a A) |
| `#` | **Cierra** la simulación, guarda los resultados y muestra las gráficas |

</div>

### 🦾 Punto 2 — Baxter con joystick, potenciómetro y botones

Los **7 ejes** de cada brazo (`s0 s1 e0 e1 w0 w1 w2`) se reparten entre los controles en **dos
modos**, y el **click del joystick** alterna entre ellos. El objetivo del juego: llevar una
**esfera, un cubo y un cilindro** que están sobre la mesa hasta la **zona verde**.

<div align="center">

| Control | 🅼 Modo 1 — Brazo | 🅼 Modo 2 — Muñeca |
|:---|:---:|:---:|
| Joystick X | `s0` hombro (lados) | `w0` giro del antebrazo |
| Joystick Y | `s1` hombro (arriba/abajo) | `w1` cabeceo de la muñeca |
| Potenciómetro | `e1` codo (posición absoluta) | *(no se usa)* |
| Botones de giro izq/der | `e0` giro del brazo | `w2` giro de la pinza |

</div>

Además: **botón brazo** → alterna brazo izquierdo/derecho · **botón pinza** → abre/cierra (si hay
un objeto a menos de 10 cm, lo sujeta) · **click del joystick** → cambia de modo.

### 🧍 Punto 3 — Atlas (Boston Dynamics)

Una consola de mandos para que el Atlas tenga una **movilidad real**. El click del joystick
alterna entre **CAMINAR** y **BRAZOS**.

<div align="center">

| Control | 🚶 Modo CAMINAR | 🙌 Modo BRAZOS |
|:---|:---|:---|
| Joystick Y / X | Adelante-atrás / girar | Sube-baja el brazo / gira el hombro |
| Potenciómetro | Velocidad máxima | Flexión del codo (recoger/estirar) |
| **B1** | Reinicia la posición | Cambia de brazo (izq/der) |
| **B2 / B3** | Paso lateral izquierda / derecha | Mueven la articulación elegida con B4 |
| **B4** | **Saltar** | Cambia lo que hacen B2/B3: flexión de muñeca → giro de muñeca → giro de codo |

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 📸 Capturas

### 🚁 Punto 1

<div align="center">

<table>
  <tr>
    <td align="center">
      <img src="docs/im%C3%A1genes/Circuito%20Punto%201.jpeg" height="320"/><br/>
      <sub>Circuito: ESP32 + teclado matricial 4×4</sub>
    </td>
    <td align="center">
      <img src="docs/im%C3%A1genes/Montaje%20Completo%20Punto%201.jpeg" height="320"/><br/>
      <sub>Montaje completo: circuito + simulación</sub>
    </td>
  </tr>
  <tr>
    <td align="center" colspan="2">
      <img src="docs/im%C3%A1genes/Interfaz%20PyBullet%20Punto%201.jpeg" width="720"/><br/>
      <sub>Enjambre de 4 drones sobre el punto A en PyBullet</sub>
    </td>
  </tr>
</table>

</div>

### 🦾 Punto 2

<div align="center">

<table>
  <tr>
    <td align="center">
      <img src="docs/im%C3%A1genes/Circuito%20Punto%202.jpeg" width="480"/><br/>
      <sub>Circuito: ESP32, joystick, potenciómetro y botones</sub>
    </td>
    <td align="center">
      <img src="docs/im%C3%A1genes/Montaje%20Completo%20Punto%202.jpg" width="480"/><br/>
      <sub>Montaje completo: circuito + simulación</sub>
    </td>
  </tr>
  <tr>
    <td align="center" colspan="2">
      <img src="docs/im%C3%A1genes/Interfaz%20PyBullet%20Punto%202.jpeg" width="720"/><br/>
      <sub>Baxter, mesa con objetos, zona verde y panel de estado</sub>
    </td>
  </tr>
</table>

</div>

### 🧍 Punto 3

<div align="center">

<table>
  <tr>
    <td align="center">
      <img src="docs/im%C3%A1genes/Circuito%20Punto%203.jpeg" width="480"/><br/>
      <sub>Circuito: ESP32, joystick, potenciómetro y 4 botones</sub>
    </td>
    <td align="center">
      <img src="docs/im%C3%A1genes/Montaje%20Completo%20Punto%203.jpeg" width="480"/><br/>
      <sub>Montaje completo: circuito + simulación</sub>
    </td>
  </tr>
  <tr>
    <td align="center" colspan="2">
      <img src="docs/im%C3%A1genes/Interfaz%20PyBullet%20Punto%203.jpeg" width="720"/><br/>
      <sub>Atlas en modo BRAZOS, cámara sintética y panel de control</sub>
    </td>
  </tr>
</table>

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 🎥 Videos de funcionamiento

> GitHub no reproduce videos `.mp4` alojados en el repo directamente dentro del README,
> así que se dejan como enlaces descargables/reproducibles desde el navegador.

| Punto | Video | Qué muestra |
|:---:|:---|:---|
| 1 | ▶️ [**Funcionamiento Punto 1**](docs/videos/Funcionamiento%20Punto%201.mp4) | El enjambre despegando, volando y aterrizando según la tecla presionada |
| 2 | ▶️ [**Montaje completo**](docs/videos/Funcionamiento%20Punto%202%20Montaje%20Completo.mp4) | Circuito físico y simulación funcionando al mismo tiempo |
| 2 | ▶️ [**Solo PyBullet**](docs/videos/Funcionamiento%20Punto%202%20en%20PyBullet.mp4) | Baxter moviendo los objetos hacia la zona verde |
| 3 | ▶️ [**Montaje completo**](docs/videos/Funcionamiento%20Punto%203%20Montaje%20Completo.mp4) | Circuito físico y simulación funcionando al mismo tiempo |
| 3 | ▶️ [**Desplazamientos**](docs/videos/Funcionamiento%20Desplazamientos%20Punto%203.mp4) | El Atlas caminando, girando y dando pasos laterales |
| 3 | ▶️ [**Brazos en diferentes ejes**](docs/videos/Funcionamiento%20Movimiento%20Brazo%20En%20Diferentes%20Ejes%20Punto%203.mp4) | Modo BRAZOS: hombro, codo y muñeca |

<div align="center">

**GIF de vista rápida:** el funcionamiento de cada punto en acción.

<table>
  <tr>
    <td align="center" valign="top" width="33%">
      <img src="docs/videos/GIF%20Punto%201.gif" width="100%" alt="GIF del Punto 1: enjambre de drones"/><br/>
      <sub>🔵 <b>Punto 1</b> · enjambre de drones</sub>
    </td>
    <td align="center" valign="top" width="33%">
      <img src="docs/videos/GIF%20Punto%202.gif" width="100%" alt="GIF del Punto 2: Baxter"/><br/>
      <sub>🔴 <b>Punto 2</b> · Baxter</sub>
    </td>
    <td align="center" valign="top" width="33%">
      <img src="docs/videos/GIF%20Punto%203.gif" width="100%" alt="GIF del Punto 3: Atlas"/><br/>
      <sub>🟣 <b>Punto 3</b> · Atlas</sub>
    </td>
  </tr>
</table>

</div>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 📐 Arquitectura general

Los tres puntos comparten **la misma arquitectura**: cambia el robot y la consola, no el patrón.
Cada columna es un punto, con su propio color: 🔵 **Punto 1**, 🔴 **Punto 2**, 🟣 **Punto 3**.

<div align="center">

<table>
  <tr>
    <td align="center" valign="top" width="33%">
      <img src="docs/im%C3%A1genes/diagrama-arquitectura-punto-1.svg" width="100%" alt="Arquitectura del Punto 1: teclado matricial, ESP32, Python y drones"/>
    </td>
    <td align="center" valign="top" width="33%">
      <img src="docs/im%C3%A1genes/diagrama-arquitectura-punto-2.svg" width="100%" alt="Arquitectura del Punto 2: joystick, ESP32, Python y Baxter"/>
    </td>
    <td align="center" valign="top" width="33%">
      <img src="docs/im%C3%A1genes/diagrama-arquitectura-punto-3.svg" width="100%" alt="Arquitectura del Punto 3: joystick, ESP32, Python y Atlas"/>
    </td>
  </tr>
</table>

</div>

| Símbolo | Significado |
|:---:|:---|
| **→** | Relación de un solo sentido: los datos fluyen en esa dirección |
| **⇢ Línea punteada** | Retroalimentación visual: el usuario ve la simulación y reacciona |
| ⬜ **Caja gris clara** | El usuario y las salidas secundarias (panel de estado, *logger*) |
| 🎨 **Caja de color claro** | Consola física y etapas de software de cada punto |
| ⬛ **Caja de color oscuro** | La pieza central: la ESP32 y la etapa de lógica principal en Python |
| 🟨 **Caja ámbar** | Canal de comunicación: puerto serie USB a 115200 baudios, con el formato de la línea |
| 🟩 **Caja verde** | Lo que se simula en PyBullet: drones, Baxter o Atlas |

> 💡 **Idea clave:** la ESP32 **no sabe nada de robótica**. Solo lee entradas, las limpia
> (zona muerta, filtro, antirrebote) y las reporta como texto. Toda la lógica de modos, los
> límites de las articulaciones, la marcha del Atlas y el control de los drones viven en
> Python, en la PC. Por eso el mismo firmware sencillo sirve para controlar robots muy distintos.

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 🔎 Análisis del proyecto

**¿Qué problema resuelve?** Probar un robot "de verdad" es caro y riesgoso. Con **real-to-sim**
se diseña y prueba la consola de mandos real (hardware + firmware) contra un robot simulado:
si funciona con la simulación, la interfaz humano-máquina ya está validada.

**Decisiones de diseño**

| Decisión | Por qué se tomó |
|:---|:---|
| **ESP32 "tonta", PC "inteligente"** | Mantiene el firmware pequeño y deja el trabajo pesado donde hay más cómputo y es más fácil depurar |
| **Protocolo de texto plano, una línea por mensaje** | Se puede probar con el Monitor Serial, es fácil de parsear y de depurar a ojo |
| **Calibración del centro del joystick al arrancar** | Cada joystick tiene un "cero" distinto; se promedian lecturas en `setup()` (por eso no hay que tocarlo al encender) |
| **Zona muerta + filtro + suavizado exponencial** | Quitan el ruido del ADC de la ESP32 y hacen el movimiento **fluido**, sin sacudidas |
| **Joystick = velocidad, potenciómetro = posición** | El joystick *regresa solo al centro* (sirve para mover a velocidad), el potenciómetro *se queda donde lo dejas* (sirve para una posición absoluta, como el codo) |
| **"Enganchar" el potenciómetro (Baxter)** | Al cambiar de modo o de brazo el codo no salta a la posición del pote: hay que moverlo un poco para retomar el control |
| **Modos con el click del joystick** | Con pocos componentes físicos se controlan 7 ejes por brazo |
| **Marcha procedural (Atlas)** | Caminar con física real requiere un controlador de equilibrio complejo; aquí la pelvis se guía con una restricción y las piernas siguen una marcha periódica, lo que da movilidad fluida y estable |
| **Panel en proceso/ventana aparte** | La interfaz de estado no bloquea el bucle de simulación |
| **Respaldo con teclado del PC** | Permite probar el software sin tener el circuito armado |

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 📁 Estructura del repositorio

```
Taller-2Do-Corte-Micros/
├── PUNTO 1 TALLER/
│   ├── pid_ABC.py                          # Enjambre de drones controlado por teclado (PC)
│   ├── teclado_matricial_ESP32/
│   │   └── teclado_matricial_ESP32.ino     # Firmware: lee el teclado 4x4 y envía la tecla
│   └── results/                            # Vuelos guardados (.npy y CSV) generados al cerrar con '#'
├── PUNTO 2 TALLER/
│   ├── baxter_esp32_control.py             # Baxter en PyBullet + panel Tkinter (PC)
│   └── baxter_esp32/
│       └── baxter_esp32.ino                # Firmware: joystick, pot y botones
├── PUNTO 3 TALLER/
│   ├── atlas_esp32_control.py              # Atlas en PyBullet + panel Tkinter (PC)
│   └── atlas_console/
│       └── atlas_console.ino               # Firmware: joystick, pot y 4 botones
├── docs/                                   # Capturas, esquemas y videos de demostración
│   ├── imágenes/
│   │   ├── diagrama-arquitectura-punto-1.svg   # Esquema de arquitectura del Punto 1
│   │   ├── diagrama-arquitectura-punto-2.svg   # Esquema de arquitectura del Punto 2
│   │   ├── diagrama-arquitectura-punto-3.svg   # Esquema de arquitectura del Punto 3
│   │   └── ...                                 # Circuito, montaje completo e interfaz de cada punto
│   └── videos/
│       ├── GIF Punto 1.gif · GIF Punto 2.gif · GIF Punto 3.gif   # Vista rápida
│       └── ...                                 # Videos de funcionamiento (.mp4)
├── requirements.txt                        # Dependencias de Python
└── README.md
```

> ℹ️ Los robots **Baxter** y **Atlas** vienen del repositorio
> [`erwincoumans/pybullet_robots`](https://github.com/erwincoumans/pybullet_robots), que **no**
> está incluido aquí: se descarga aparte (ver [Requisitos](#️-requisitos)).

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## ⚙️ Requisitos

<div align="center">
<img src="https://skillicons.dev/icons?i=python,arduino,cpp&theme=dark" />
</div>

- ✅ Python 3.10+ (los scripts están pensados para **Windows**; usan `COM3` por defecto)
- ✅ Una **ESP32 DevKit** por cada circuito y un cable USB
- ✅ Arduino IDE con el paquete de placas **esp32 by Espressif Systems**
- ✅ Los componentes de cada punto (tabla de abajo)

**Instalación de dependencias de Python:**

```bash
pip install -r requirements.txt
```

**Punto 1** además necesita el simulador de drones:

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
pip install -e .
```

**Puntos 2 y 3** necesitan los modelos de los robots. Descarga
[`pybullet_robots`](https://github.com/erwincoumans/pybullet_robots) (`git clone` o *Code → Download ZIP*)
y déjalo **en la raíz de este repositorio**, sin mover archivos (las mallas usan rutas relativas):

```
Taller-2Do-Corte-Micros/
└── pybullet_robots-master/     # ← aquí (contiene toms_baxter.urdf y atlas_v4_with_multisense.urdf)
```

> ℹ️ `tkinter`, `time`, `math`, `threading`, `multiprocessing` vienen con la instalación estándar de Python.
> Si instalaste `serial` por error, ejecuta `pip uninstall serial` y luego `pip install pyserial`.

### 🔩 Hardware: conexiones por punto

**Punto 1 — Teclado matricial 4×4**

| Teclado | Fila 1 | Fila 2 | Fila 3 | Fila 4 | Col 1 | Col 2 | Col 3 | Col 4 |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **GPIO ESP32** | 13 | 12 | 14 | 27 | 26 | 25 | 33 | 32 |

**Punto 2 — Baxter**

| Componente | Conexión |
|:---|:---|
| Joystick | `VRx` → GPIO 34 · `VRy` → GPIO 35 · `SW` (click) → GPIO 33 · `VCC` → 3V3 · `GND` → GND |
| Potenciómetro | Pin central → GPIO 32 · extremos a 3V3 y GND |
| Botón brazo | GPIO 25 → GND |
| Botón pinza | GPIO 26 → GND |
| Giro izquierda | GPIO 27 → GND |
| Giro derecha | GPIO 14 → GND |

**Punto 3 — Atlas**

| Componente | Conexión |
|:---|:---|
| Joystick | `VRx` → GPIO 34 · `VRy` → GPIO 35 · `SW` (click) → GPIO 32 · `VCC` → 3V3 · `GND` → GND |
| Potenciómetro | Pin central → GPIO 33 · extremos a 3V3 y GND |
| B1 · B2 · B3 · B4 | GPIO 25 · 26 · 27 · 14 (el otro extremo de cada botón a GND) |

> 💡 Los botones usan `INPUT_PULLUP`: **no necesitan resistencia externa**; pulsado = `LOW`.
> Los GPIO 34–39 de la ESP32 son solo entrada, por eso se usan para los ejes analógicos del joystick.

### 📦 Librerías de Python: qué hacen y por qué se eligieron

| Librería | Para qué sirve | Dónde se usa | Por qué esta y no otra |
|:---|:---|:---|:---|
| **PyBullet** (`pybullet`) | Motor de física: carga el URDF, simula gravedad y colisiones, y controla los motores de cada articulación | Los 3 puntos | Es el simulador que pide el taller; es ligero, gratuito y trae modelos listos (Baxter, Atlas) |
| **gym-pybullet-drones** | Entorno de drones (`CtrlAviary`) y controlador PID (`DSLPIDControl`) con el modelo Crazyflie | Punto 1 | Ya trae la dinámica del dron y un controlador PID calibrado, así no hay que modelarlo desde cero |
| **PySerial** (`pyserial`) | Abre el puerto serie USB, lee las líneas de la ESP32 y lista los puertos disponibles | Los 3 puntos | Es la librería estándar para puertos serie; funciona igual en Windows, Linux y Mac |
| **NumPy** (`numpy`) | Vectores y trayectorias (puntos, offsets de formación, interpolaciones, filtros) | Los 3 puntos | Operaciones numéricas rápidas y es lo que PyBullet y gym-pybullet-drones ya usan |
| **Matplotlib** (`matplotlib`) | Gráficas de posición, velocidad y RPM de los drones al terminar la simulación | Punto 1 (vía `Logger`) | Es la que usa el `Logger` de gym-pybullet-drones |
| **Tkinter** (`tkinter`) | Ventana aparte con el estado: modo activo, ángulos, joystick, pinza | Puntos 2 y 3 | Viene incluido con Python, sin instalar nada |

<details>
<summary>Ver <code>requirements.txt</code></summary>

```txt
pybullet>=3.2.5       # simulación física (los 3 puntos)
numpy>=1.24           # vectores y trayectorias
pyserial>=3.5         # puerto serie hacia la ESP32
matplotlib>=3.7       # gráficas de resultados (Punto 1)
# gym-pybullet-drones se instala aparte (Punto 1): ver sección Requisitos
```

</details>

### 🧰 Librerías de Arduino (firmware)

| Firmware | Librería / API | Para qué sirve |
|:---|:---|:---|
| `teclado_matricial_ESP32.ino` | **Keypad** (Mark Stanley y Alexander Brevig) | Escanea filas y columnas del teclado 4×4 y devuelve la tecla presionada. Se instala desde *Herramientas → Administrar bibliotecas → "Keypad"* |
| `baxter_esp32.ino` · `atlas_console.ino` | `analogRead()` / `analogReadResolution(12)` | Lee el joystick y el potenciómetro con el ADC de 12 bits (0–4095) |
| Los tres | `Serial` (core de Arduino) | Envía los datos a la PC a 115200 baudios; `Serial.printf()` arma las líneas de texto |

> ℹ️ Los firmware de los puntos 2 y 3 **no requieren librerías adicionales**, solo el paquete de placas de la ESP32.

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## ▶️ Paso a paso: cómo correrlo

Los pasos 1–3 son iguales para los tres puntos:

1. **Sube el firmware** del punto desde Arduino IDE (placa `ESP32 Dev Module`).
2. **Cierra el Monitor Serial** del Arduino IDE (ocupa el puerto y el script no podrá abrirlo).
3. **Anota el puerto** (`COM3` en Windows, `/dev/ttyUSB0` en Linux/Mac). En el Punto 1 se pasa con `--serial_port`, en el 3 con `--port`, y en el 2 se edita `SERIAL_PORT` en el script.

> ⚠️ **No muevas el joystick** al encender la ESP32 (Puntos 2 y 3): en ese momento calibra su posición de reposo.

### 🚁 Punto 1

```bash
cd "PUNTO 1 TALLER"
python pid_ABC.py --serial_port COM3
```

Presiona **A, B, C o D** en el teclado matricial. Con `*` reinicias y con `#` cierras (se guardan los resultados en `results/` y se muestran las gráficas).
*Sin hardware:* escribe las teclas `a`, `b`, `c`, `d`, `#`, `*` con la ventana de PyBullet activa.

| Argumento | Valor por defecto | Descripción |
|:---|:---:|:---|
| `--serial_port` | `COM3` | Puerto de la ESP32 |
| `--baud_rate` | `115200` | Debe coincidir con `Serial.begin()` |
| `--num_drones` | `4` | Número de drones del enjambre |
| `--plot` | `True` | Mostrar las gráficas al terminar |
| `--max_duration_sec` | `300` | Límite de seguridad de la simulación |

### 🦾 Punto 2

```bash
cd "PUNTO 2 TALLER"
python baxter_esp32_control.py              # con la ESP32 (edita SERIAL_PORT en el script)
python baxter_esp32_control.py --teclado    # sin hardware, con el teclado del PC
python baxter_esp32_control.py --sin-panel  # sin la ventana de estado
```

| Tecla (modo `--teclado`) | Acción |
|:---:|:---|
| Flechas | Joystick |
| `R` / `F` | Sube / baja el potenciómetro |
| `Q` / `E` | Giro izquierda / derecha |
| `M` | Cambia de modo |
| `1` / `2` | Brazo izquierdo / derecho |
| `Espacio` | Abre / cierra la pinza |
| `C` | Reinicia los objetos |

### 🧍 Punto 3

```bash
cd "PUNTO 3 TALLER"
python atlas_esp32_control.py --port COM3   # con la ESP32
python atlas_esp32_control.py --teclado     # sin hardware
python atlas_esp32_control.py --joints      # lista los joints del URDF
```

| Tecla (modo `--teclado`) | Acción |
|:---:|:---|
| Flechas | Joystick |
| `M` | Cambia de modo (CAMINAR / BRAZOS) |
| `B` · `Z` · `X` · `V` | B1 · B2 · B3 · B4 |
| `W` / `S` | Sube / baja el potenciómetro |
| `U` / `J` | Sube / baja el piso (la consola imprime el valor para usarlo con `--piso`) |

<details>
<summary><b>🔧 Opciones de rendimiento y escena del Atlas</b></summary>

| Argumento | Descripción |
|:---|:---|
| `--hz 120` | Frecuencia de la física (60/90/120/240). Menos = más ligero |
| `--sinpaneles` | Apaga los 3 paneles de cámara (RGB, profundidad, segmentación) |
| `--sinpanel` | No abre la ventana aparte con el estado del control |
| `--sombras` | Activa sombras (más lento) |
| `--sinentorno` | Solo un piso (lo más liviano) |
| `--sincaja` | No crea la plataforma azul |
| `--piso` / `--x` / `--y` / `--z` | Altura del piso y posición/altura inicial del robot |
| `--dist` / `--camyaw` | Distancia y ángulo de la cámara |
| `--urdf` / `--entorno` | Rutas al URDF del Atlas y al laboratorio, si no los encuentra solo |

</details>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 📡 Protocolos de comunicación

Todos usan texto plano terminado en salto de línea (`\n`), a **115200 baudios**.

<div align="center">

| Punto | Formato de la línea | Frecuencia | Ejemplo |
|:---:|:---|:---:|:---|
| **1** | `<tecla>` — un solo carácter: `A` `B` `C` `D` `#` `*` | Al presionar | `B` |
| **2** | `jx,jy,pot,rot,arm,grip,mode` | ~50 Hz | `0.000,-0.412,0.530,0,1,0,0` |
| **3** | `X,Y,POT,SW,B1,B2,B3,B4` | 50 Hz | `0,85,60,0,0,0,0,0` |

</div>

<details>
<summary><b>📖 Significado de cada campo</b></summary>

**Punto 2 — Baxter**

| Campo | Significado | Rango |
|:---:|:---|:---:|
| `jx`, `jy` | Joystick (0 = centro) | -1 a 1 |
| `pot` | Potenciómetro (posición absoluta) | 0 a 1 |
| `rot` | Botones de giro: izquierda / ninguno / derecha | -1, 0, +1 |
| `arm` | Brazo activo (el botón conmuta): izquierdo / derecho | 0 / 1 |
| `grip` | Estado de la pinza (cambia en cada pulsación) | 0 / 1 |
| `mode` | Modo (el click del joystick conmuta): brazo / muñeca | 0 / 1 |

**Punto 3 — Atlas**

| Campo | Significado | Rango |
|:---:|:---|:---:|
| `X`, `Y` | Joystick, con zona muerta | -100 a 100 |
| `POT` | Potenciómetro | 0 a 100 |
| `SW`, `B1`–`B4` | Click del joystick y botones (1 = pulsado) | 0 / 1 |

</details>

> 🛡️ **Robustez:** Python solo acepta una línea si llegó **completa** (con `\n`) y con el formato
> esperado (7 u 8 campos numéricos, o exactamente un carácter válido en el Punto 1). Así el texto
> de arranque de la ESP32 o el ruido del puerto nunca se confunden con un comando real.

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 🧩 Explicación del código, bloque por bloque

### 1️⃣ Punto 1 — `pid_ABC.py` y `teclado_matricial_ESP32.ino`

<details>
<summary><b>📍 Puntos A, B, C, D y formación en cuadrado</b></summary>

```python
TILE = 1.0
A = np.array([0*TILE, 0*TILE, H])
B = np.array([3*TILE, 0*TILE, H])
C = np.array([4*TILE, 3*TILE, H])
D = np.array([1*TILE, 3*TILE, H])
POINTS_DICT = {'A': A, 'B': B, 'C': C, 'D': D}

E = .18
SWARM_OFFSETS = np.array([
    [-E/2, -E/2, 0], [ E/2, -E/2, 0],
    [-E/2,  E/2, 0], [ E/2,  E/2, 0],
])
```

`POINTS_DICT` traduce la letra del teclado al punto del piso (`H = 0.02` m: casi tocando el
suelo). `SWARM_OFFSETS` define dónde va cada dron respecto al **centro del enjambre**: así los
4 drones forman un cuadrado de 18 cm y se mueven como un solo cuerpo.

</details>

<details>
<summary><b>🛫 Trayectoria despegar → avanzar → aterrizar</b></summary>

```python
def build_leg(p_from, p_to):
    return np.vstack([
        vertical(p_from[2], Z_CRUISE, p_from, N_PHASE),     # sube a 1 m
        horizontal(p_from, p_to, Z_CRUISE, N_PHASE),         # avanza en línea recta
        vertical(Z_CRUISE, p_to[2], p_to, N_PHASE),          # baja al piso
    ])
```

Cada vuelo dura `LEG_DURATION_SEC = 4` s repartidos en **tres fases iguales** de `N_PHASE`
puntos. `vertical()` y `horizontal()` interpolan linealmente entre dos puntos. El resultado es
una lista de *waypoints* que el controlador va persiguiendo uno por ciclo.

</details>

<details>
<summary><b>📥 Lectura no bloqueante del serial</b></summary>

```python
if ser is not None and ser.in_waiting > 0:
    serial_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')

if '\n' in serial_buffer:
    lineas = serial_buffer.split('\n')
    serial_buffer = lineas[-1]          # guarda el fragmento incompleto
    for linea in lineas[:-1]:
        token = linea.strip().upper()
        if len(token) == 1:
            if token in POINTS_DICT:      requested_key = token
            elif token in ('#', '*'):     requested_control = token
```

`timeout=0` evita que la simulación se congele esperando datos. Los bytes se acumulan en un
*buffer* y solo se procesan **líneas completas** de **un carácter**. Además, al abrir el puerto
se espera 2.5 s y se vacía el buffer, porque la ESP32 **se reinicia** al abrirse el puerto y
escupe mensajes de arranque.

</details>

<details>
<summary><b>🎛️ Máquina de estados: aterrizado ↔ en vuelo</b></summary>

```python
if (not flying) and (requested_key is not None):
    if requested_key == current_key:
        print(f"Ya estoy en el punto {requested_key}.")
    else:
        leg_traj = build_leg(current_point, POINTS_DICT[requested_key])
        step_in_leg = 0
        flying = True
```

Solo se acepta una letra nueva cuando el enjambre **está aterrizado**; mientras vuela, las teclas
se ignoran. Al terminar la trayectoria se actualiza `current_key` y el enjambre se queda
esperando el siguiente comando.

</details>

<details>
<summary><b>🧭 Control PID de cada dron</b></summary>

```python
for j in range(num_drones):
    target = base_wp + SWARM_OFFSETS[j]
    action[j, :], _, _ = ctrl[j].computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP,
        state=obs[j],
        target_pos=target,
        target_rpy=INIT_RPYS[j, :])
```

Cada dron tiene su propio `DSLPIDControl`. El objetivo de cada uno es el *waypoint* del
enjambre **más su offset**, de modo que la formación se mantiene durante todo el vuelo. La
simulación corre a 240 Hz y el control a 48 Hz.

</details>

<details>
<summary><b>🔁 Reinicio (<code>*</code>) y cierre (<code>#</code>)</b></summary>

```python
if requested_control == '#':
    mission_done = True
elif requested_control == '*':
    obs, info = env.reset()
    draw_bases()      # env.reset() borra el mundo: hay que redibujar las cajitas blancas
    current_key = 'A'; flying = False
```

`#` termina el bucle: se cierra el puerto, se guardan los datos (`logger.save()` y
`logger.save_as_csv("pid_ABC")` en `results/`) y se muestran las gráficas. `*` reinicia el
entorno y devuelve el enjambre al punto A.

</details>

<details>
<summary><b>⌨️ Firmware del teclado (<code>teclado_matricial_ESP32.ino</code>)</b></summary>

```cpp
char keys[ROWS][COLS] = {
  {'1','2','3','A'}, {'4','5','6','B'},
  {'7','8','9','C'}, {'*','0','#','D'}
};
byte rowPins[ROWS] = {13, 12, 14, 27};
byte colPins[COLS] = {26, 25, 33, 32};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void loop() {
  char key = keypad.getKey();
  if (key == 'A' || key == 'B' || key == 'C' || key == 'D' || key == '#' || key == '*')
    Serial.println(key);
}
```

La librería `Keypad` escanea la matriz y devuelve la tecla presionada. El firmware **filtra**:
solo envía `A`, `B`, `C`, `D`, `#` y `*`; los números del 0 al 9 se ignoran. Los GPIO se
eligieron evitando los pines de arranque (0, 2, 4, 15) y los de solo lectura (34–39).

</details>

### 2️⃣ Punto 2 — `baxter_esp32_control.py` y `baxter_esp32.ino`

<details>
<summary><b>🗺️ Reparto de los 7 ejes por modo</b></summary>

```python
MODES = [
    {"jx": "s0", "jy": "s1", "pot": "e1", "rot": "e0"},   # modo 1: brazo
    {"jx": "w0", "jy": "w1", "pot": None, "rot": "w2"},   # modo 2: muñeca
]
```

Un diccionario dice **qué articulación mueve cada control en cada modo**. Para cambiar el
reparto basta editar esta tabla; el resto del código no se toca. Si una articulación se mueve
al revés, se cambia su signo en `SIGN`.

</details>

<details>
<summary><b>🕹️ Joystick → velocidad, potenciómetro → posición</b></summary>

```python
# Joystick y botones de giro -> velocidad de articulación
move_joint(body, A, name, A["q"][name] + SIGN[name] * v * speed * DT)

# Potenciómetro -> ángulo absoluto, con límite de velocidad
goal = pot_to_angle(A, name, pot)
dq = float(np.clip(goal - A["q"][name], -POT_SPEED * DT, POT_SPEED * DT))
move_joint(body, A, name, A["q"][name] + dq)
```

El joystick **suma un pequeño incremento** al ángulo cada ciclo (control por velocidad); el
potenciómetro define un **ángulo objetivo absoluto** dentro de los límites del joint, pero el
codo lo persigue con velocidad limitada, así nunca "salta". `deadzone()` ignora lecturas
menores a 0.12 y `filt += SMOOTH * (raw - filt)` suaviza los cambios bruscos.

</details>

<details>
<summary><b>🤝 "Enganchar" el potenciómetro</b></summary>

```python
if (side, m) != prev_ctx:           # cambió el brazo o el modo
    pot_engaged = False
    pot_ref = pot
if not pot_engaged and abs(pot - pot_ref) > POT_ENGAGE:
    pot_engaged = True
```

Al cambiar de brazo o de modo, el potenciómetro está en una posición cualquiera que no
coincide con el codo. Si el codo la siguiera de inmediato, **saltaría de golpe**. Por eso el
pote solo toma el control cuando se mueve más de `POT_ENGAGE` (0.04) desde que cambió el contexto.

</details>

<details>
<summary><b>✋ Pinza y agarre de objetos</b></summary>

```python
def toggle_gripper(body, A, arms, objs):
    A["grip"] = not A["grip"]
    ...
    if want and A["cid"] is None:
        # busca el objeto libre más cercano a la pinza (menos de GRASP_DISTANCE = 10 cm)
        ...
        A["cid"] = p.createConstraint(body, A["ee"], best["id"], -1, p.JOINT_FIXED, ...)
```

Cada vez que cambia la señal `grip` de la ESP32 se alterna la pinza. Al cerrarla, se busca el
objeto más cercano y, si está a menos de 10 cm, se crea una **restricción fija** entre la
pinza y el objeto, lo que simula el agarre. Al abrirla, la restricción se elimina y el objeto cae.

</details>

<details>
<summary><b>🎯 Escena de juego</b></summary>

```python
OBJECT_DX = {"esfera": -0.12, "cubo": 0.0, "cilindro": 0.12}
ZONE_OFFSET = (0.0, 0.20)
ZONE_HALF = 0.07
```

`spawn_scene()` crea la **mesa**, la **zona verde** (visual, sin colisión) y los tres objetos
con fricción alta. Un objeto cuenta como "en la zona" si su posición cae dentro del cuadrado
verde y no está sujetado. El panel muestra el marcador `En la zona verde: n / 3`. Si un objeto
cae al piso, se reinicia solo; con `C` se reinician todos.

</details>

<details>
<summary><b>🖥️ Panel de estado (Tkinter)</b></summary>

La clase `Panel` abre una ventana siempre visible con: el **brazo** activo (azul = izquierdo,
naranja = derecho), el **modo**, el mapa de controles, una **barra por cada eje** con su ángulo
actual, el estado de la pinza y el objeto sujetado, y un aviso `! No llegan datos del ESP32`
si pasan más de 2 s sin recibir líneas. Se refresca cada 4 ciclos para no frenar la simulación.

</details>

<details>
<summary><b>🔌 Firmware (<code>baxter_esp32.ino</code>)</b></summary>

```cpp
// calibración del centro al arrancar (promedio de 64 lecturas)
centroJX = promedio(PIN_JX);  centroJY = promedio(PIN_JY);

// botones que conmutan, con antirrebote de 250 ms
if (b.last == HIGH && now == LOW && ms - b.t > 250) { b.state = !b.state; b.t = ms; }

// potenciómetro con filtro suave
potFilt += 0.2 * (analogRead(PIN_POT) / 4095.0 - potFilt);

Serial.printf("%.3f,%.3f,%.3f,%d,%d,%d,%d\n", jx, jy, potFilt, rot, arm, grip, mode);
```

- **`normaliza()`** convierte la lectura del ADC (0–4095) a -1…1 respecto al centro calibrado.
- **`Toggle`**: los botones *brazo*, *pinza* y *click del joystick* **conmutan** su estado en
  cada pulsación (con antirrebote), así Python recibe un estado estable y no pulsos.
- **`INVERT_X` / `INVERT_Y`** corrigen la orientación según cómo esté montado el joystick.

</details>

### 3️⃣ Punto 3 — `atlas_esp32_control.py` y `atlas_console.ino`

<details>
<summary><b>🧵 Lectura del serial en un hilo aparte</b></summary>

```python
class SerialInput:
    def __init__(self, port, baud=115200):
        ...
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            v = [int(t) for t in line.split(",")]
            if len(v) == 8:
                self.data = v
```

Un **hilo** lee el puerto continuamente y guarda la última lectura válida, así el bucle de
simulación nunca se queda esperando datos. `conectado()` devuelve `False` si pasan más de 0.5 s
sin recibir nada, y el panel lo avisa.

</details>

<details>
<summary><b>🔀 Cambio de modo con flancos de subida</b></summary>

```python
if c["sw"] and not prev["sw"]:
    mode = 1 - mode                 # CAMINAR <-> BRAZOS
b1_edge = c["b1"] and not prev["b1"]
b4_edge = c["b4"] and not prev["b4"]
prev = dict(sw=c["sw"], b1=c["b1"], b4=c["b4"])
```

Un botón mantenido apretado enviaría `1` en cada línea. Para que **una pulsación = una acción**,
solo se reacciona al **flanco de subida** (0 → 1): se compara la lectura actual con la anterior.

</details>

<details>
<summary><b>🚶 Modo CAMINAR: velocidades suavizadas</b></summary>

```python
speed = 0.2 + 0.8 * c["pot"]
cmd_f = c["y"] * MAX_FWD * speed                 # adelante / atrás
cmd_s = (c["b3"] - c["b2"]) * MAX_STRAFE * speed # paso lateral
cmd_w = -c["x"] * MAX_TURN * speed               # giro

vf += (cmd_f - vf) * a      # suavizado exponencial: movimiento fluido
```

El joystick y los botones fijan la **velocidad deseada**; el potenciómetro escala la velocidad
máxima (entre 20 % y 100 %). El filtro exponencial (`TAU = 0.15 s`) evita arranques y frenadas bruscas.

</details>

<details>
<summary><b>🦿 Marcha procedural y salto</b></summary>

```python
phase += 2 * math.pi * GAIT_FREQ * mag * DT
for s, off in (("l", 0.0), ("r", math.pi)):          # piernas en contrafase
    ph = phase + off
    hip  = k * STANCE["hpy"] - HIP_SIGN * direction * HIP_AMP * mag * math.sin(ph)
    knee = k * STANCE["kny"] + KNEE_AMP * mag * max(0.0, math.cos(ph))
    ankle = -(hip + knee)                              # mantiene el pie plano
```

La pelvis del robot se sujeta con una restricción (`p.changeConstraint`) que se mueve según la
velocidad deseada, y las **piernas siguen una marcha periódica**: la cadera oscila con un seno
y la rodilla se flexiona en media onda; ambas piernas van desfasadas 180°. El tobillo compensa
para que el pie quede plano. El **salto** (`B4`) es una pequeña máquina de estados: se agacha
(0.18 s) → despega con velocidad vertical `JUMP_V` → vuelo con gravedad → aterriza.

</details>

<details>
<summary><b>🙌 Modo BRAZOS</b></summary>

```python
mover_joint(target, J, n("shx"), c["y"] * ARM_SUBE[side] * sd)        # sube / baja
mover_joint(target, J, n("shz"), c["x"] * ARM_ADELANTE[side] * sd)    # gira el hombro
target[n("elx")] = clip(c["pot"] * extremo, lo, hi)                    # codo con el pote
B23_FUNC = [("mwx", "flexion muneca"), ("uwy", "giro muneca"), ("ely", "giro codo")]
```

El joystick mueve el hombro por velocidad y el potenciómetro fija la **flexión del codo**
(0 = estirado, 100 = recogido). `B4` recorre la lista `B23_FUNC` para decidir qué articulación
mueven `B2`/`B3`. `ARM_SUBE` y `ARM_ADELANTE` guardan el signo de cada brazo, porque el
izquierdo y el derecho están en espejo.

</details>

<details>
<summary><b>⚡ Rendimiento y paneles</b></summary>

- **`esperar_hasta()`** mantiene un ritmo constante de simulación descontando lo que tardó cada ciclo.
- **`setJointMotorControlArray`** envía todos los objetivos en **una sola llamada**, con límite de velocidad por articulación.
- El **panel de estado** corre en un **proceso aparte** (`multiprocessing`) y recibe los datos por una cola, así no frena la simulación.
- Los **3 paneles de cámara** (RGB, profundidad y segmentación) vienen de una cámara sintética colocada en la cabeza del Atlas; se pueden apagar con `--sinpaneles`.

</details>

<details>
<summary><b>🔌 Firmware (<code>atlas_console.ino</code>)</b></summary>

```cpp
int normalizar(int raw, int centro, bool invertir) {
  int d = raw - centro;
  if (abs(d) < DEADZONE) return 0;                   // zona muerta
  int v = (d > 0) ? (long)(d - DEADZONE) * 100 / (4095 - centro - DEADZONE)
                  : (long)(d + DEADZONE) * 100 / (centro - DEADZONE);
  return invertir ? -constrain(v, -100, 100) : constrain(v, -100, 100);
}

Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", x, y, pot, sw, b1, b2, b3, b4);
```

Calibra el centro al arrancar, aplica una **zona muerta de 250 cuentas**, escala a -100…100,
filtra el potenciómetro y envía una línea cada 20 ms (50 Hz). Los botones se envían como `1`
cuando están pulsados (`!digitalRead(...)`, porque usan `INPUT_PULLUP`).

</details>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 🧠 Conceptos clave

<details>
<summary><b>🔁 Real-to-Sim</b></summary>

Técnica en la que un **dispositivo físico real** (aquí, la consola con ESP32) controla un
**modelo simulado**. Permite validar la interfaz y la lógica de control sin riesgo ni costo
de un robot real, y es el paso previo a conectar el mismo control a hardware real.

</details>

<details>
<summary><b>🦴 URDF (Unified Robot Description Format)</b></summary>

Formato XML que describe un robot como un árbol de **links** (cuerpos rígidos) unidos por
**joints** (articulaciones), con sus límites, masas e inercias. PyBullet carga el modelo con
`p.loadURDF(...)`. Baxter y Atlas se cargan desde su URDF.

</details>

<details>
<summary><b>🎯 POSITION_CONTROL en PyBullet</b></summary>

Modo de control donde se le indica al motor de cada joint una posición objetivo
(`targetPosition`), una fuerza máxima (`force`) y una velocidad máxima (`maxVelocity`). PyBullet
calcula el torque necesario para llegar, similar a un servo real.

</details>

<details>
<summary><b>🧮 PID (Proporcional-Integral-Derivativo)</b></summary>

Controlador que corrige el error entre la posición **deseada** y la **real** combinando tres
términos: uno proporcional al error, uno a su acumulación y otro a su tasa de cambio. En el
Punto 1, `DSLPIDControl` lo usa para llevar cada dron a su *waypoint*.

</details>

<details>
<summary><b>🔌 Comunicación serie (UART/USB)</b></summary>

Forma simple de que una PC y un microcontrolador se hablen: un cable USB por el que viajan
bytes, línea por línea. La ESP32 envía texto y Python lo lee con `pyserial`.

</details>

<details>
<summary><b>⚡ Baudios</b></summary>

Velocidad a la que viajan los datos por el puerto serie (aquí **115200**). La PC y la ESP32
deben usar el mismo valor, o los datos llegarán corruptos.

</details>

<details>
<summary><b>📊 ADC (Convertidor Analógico-Digital)</b></summary>

Convierte el voltaje analógico de un potenciómetro o de un eje del joystick en un número
digital. La ESP32 usa 12 bits de resolución: valores de 0 a 4095.

</details>

<details>
<summary><b>🕳️ Zona muerta (deadzone)</b></summary>

Rango alrededor del centro del joystick que se **ignora**. Los joysticks nunca marcan
exactamente cero en reposo; sin zona muerta, el robot se movería solo.

</details>

<details>
<summary><b>🔘 Pull-up y antirrebote (debounce)</b></summary>

`INPUT_PULLUP` activa una resistencia interna que mantiene el pin en alto; al pulsar el botón
se conecta a GND y se lee `LOW`. El **antirrebote** ignora los rebotes mecánicos del contacto
para que una pulsación cuente una sola vez.

</details>

<details>
<summary><b>📈 Flanco de subida</b></summary>

Instante en que una señal pasa de 0 a 1. Detectarlo permite que **una pulsación produzca una
sola acción**, aunque el botón siga apretado.

</details>

<details>
<summary><b>🧵 Hilos y procesos</b></summary>

Un **hilo** (`threading`) lee el serial en segundo plano dentro del mismo programa. Un **proceso**
(`multiprocessing`) ejecuta el panel en una ventana totalmente independiente. Ambos evitan que
una tarea lenta congele la simulación.

</details>

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

## 🛠️ Solución de problemas

| Problema | Posible solución |
|:---|:---|
| `ModuleNotFoundError` (`pybullet`, `serial`, `gym_pybullet_drones`) | Ejecuta `pip install -r requirements.txt` y, para el Punto 1, instala `gym-pybullet-drones` (ver [Requisitos](#️-requisitos)) |
| `serial` instalado pero no funciona (`AttributeError: ... Serial`) | `pip uninstall serial` y luego `pip install pyserial` |
| No se pudo abrir el puerto serie | Cierra el **Monitor Serial** del Arduino IDE y revisa el puerto en el *Administrador de dispositivos* |
| No encuentra `toms_baxter.urdf` / `atlas_v4_with_multisense.urdf` | Descarga `pybullet_robots` y déjalo en la raíz del repo como `pybullet_robots-master/`, o usa `--urdf` en el Atlas |
| El robot se mueve solo o deriva | No toques el joystick al encender la ESP32 (se calibra al arrancar); reinicia la placa |
| El joystick va al revés | Cambia `INVERT_X` / `INVERT_Y` en el `.ino` (Baxter y Atlas) |
| Una articulación del Baxter gira al revés | Pon `-1` en esa articulación dentro de `SIGN` |
| Los drones no responden al teclado | Espera el mensaje *"Listo. El teclado matricial ya está activo."*; recuerda que solo atienden letras cuando están **aterrizados** |
| El teclado matricial no envía nada | Revisa que las 8 conexiones coincidan con la tabla y que la librería **Keypad** esté instalada |
| El Atlas va lento o entrecortado | Usa `--hz 60`, `--sinpaneles`, `--sinentorno` o quita las sombras |
| Los pies del Atlas atraviesan o flotan sobre el piso | Ajusta la altura con `U` / `J` y luego fíjala con `--piso <valor>` |
| El panel de Tkinter no abre | En Linux instala `sudo apt install python3-tk`; o ejecuta con `--sin-panel` / `--sinpanel` |
| El Baxter no coge el objeto | Acerca la pinza a menos de 10 cm del objeto antes de cerrarla |

<img src="https://capsule-render.vercel.app/api?type=rect&color=0:0B3D2E,100:1F8A4C&height=3&section=header" width="100%"/>

<div align="center">

## 👤 Autor

**Julián** · Ingeniería Mecatrónica · Universidad Militar Nueva Granada

![Footer](https://capsule-render.vercel.app/api?type=waving&color=0:1F8A4C,50:14683F,100:0B3D2E&height=120&section=footer)

</div>
