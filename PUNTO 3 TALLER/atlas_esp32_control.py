#!/usr/bin/env python3
"""
Consola de mandos ESP32 -> robot Atlas (Boston Dynamics) en PyBullet.
VERSION OPTIMIZADA PARA FLUIDEZ (portatil con GTX 950M / Windows).

Uso:
  python atlas_esp32_control.py --port COM3        # con la ESP32
  python atlas_esp32_control.py --teclado          # sin hardware (pruebas)
  python atlas_esp32_control.py --joints           # lista los joints del URDF

Opciones de rendimiento nuevas:
  --hz 120         frecuencia de la fisica (60/90/120/240). Menos = mas ligero
  --sinpaneles     apaga los 3 paneles de camara (RGB/profundidad/segmentacion)
  --sombras        activa sombras (por defecto apagadas)
  --sinentorno     solo un piso (lo mas ligero)

Dependencias:  pip install pybullet numpy pyserial

Modos (el click del joystick alterna):
  CAMINAR: joystick Y = adelante/atras, X = girar, pot = velocidad maxima,
           B2/B3 = paso lateral izq/der, B1 = reiniciar posicion, B4 = SALTAR
  BRAZOS : joystick arriba/abajo = sube/baja el brazo, joystick izq/der = gira el
           brazo (hombro), pot = flexion del codo (recoger/estirar),
           B4 = cambia lo que hacen B2/B3 (flexion muneca / giro muneca / giro codo),
           B1 = cambia brazo izq/der

Teclado: flechas = joystick, M = modo, B = B1, Z/X = B2/B3, V = B4, W/S = potenciometro
Piso (con la ventana de PyBullet activa): U = subir el piso, J = bajarlo. La consola
imprime el valor para usarlo despues con --piso.
"""
import argparse
import glob
import math
import multiprocessing as mp
import os
import queue
import threading
import time

import numpy as np
import pybullet as p
import pybullet_data

DT = 1.0 / 120.0     # se reasigna en main() segun --hz
URDF_DEFAULT = "atlas_v4_with_multisense.urdf"

# ---------------- Ajustes de "sensacion" ----------------
MAX_FWD = 0.6        # m/s adelante/atras
MAX_STRAFE = 0.3     # m/s lateral
MAX_TURN = 0.9       # rad/s giro
TAU = 0.15           # suavizado de comandos (s): mayor = mas fluido/lento
ARM_RATE = 1.2       # rad/s de los brazos con el joystick
JOINT_MAX_VEL = 2.5  # rad/s limite de velocidad de cada articulacion
JOINT_FORCE = 400.0  # N*m
GAIT_FREQ = 1.6      # Hz de la marcha a velocidad maxima

# Sentido de los ejes del joystick de la ESP32 (cambia True/False si algo sale al reves)
# Con True: izquierda = negativo/giro a la izquierda, arriba = adelante.
INVERTIR_X = True
INVERTIR_Y = True

# Sentido de los brazos (uno por lado). Si un brazo se mueve al reves, cambia su signo.
ARM_SUBE = {"l": 1.0, "r": -1.0}       # signo de shx que SUBE el brazo
ARM_ADELANTE = {"l": -1.0, "r": 1.0}   # signo de shz que lo lleva hacia ADELANTE (joystick derecha)

# Funciones de B2/B3 en modo BRAZOS (B4 las va cambiando en orden)
B23_FUNC = [("mwx", "flexion muneca"), ("uwy", "giro muneca"), ("ely", "giro codo")]

# Salto (B4 en modo CAMINAR)
JUMP_V = 2.8           # m/s de despegue (altura ~ V^2 / 19.6 = 0.40 m)
JUMP_CROUCH_T = 0.18   # s que se agacha antes de saltar
JUMP_HIP = -0.5        # postura agachada / encogida en el aire (rad)
JUMP_KNEE = 0.9

# Postura semiflexionada (rad). Ajusta signos si las piernas se doblan al reves.
STANCE = {"hpy": -0.25, "kny": 0.50, "aky": -0.25}
HIP_AMP, KNEE_AMP = 0.35, 0.60
HIP_SIGN = 1.0       # pon -1.0 si el robot "camina" hacia atras
ARM_HOME = {}        # p.ej. {"l_arm_shx": 1.2, "r_arm_shx": -1.2} para bajar los brazos

PISO_Z = [-2.35]  # altura del piso del laboratorio (ajustable con --piso o teclas U/J)
PLANO = []         # id del plano fisico invisible (para poder subirlo/bajarlo en vivo)
PISO_RATE = 0.05   # m/s al mantener U/J
BOX_HALF = 0.5     # boston_box.urdf: cubo de 1 m
BOX_TOP = -1.5     # tope de la caja (caja en z=-2, como en atlas.py)

ARM_JOINTS = ["shz", "shx", "ely", "elx", "uwy", "mwx"]


# ---------------- Entradas ----------------
class SerialInput:
    """Lee 'X,Y,POT,SW,B1,B2,B3,B4' de la ESP32 en un hilo aparte."""

    nombre = "ESP32"

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.data = [0, 0, 50, 0, 0, 0, 0, 0]
        self.t_last = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                v = [int(t) for t in line.split(",")]
                if len(v) == 8:
                    self.data = v
                    self.t_last = time.time()
            except Exception:
                pass

    def read(self):
        x, y, pot, sw, b1, b2, b3, b4 = self.data
        sx = -1.0 if INVERTIR_X else 1.0
        sy = -1.0 if INVERTIR_Y else 1.0
        return dict(x=sx * x / 100.0, y=sy * y / 100.0, pot=pot / 100.0,
                    sw=sw, b1=b1, b2=b2, b3=b3, b4=b4)

    def conectado(self):
        return time.time() - self.t_last < 0.5


class KeyboardInput:
    nombre = "TECLADO"

    def conectado(self):
        return True

    def __init__(self):
        self.pot = 0.6
        self.prev = {}

    def read(self):
        k = p.getKeyboardEvents()
        d = lambda key: 1 if k.get(key, 0) & p.KEY_IS_DOWN else 0
        self.pot = float(np.clip(self.pot + (d(ord("w")) - d(ord("s"))) * 0.01, 0, 1))
        return dict(
            x=d(p.B3G_RIGHT_ARROW) - d(p.B3G_LEFT_ARROW),
            y=d(p.B3G_UP_ARROW) - d(p.B3G_DOWN_ARROW),
            pot=self.pot, sw=d(ord("m")), b1=d(ord("b")),
            b2=d(ord("z")), b3=d(ord("x")), b4=d(ord("v")))


# ---------------- Utilidades ----------------
def clip(v, lo, hi):
    return max(lo, min(hi, v))


def mover_joint(target, J, name, delta):
    """Suma delta al objetivo de la articulacion respetando sus limites."""
    if name in J:
        _, lo, hi = J[name]
        target[name] = clip(target[name] + delta, lo, hi)


def panel_proceso(q):
    """Ventana aparte (Tkinter) con el estado del control. Corre en OTRO proceso
    para no quitarle fluidez a la simulacion."""
    import tkinter as tk

    root = tk.Tk()
    root.title("Atlas - Panel de control")
    root.geometry("420x720+20+20")
    root.configure(bg="#1e1e2e")
    root.attributes("-topmost", True)
    F = "Consolas"

    lbl_modo = tk.Label(root, text="---", font=(F, 28, "bold"), fg="white",
                        bg="#444444", pady=12)
    lbl_modo.pack(fill="x", padx=10, pady=(10, 4))
    lbl_enlace = tk.Label(root, text="", font=(F, 11), fg="#a6adc8", bg="#1e1e2e")
    lbl_enlace.pack()
    lbl_info = tk.Label(root, text="", font=(F, 13), fg="#cdd6f4", bg="#1e1e2e",
                        justify="left", anchor="w")
    lbl_info.pack(fill="x", padx=16, pady=6)

    cv = tk.Canvas(root, width=150, height=150, bg="#11111b", highlightthickness=0)
    cv.pack(pady=4)
    barra = tk.Canvas(root, width=340, height=22, bg="#11111b", highlightthickness=0)
    barra.pack(pady=4)

    fila = tk.Frame(root, bg="#1e1e2e")
    fila.pack(pady=8)
    etiquetas = {}
    for clave, texto in (("sw", "CLICK"), ("b1", "B1"), ("b2", "B2"),
                         ("b3", "B3"), ("b4", "B4")):
        lb = tk.Label(fila, text=texto, font=(F, 11, "bold"), width=6,
                      fg="#6c7086", bg="#313244", pady=6)
        lb.pack(side="left", padx=3)
        etiquetas[clave] = lb

    lbl_ayuda = tk.Label(root, text="", font=(F, 10), fg="#9399b2", bg="#1e1e2e",
                         justify="left", anchor="w")
    lbl_ayuda.pack(fill="x", padx=16, pady=6)

    NOMBRES = {"shz": "giro hombro", "shx": "subir brazo", "ely": "giro codo",
               "elx": "flexion codo", "uwy": "giro muneca", "mwx": "flexion muneca"}
    estado = {}

    def actualizar():
        try:
            while True:
                m = q.get_nowait()
                if m is None:            # la simulacion se cerro
                    root.destroy()
                    return
                estado.clear()
                estado.update(m)
        except queue.Empty:
            pass

        if estado:
            e = estado
            caminar = e["modo"] == 0
            lbl_modo.config(text="MODO: CAMINAR" if caminar else "MODO: BRAZOS",
                            bg="#2e7d32" if caminar else "#e65100")
            lbl_enlace.config(
                text="Entrada: %s   %s" % (e["enlace"],
                                           "conectado" if e["ok"] else "SIN DATOS"),
                fg="#a6e3a1" if e["ok"] else "#f38ba8")

            if caminar:
                salto = {0: "listo", 1: "agachando", 2: "en el aire"}[e["salto"]]
                info = ("Adelante/atras: %+.2f m/s\n"
                        "Lateral       : %+.2f m/s\n"
                        "Giro          : %+.2f rad/s\n"
                        "Salto         : %s" % (e["vf"], e["vs"], e["w"], salto))
                ayuda = ("Joystick: mover / girar\nPot: velocidad maxima\n"
                         "B1: reiniciar posicion   B2/B3: paso lateral\n"
                         "B4: SALTAR\n"
                         "CLICK: pasar a BRAZOS")
                etiqueta_pot = "Velocidad"
            else:
                lado = "IZQUIERDO" if e["side"] == "l" else "DERECHO"
                lineas = ["Brazo activo: %s" % lado]
                for j, v in e["brazo"].items():
                    lineas.append("%-14s: %+7.1f grados" % (NOMBRES.get(j, j), v))
                info = "\n".join(lineas)
                ayuda = ("Joystick: arriba/abajo sube-baja, izq/der gira\n"
                         "Pot: flexion del codo (recoger/estirar)\n"
                         "B1: cambia de brazo\n"
                         "B4: cambia la funcion de B2/B3\n"
                         "B2/B3 ahora: %s\n"
                         "CLICK: pasar a CAMINAR" % e["sel"])
                etiqueta_pot = "Codo (flexion)"
            lbl_info.config(text=info)
            lbl_ayuda.config(text=ayuda)

            # joystick
            cv.delete("all")
            cv.create_oval(10, 10, 140, 140, outline="#585b70")
            cv.create_line(75, 10, 75, 140, fill="#313244")
            cv.create_line(10, 75, 140, 75, fill="#313244")
            px = 75 + max(-1.0, min(1.0, e["x"])) * 65
            py = 75 - max(-1.0, min(1.0, e["y"])) * 65
            cv.create_oval(px - 8, py - 8, px + 8, py + 8, fill="#f9e2af", outline="")

            # potenciometro
            pot = max(0.0, min(1.0, e["pot"]))
            barra.delete("all")
            barra.create_rectangle(0, 0, 340, 22, fill="#313244", outline="")
            barra.create_rectangle(0, 0, 340 * pot, 22, fill="#89b4fa", outline="")
            barra.create_text(170, 11, text="%s: %d %%" % (etiqueta_pot, pot * 100),
                              fill="white", font=(F, 10, "bold"))

            # botones
            for clave, lb in etiquetas.items():
                on = e[clave]
                lb.config(bg="#f9e2af" if on else "#313244",
                          fg="#1e1e2e" if on else "#6c7086")

        root.after(50, actualizar)

    actualizar()
    root.mainloop()


def activar_reloj_preciso():
    """En Windows time.sleep() tiene ~15 ms de granularidad -> tirones.
    Esto la baja a 1 ms."""
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass


def esperar_hasta(t_objetivo):
    """Espera hasta t_objetivo (perf_counter): duerme casi todo y afina con espera activa."""
    while True:
        resto = t_objetivo - time.perf_counter()
        if resto <= 0:
            return
        if resto > 0.002:
            time.sleep(resto - 0.0015)


def find_urdf(name):
    """Busca el URDF: ruta directa, carpeta del script, pybullet_data y subcarpetas."""
    if os.path.isfile(name):
        return os.path.abspath(name)
    here = os.path.dirname(os.path.abspath(__file__))
    for base in (here, os.path.dirname(here), os.getcwd(), pybullet_data.getDataPath()):
        hits = glob.glob(os.path.join(base, "**", os.path.basename(name)), recursive=True)
        if hits:
            return hits[0]
    raise SystemExit(
        "No encontre el URDF del Atlas.\n"
        "Descarga el repo (git clone https://github.com/erwincoumans/pybullet_robots "
        "o boton Code > Download ZIP), descomprimelo junto a este script "
        "(sin mover archivos, las mallas usan rutas relativas) y vuelve a ejecutar,\n"
        "o pasa la ruta con --urdf \"C:\\ruta\\atlas_v4_with_multisense.urdf\".")


def orientar_lab(ids):
    """Igual que atlas.py: el SDF trae el eje Y arriba; se rota a Z arriba."""
    y2x = p.getQuaternionFromEuler([3.14 / 2.0, 0, 3.14 / 2.0])
    for o in ids:
        pos, orn = p.getBasePositionAndOrientation(o)
        newpos, neworn = p.multiplyTransforms([0, 0, 0], y2x, pos, orn)
        p.resetBasePositionAndOrientation(o, newpos, neworn)


def solo_visual(ids):
    """El laboratorio es solo decoracion: sin colisiones (evita temblores y bugs)."""
    for b in ids:
        for link in range(-1, p.getNumJoints(b)):
            p.setCollisionFilterGroupMask(b, link, 0, 0)


def piso_invisible(z=-2.5):
    """Piso fisico invisible, para que no compita con el piso del laboratorio."""
    pid = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, z])
    p.changeVisualShape(pid, -1, rgbaColor=[1, 1, 1, 0])
    PLANO[:] = [pid]


def load_environment(sin_entorno=False, ruta=None):
    """Carga el laboratorio 'botlab' del repo (como en la imagen de referencia)."""
    if ruta:
        ruta = os.path.abspath(ruta)
        print("Entorno:", ruta)
        p.setAdditionalSearchPath(os.path.dirname(ruta))
        if ruta.endswith(".sdf"):
            ids = p.loadSDF(ruta, globalScaling=2.0)
            orientar_lab(ids)
        else:
            ids = [p.loadURDF(ruta, useFixedBase=True)]
        solo_visual(ids)
        piso_invisible(PISO_Z[0])
        return
    if not sin_entorno:
        here = os.path.dirname(os.path.abspath(__file__))
        for base in (here, os.path.dirname(here), os.getcwd()):
            for pat in ("botlab*.sdf", "botlab*.urdf"):
                hits = glob.glob(os.path.join(base, "**", pat), recursive=True)
                if hits:
                    path = hits[0]
                    print("Entorno:", path)
                    p.setAdditionalSearchPath(os.path.dirname(path))
                    if path.endswith(".sdf"):
                        ids = p.loadSDF(path, globalScaling=2.0)
                        orientar_lab(ids)
                    else:
                        ids = [p.loadURDF(path, useFixedBase=True)]
                    solo_visual(ids)
                    piso_invisible(PISO_Z[0])
                    return
        print("No encontre el botlab; uso un piso simple.")
    PLANO[:] = [p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"),
                           [0, 0, PISO_Z[0]])]


def main():
    global DT
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--teclado", action="store_true")
    ap.add_argument("--joints", action="store_true")
    ap.add_argument("--entorno", default=None, help="ruta al .sdf/.urdf del laboratorio")
    ap.add_argument("--x", type=float, default=-2.0, help="posicion inicial X")
    ap.add_argument("--y", type=float, default=3.0, help="posicion inicial Y")
    ap.add_argument("--dist", type=float, default=2.5, help="distancia de la camara")
    ap.add_argument("--piso", type=float, default=-2.35, help="altura Z del piso del laboratorio")
    ap.add_argument("--sincaja", action="store_true", help="no crear la plataforma azul")
    ap.add_argument("--camyaw", type=float, default=148.0, help="angulo de la camara")
    ap.add_argument("--sinentorno", action="store_true", help="usar solo un piso")
    ap.add_argument("--urdf", default=URDF_DEFAULT)
    ap.add_argument("--z", type=float, default=0.93, help="altura inicial de la pelvis")
    ap.add_argument("--hz", type=int, default=120, help="frecuencia de la fisica")
    ap.add_argument("--sinpaneles", action="store_true",
                    help="apagar los paneles RGB/profundidad/segmentacion (mas fluido)")
    ap.add_argument("--sombras", action="store_true", help="activar sombras (lento)")
    ap.add_argument("--sinpanel", action="store_true",
                    help="no abrir la ventana aparte con el estado del control")
    args = ap.parse_args()
    args.paneles = not args.sinpaneles

    DT = 1.0 / max(30, args.hz)
    activar_reloj_preciso()

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    # --- ajustes de rendimiento del visor ---
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1 if args.sombras else 0)
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)   # evita "arrastrar" el robot sin querer
    pv = 1 if args.paneles else 0
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, pv)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, pv)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, pv)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)
    p.setPhysicsEngineParameter(numSolverIterations=30)
    PISO_Z[0] = args.piso
    load_environment(args.sinentorno, args.entorno)
    base_h = args.piso
    if not args.sincaja:
        box_urdf = find_urdf("boston_box.urdf")
        p.loadURDF(box_urdf, [args.x, args.y, BOX_TOP - 0.5], useFixedBase=True)
        base_h = BOX_TOP
    urdf_path = find_urdf(args.urdf)
    print('Cargando', urdf_path)
    robot = p.loadURDF(urdf_path, [args.x, args.y, args.z + base_h], useFixedBase=False)

    # --- mapa de joints por nombre ---
    J = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            J[info[1].decode()] = (i, info[8], info[9])
    if args.joints:
        for n, (i, lo, hi) in J.items():
            print(f"{i:3d} {n:20s} [{lo:+.2f}, {hi:+.2f}]")
        return

    faltan = [f"{s}_arm_{j}" for s in "lr" for j in ARM_JOINTS if f"{s}_arm_{j}" not in J]
    if faltan:
        print("AVISO: estas articulaciones no existen en el URDF:", ", ".join(faltan))
        print("       ejecuta con --joints para ver los nombres reales.")

    # --- objetivos de cada joint (posicion de reposo) ---
    target = {n: 0.0 for n in J}
    for s in "lr":
        for j, v in STANCE.items():
            target[f"{s}_leg_{j}"] = v
    target.update({k: v for k, v in ARM_HOME.items() if k in target})

    # vectores para enviar todos los joints en UNA sola llamada por ciclo
    names = list(J.keys())
    idx = [J[n][0] for n in names]
    cur = np.zeros(len(names))                 # posicion comandada (con limite de velocidad)
    forces = [JOINT_FORCE] * len(names)
    max_step = JOINT_MAX_VEL * DT

    # --- restriccion que mantiene el equilibrio y permite "caminar" ---
    pos = np.array([args.x, args.y, args.z + base_h])
    zt = base_h            # altura del suelo bajo el robot (suave)
    yaw = 0.0
    cid = p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED,
                             [0, 0, 0], [0, 0, 0], pos.tolist())
    inp = KeyboardInput() if args.teclado else SerialInput(args.port)

    # --- estado ---
    vf = vs = w = 0.0      # velocidades suavizadas
    phase = 0.0
    mode = 0               # 0 = caminar, 1 = brazos
    side = "l"
    prev = dict(sw=0, b1=0, b4=0)
    b23_i = 0                                  # funcion actual de B2/B3 en BRAZOS
    jump_state, jump_t, jump_h, jump_vz = 0, 0.0, 0.0, 0.0   # 0 listo, 1 agachado, 2 en el aire
    frame = 0
    mag_prev = 0.0
    cam_every = max(1, args.hz // 60)          # ~60 actualizaciones de camara por segundo
    img_every = max(1, args.hz // 10)          # ~10 imagenes/s para los paneles
    txt_every = max(1, args.hz // 10)          # texto ~10 veces por segundo
    last_txt = ""
    panel = -1
    p.resetDebugVisualizerCamera(args.dist, args.camyaw, -9, pos.tolist())

    # --- ventana aparte con el estado (proceso independiente) ---
    q_panel = None
    if not args.sinpanel:
        q_panel = mp.Queue(maxsize=2)
        q_panel.cancel_join_thread()
        mp.Process(target=panel_proceso, args=(q_panel,), daemon=True).start()
    panel_every = max(1, args.hz // 20)        # ~20 actualizaciones/s

    t_next = time.perf_counter()
    while p.isConnected():
        c = inp.read()
        a = min(1.0, DT / TAU)

        # --- ajuste del piso en vivo: U sube, J baja (ventana de PyBullet activa) ---
        ks = p.getKeyboardEvents()
        dz = (1 if ks.get(ord("u"), 0) & p.KEY_IS_DOWN else 0) - \
             (1 if ks.get(ord("j"), 0) & p.KEY_IS_DOWN else 0)
        if dz:
            args.piso += dz * PISO_RATE * DT
            if PLANO:
                p.resetBasePositionAndOrientation(PLANO[0], [0, 0, args.piso], [0, 0, 0, 1])
            if frame % 15 == 0:
                print("piso = %.3f   (para dejarlo fijo: --piso %.3f)" % (args.piso, args.piso))

        # flancos de subida
        if c["sw"] and not prev["sw"]:
            mode = 1 - mode
        b1_edge = c["b1"] and not prev["b1"]
        b4_edge = c["b4"] and not prev["b4"]
        prev = dict(sw=c["sw"], b1=c["b1"], b4=c["b4"])

        if mode == 0:  # ---------------- CAMINAR ----------------
            speed = 0.2 + 0.8 * c["pot"]
            cmd_f = c["y"] * MAX_FWD * speed
            cmd_s = (c["b3"] - c["b2"]) * MAX_STRAFE * speed
            cmd_w = -c["x"] * MAX_TURN * speed
            if b1_edge:
                pos[:2] = [args.x, args.y]
                yaw = 0.0
            if b4_edge and jump_state == 0:
                jump_state, jump_t = 1, 0.0
        else:          # ---------------- BRAZOS ----------------
            cmd_f = cmd_s = cmd_w = 0.0
            if b1_edge:
                side = "r" if side == "l" else "l"
            n = lambda j: f"{side}_arm_{j}"
            sd = ARM_RATE * DT
            # joystick arriba/abajo -> sube/baja el brazo (shx)
            mover_joint(target, J, n("shx"), c["y"] * ARM_SUBE[side] * sd)
            # joystick izquierda/derecha -> gira el brazo en el hombro (shz)
            mover_joint(target, J, n("shz"), c["x"] * ARM_ADELANTE[side] * sd)
            # potenciometro -> flexion del codo: 0 = brazo estirado, 100 = codo recogido
            if n("elx") in J:
                _, lo, hi = J[n("elx")]
                extremo = hi if abs(hi) >= abs(lo) else lo
                target[n("elx")] = clip(c["pot"] * extremo, lo, hi)
            # B4 (pulsar) cambia lo que controlan B2/B3: flexion muneca / giro muneca / giro codo
            if b4_edge:
                b23_i = (b23_i + 1) % len(B23_FUNC)
            mover_joint(target, J, n(B23_FUNC[b23_i][0]), (c["b3"] - c["b2"]) * sd)

        # suavizado exponencial -> movimiento fluido
        vf += (cmd_f - vf) * a
        vs += (cmd_s - vs) * a
        w += (cmd_w - w) * a

        # --- mover la base ---
        yaw += w * DT
        pos[0] += (vf * math.cos(yaw) - vs * math.sin(yaw)) * DT
        pos[1] += (vf * math.sin(yaw) + vs * math.cos(yaw)) * DT
        # altura del suelo: sobre la caja o en el piso (bajada/subida suave)
        sobre_caja = (not args.sincaja and abs(pos[0] - args.x) < BOX_HALF - 0.1
                      and abs(pos[1] - args.y) < BOX_HALF - 0.1)
        zt += ((BOX_TOP if sobre_caja else args.piso) - zt) * min(1.0, DT / 0.15)
        # --- salto (B4 en modo caminar): agacharse -> despegar -> vuelo -> aterrizar ---
        crouch = 0.0
        if jump_state == 1:
            jump_t += DT
            crouch = math.sin(0.5 * math.pi * min(1.0, jump_t / JUMP_CROUCH_T))
            if jump_t >= JUMP_CROUCH_T:
                jump_state, jump_vz, jump_h = 2, JUMP_V, 0.0
        elif jump_state == 2:
            jump_vz -= 9.81 * DT
            jump_h += jump_vz * DT
            if jump_h <= 0.0:
                jump_state, jump_h, jump_vz = 0, 0.0, 0.0
        flex = crouch if jump_state == 1 else (1.0 if jump_state == 2 else 0.0)
        pos[2] = (args.z + zt - 0.03 * min(1.0, mag_prev * 4)
                  - 0.09 * crouch + jump_h)
        p.changeConstraint(cid, pos.tolist(),
                           p.getQuaternionFromEuler([0, 0, yaw]), maxForce=5000)

        # --- marcha procedural de las piernas ---
        mag = min(1.0, max(abs(vf) / MAX_FWD, abs(vs) / MAX_STRAFE, abs(w) / MAX_TURN))
        mag_prev = mag
        k = min(1.0, mag * 4)      # 0 en reposo (piernas rectas), 1 caminando
        direction = 1.0 if vf >= -1e-3 else -1.0
        phase += 2 * math.pi * GAIT_FREQ * mag * DT
        for s, off in (("l", 0.0), ("r", math.pi)):
            ph = phase + off
            hip = k * STANCE["hpy"] - HIP_SIGN * direction * HIP_AMP * mag * math.sin(ph)
            knee = k * STANCE["kny"] + KNEE_AMP * mag * max(0.0, math.cos(ph))
            hip = hip * (1.0 - flex) + JUMP_HIP * flex      # salto: agacharse / encoger
            knee = knee * (1.0 - flex) + JUMP_KNEE * flex
            ankle = -(hip + knee)           # mantiene el pie plano
            for j, v in (("hpy", hip), ("kny", knee), ("aky", ankle)):
                name = f"{s}_leg_{j}"
                if name in J:
                    target[name] = clip(v, J[name][1], J[name][2])

        # --- enviar objetivos: limite de velocidad en Python + una sola llamada ---
        tgt = np.fromiter((target[n] for n in names), dtype=float, count=len(names))
        cur += np.clip(tgt - cur, -max_step, max_step)
        p.setJointMotorControlArray(robot, idx, p.POSITION_CONTROL,
                                    targetPositions=cur.tolist(), forces=forces)

        frame += 1

        # --- panel de estado (pegado al robot, solo se redibuja si cambia) ---
        if frame % txt_every == 0:
            txt = ("MODO: CAMINAR  v=%.1f m/s  giro=%.1f rad/s" % (vf, w)) if mode == 0 \
                else "MODO: BRAZOS  brazo=%s" % ("IZQUIERDO" if side == "l" else "DERECHO")
            if txt != last_txt:
                panel = p.addUserDebugText(txt, [0, 0, 1.3], textSize=1.6,
                                           textColorRGB=[1, 1, 0],
                                           parentObjectUniqueId=robot, parentLinkIndex=-1,
                                           replaceItemUniqueId=panel)
                last_txt = txt

        # --- la camara sigue al robot pero respeta lo que muevas con el mouse ---
        if frame % cam_every == 0:
            cam = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], pos.tolist())

        # --- camara sintetica en la cabeza (alimenta los 3 paneles; se apaga con --sinpaneles) ---
        if args.paneles and frame % img_every == 0:
            fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            eye = pos + np.array([0.0, 0.0, 0.75]) + fwd * 0.25
            vm = p.computeViewMatrix(eye.tolist(), (eye + fwd * 3).tolist(), [0, 0, 1])
            pm = p.computeProjectionMatrixFOV(60, 4 / 3, 0.1, 20)
            p.getCameraImage(160, 120, vm, pm, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # --- estado hacia la ventana aparte ---
        if q_panel is not None and frame % panel_every == 0:
            est = dict(modo=mode, side=side, vf=vf, vs=vs, w=w,
                       x=c["x"], y=c["y"], pot=c["pot"], sw=c["sw"],
                       b1=c["b1"], b2=c["b2"], b3=c["b3"], b4=c["b4"],
                       enlace=inp.nombre, ok=inp.conectado(),
                       salto=jump_state, sel=B23_FUNC[b23_i][1],
                       brazo={j: math.degrees(target[f"{side}_arm_{j}"])
                              for j in ("shz", "shx", "ely", "elx", "uwy", "mwx")
                              if f"{side}_arm_{j}" in target})
            try:
                q_panel.put_nowait(est)
            except queue.Full:
                pass

        p.stepSimulation()

        # --- ritmo constante: descuenta lo que tardo el ciclo, sin acumular retraso ---
        t_next += DT
        if time.perf_counter() - t_next > 0.1:   # si nos atrasamos mucho, no "recuperar" a saltos
            t_next = time.perf_counter()
        esperar_hasta(t_next)

    if q_panel is not None:
        try:
            q_panel.put_nowait(None)
        except queue.Full:
            pass


if __name__ == "__main__":
    main()