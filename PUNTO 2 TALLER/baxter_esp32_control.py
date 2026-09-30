#!/usr/bin/env python3
"""
Baxter en PyBullet controlado por articulaciones (sin cinemática inversa).

    python baxter_control.py --teclado      # prueba con el teclado
    python baxter_control.py                # con el ESP32 (puerto en SERIAL_PORT)
    python baxter_control.py --sin-panel    # sin la ventana de estado

Reparto de los 7 ejes (cada brazo: s0 s1 e0 e1 w0 w1 w2):

    Control                 MODO 1 (brazo)         MODO 2 (muñeca)
    Joystick X              s0  hombro (lados)     w0  giro del antebrazo
    Joystick Y              s1  hombro (arriba)    w1  cabeceo de la muñeca
    Potenciómetro           e1  codo (posición)    (no se usa)
    Botones giro izq/der    e0  giro del brazo     w2  giro de la pinza

    Botón brazo    -> cambia brazo izquierdo / derecho
    Botón pinza    -> abre / cierra
    Click joystick -> cambia entre MODO 1 y MODO 2

Juego: sobre la mesa hay una esfera, un cubo y un cilindro. Llévalos a la ZONA VERDE.

Teclado (clic antes en la ventana de PyBullet):
    flechas = joystick | R/F = potenciómetro | Q/E = giro izq/der
    M = cambiar modo | 1/2 = brazo izq/der | ESPACIO = pinza | C = reiniciar objetos
"""
import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data

try:
    import serial
except ImportError:
    serial = None

# ------------------------- Configuración -------------------------
REPO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pybullet_robots-master")
SERIAL_PORT = "COM3"
BAUD = 115200

DEADZONE = 0.12               # zona muerta del joystick
SMOOTH = 0.20                 # suavizado del joystick (más bajo = más suave)

JOY_SPEED = 0.5               # rad/s de la articulación con el joystick al máximo
ROT_SPEED = 0.6               # rad/s con los botones de giro
POT_SPEED = 0.8               # rad/s máximos con los que el codo sigue al potenciómetro
POT_ENGAGE = 0.04             # cuánto hay que mover el pote para "engancharlo" (0 a 1)
MAX_JOINT_VEL = 1.0           # rad/s, tope de los motores
JOINT_FORCE = 300

# Pon -1 en una articulación si se mueve al revés de lo que esperas
SIGN = {"s0": 1, "s1": 1, "e0": 1, "e1": 1, "w0": 1, "w1": 1, "w2": 1}

# Qué articulación mueve cada control en cada modo (None = no se usa)
MODES = [
    {"jx": "s0", "jy": "s1", "pot": "e1", "rot": "e0"},    # modo 1: brazo
    {"jx": "w0", "jy": "w1", "pot": None, "rot": "w2"},    # modo 2: muñeca
]
MODE_NAMES = ["BRAZO (s0 s1 e1 e0)", "MUÑECA (w0 w1 w2)"]

LOOP_DT_STEPS = 4             # pasos de simulación por ciclo
DT_SIM = 1.0 / 240.0
DT = LOOP_DT_STEPS * DT_SIM   # tiempo simulado por ciclo (1/60 s)

# Escena
OBJECT_DX = {"esfera": -0.12, "cubo": 0.0, "cilindro": 0.12}   # posición de cada objeto (m)
ZONE_OFFSET = (0.0, 0.20)     # zona verde respecto al centro de la mesa (x, y) en metros
ZONE_HALF = 0.07              # mitad del lado de la zona verde
GRASP_DISTANCE = 0.10         # un objeto se agarra si está a menos de 10 cm de la pinza

JOINT_SUFFIXES = ("s0", "s1", "e0", "e1", "w0", "w1", "w2")


# ------------------------- Entradas -------------------------
class SerialInput:
    """Lee del ESP32: jx,jy,pot,rot,arm,grip,mode"""
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0)
        print("Esperando 2 s a que el ESP32 calibre el joystick... NO lo toques.")
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.buf = b""
        self.last = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        self.t_rx = time.time()

    def read(self):
        self.buf += self.ser.read(self.ser.in_waiting or 1)
        *lines, self.buf = self.buf.split(b"\n")
        self.buf = self.buf[-256:]
        for ln in reversed(lines):
            try:
                v = [float(x) for x in ln.decode().strip().split(",")]
            except ValueError:
                continue
            if len(v) == 7:
                self.last = v
                self.t_rx = time.time()
                break
        return self.last

    def stale(self):
        return time.time() - self.t_rx > 2.0


class KeyboardInput:
    """Simula el ESP32 con el teclado."""
    def __init__(self):
        self.pot = 0.5
        self.arm = 0.0
        self.grip = 0.0
        self.mode = 0.0

    def read(self):
        k = p.getKeyboardEvents()

        def down(key):
            return float(bool(key in k and k[key] & p.KEY_IS_DOWN))

        def hit(key):
            return bool(key in k and k[key] & p.KEY_WAS_TRIGGERED)

        if hit(ord("1")):
            self.arm = 0.0
        if hit(ord("2")):
            self.arm = 1.0
        if hit(ord(" ")):
            self.grip = 1.0 - self.grip
        if hit(ord("m")):
            self.mode = 1.0 - self.mode
        self.pot = float(np.clip(self.pot + (down(ord("r")) - down(ord("f"))) * 0.4 * DT, 0.0, 1.0))
        return [down(p.B3G_RIGHT_ARROW) - down(p.B3G_LEFT_ARROW),
                down(p.B3G_UP_ARROW) - down(p.B3G_DOWN_ARROW),
                self.pot,
                down(ord("e")) - down(ord("q")),
                self.arm, self.grip, self.mode]

    def stale(self):
        return False


def deadzone(v):
    if abs(v) < DEADZONE:
        return 0.0
    return float(np.sign(v)) * (abs(v) - DEADZONE) / (1.0 - DEADZONE)


# ------------------------- Panel de estado (ventana aparte) -------------------------
class Panel:
    """Ventana de Tkinter con el brazo activo, el modo, los 7 ángulos y el estado de la pinza."""
    BG = "#1e1e1e"
    COLORS = {"left": "#2b7de9", "right": "#f28c28"}
    BAR_W = 230

    def __init__(self):
        import tkinter as tk
        self.tk = tk
        self.root = tk.Tk()
        self.root.title("Panel Baxter")
        self.root.geometry("410x620+20+40")
        self.root.configure(bg=self.BG)
        self.root.attributes("-topmost", True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.alive = True

        def label(**kw):
            base = dict(bg=self.BG, fg="white", anchor="w")
            base.update(kw)
            lb = tk.Label(self.root, **base)
            lb.pack(fill="x", padx=12, pady=(2, 2))
            return lb

        self.lbl_arm = tk.Label(self.root, font=("Segoe UI", 22, "bold"), fg="white", pady=8)
        self.lbl_arm.pack(fill="x")
        self.lbl_mode = tk.Label(self.root, font=("Segoe UI", 14, "bold"), bg="#333333",
                                 fg="white", pady=6)
        self.lbl_mode.pack(fill="x")
        self.lbl_map = label(font=("Consolas", 11), justify="left", pady=6)

        frame = tk.Frame(self.root, bg=self.BG)
        frame.pack(fill="x", padx=12, pady=4)
        self.bars = {}
        for n in JOINT_SUFFIXES:
            row = tk.Frame(frame, bg=self.BG)
            row.pack(fill="x", pady=2)
            nl = tk.Label(row, text=n, width=3, font=("Consolas", 12, "bold"), bg=self.BG, fg="white")
            nl.pack(side="left")
            cv = tk.Canvas(row, width=self.BAR_W, height=16, bg="#3a3a3a", highlightthickness=0)
            cv.pack(side="left", padx=6)
            rect = cv.create_rectangle(0, 0, 0, 16, width=0, fill="#777777")
            val = tk.Label(row, text="", width=7, font=("Consolas", 11), bg=self.BG, fg="white")
            val.pack(side="left")
            self.bars[n] = (nl, cv, rect, val)

        self.lbl_in = label(font=("Consolas", 10), fg="#bbbbbb", pady=6)
        self.lbl_grip = label(font=("Segoe UI", 13, "bold"))
        self.lbl_score = label(font=("Segoe UI", 13, "bold"), fg="#5fd068")
        self.lbl_warn = label(font=("Segoe UI", 10), fg="#ff6b6b")

    def update(self, body, side, m, A, cfg, inputs, pot_engaged, held_name, n_zone, n_total, stale):
        if not self.alive:
            return
        jx, jy, pot, rot = inputs
        try:
            col = self.COLORS[side]
            self.lbl_arm.config(text="BRAZO " + ("DERECHO" if side == "right" else "IZQUIERDO"), bg=col)
            self.lbl_mode.config(text=f"MODO {m + 1}: {MODE_NAMES[m]}")
            if cfg["pot"] is None:
                pot_txt = "no se usa"
            else:
                pot_txt = cfg["pot"] + (" (activo)" if pot_engaged else "  <- muévelo para engancharlo")
            self.lbl_map.config(text=(f"Joystick X    -> {cfg['jx']}\n"
                                      f"Joystick Y    -> {cfg['jy']}\n"
                                      f"Potenciómetro -> {pot_txt}\n"
                                      f"Botones giro  -> {cfg['rot']}\n"
                                      f"Click joystick: cambia de modo"))
            used = {v for v in cfg.values() if v}
            for n, (nl, cv, rect, val) in self.bars.items():
                lo, hi = A["lim"][n]
                q = p.getJointState(body, A["j"][n])[0]
                frac = float(np.clip((q - lo) / (hi - lo), 0.0, 1.0))
                cv.coords(rect, 0, 0, frac * self.BAR_W, 16)
                cv.itemconfig(rect, fill=col if n in used else "#777777")
                nl.config(fg="white" if n in used else "#888888")
                val.config(text=f"{q:+.2f}")
            self.lbl_in.config(text=f"joy=({jx:+.2f},{jy:+.2f})  pot={pot:.2f}  giro={int(rot):+d}")
            if A["grip"]:
                self.lbl_grip.config(text="PINZA: CERRADA" + (f"  (sujeta: {held_name})" if held_name else ""))
            else:
                self.lbl_grip.config(text="PINZA: ABIERTA")
            self.lbl_score.config(text=f"En la zona verde: {n_zone} / {n_total}")
            self.lbl_warn.config(text="! No llegan datos del ESP32" if stale else "")
            self.root.update()
        except self.tk.TclError:
            self.alive = False

    def close(self):
        if self.alive:
            self.alive = False
            try:
                self.root.destroy()
            except self.tk.TclError:
                pass


# ------------------------- Mundo y robot -------------------------
def find_urdf():
    base = os.path.abspath(REPO_PATH)
    parent = os.path.dirname(base)
    roots = [base]
    if os.path.isdir(parent):
        roots += [os.path.join(parent, d) for d in os.listdir(parent)
                  if d.lower().startswith("pybullet_robots")]
    for root in roots:
        for dirpath, _, files in os.walk(root):
            if "toms_baxter.urdf" in files:
                return os.path.join(dirpath, "toms_baxter.urdf")
    return None


def set_up_world(initial_steps=100):
    p.resetSimulation()
    p.setTimeStep(DT_SIM)
    p.loadURDF("plane.urdf", [0, 0, -1], useFixedBase=True)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    urdf = find_urdf()
    if urdf is None:
        sys.exit("No encuentro toms_baxter.urdf buscando en: " + os.path.abspath(REPO_PATH))
    print("Modelo Baxter:", urdf)
    body = p.loadURDF(urdf, useFixedBase=True)
    p.resetBasePositionAndOrientation(body, [0.5, -0.8, 0.0], [0., 0., -1., -1.])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.setGravity(0, 0, -10)
    for _ in range(initial_steps):
        p.stepSimulation()
    return body


def analyze_robot(body):
    """Devuelve, por brazo: índices de articulaciones, límites, dedos y efector."""
    arms = {s: {"j": {}, "lim": {}, "q": {}, "fingers": [], "ee": None,
                "grip": False, "cid": None, "held": None}
            for s in ("left", "right")}
    for i in range(p.getNumJoints(body)):
        info = p.getJointInfo(body, i)
        if info[3] <= -1:                        # articulación fija
            continue
        name = info[1].decode().lower()
        if name.startswith("left") or name.startswith("l_"):
            side = "left"
        elif name.startswith("right") or name.startswith("r_"):
            side = "right"
        else:
            continue
        suffix = name.split("_")[-1]
        if "finger" in name:
            arms[side]["fingers"].append(i)
            if "gripper_l_finger" in name:
                arms[side]["ee"] = i
        elif suffix in JOINT_SUFFIXES:
            lo, hi = info[8], info[9]
            if hi <= lo:
                lo, hi = -2.0, 2.0
            arms[side]["j"][suffix] = i
            arms[side]["lim"][suffix] = (lo, hi)
            arms[side]["q"][suffix] = p.getJointState(body, i)[0]
    for side, A in arms.items():
        missing = [s for s in JOINT_SUFFIXES if s not in A["j"]]
        if missing:
            print(f"No encontré las articulaciones {missing} del brazo {side}. Todas las articulaciones:")
            for j in range(p.getNumJoints(body)):
                print(j, p.getJointInfo(body, j)[1].decode())
            sys.exit(1)
        if A["ee"] is None:
            A["ee"] = A["j"]["w2"]               # sin dedos detectados: usa la muñeca
    return arms


def finger_targets(body, fingers):
    out = {}
    for i in fingers:
        lo, hi = p.getJointInfo(body, i)[8:10]
        out[i] = (lo, hi) if abs(lo) > abs(hi) else (hi, lo)   # (abierto, cerrado)
    return out


def spawn_scene(center):
    """Mesa + zona verde + tres objetos. 'center' = punto bajo la pinza izquierda."""
    top = center[2] - 0.02                       # altura de la superficie de la mesa
    tth = [0.35, 0.30, 0.01]
    tcol = p.createCollisionShape(p.GEOM_BOX, halfExtents=tth)
    tvis = p.createVisualShape(p.GEOM_BOX, halfExtents=tth, rgbaColor=[0.6, 0.4, 0.2, 1])
    p.createMultiBody(0, tcol, tvis, [center[0], center[1], top - tth[2]])

    zone_c = (center[0] + ZONE_OFFSET[0], center[1] + ZONE_OFFSET[1])
    zvis = p.createVisualShape(p.GEOM_BOX, halfExtents=[ZONE_HALF, ZONE_HALF, 0.002],
                               rgbaColor=[0.1, 0.75, 0.25, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=zvis,
                      basePosition=[zone_c[0], zone_c[1], top + 0.002])

    objs = []
    for name, dx in OBJECT_DX.items():
        if name == "cubo":
            h = 0.02
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h] * 3)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[h] * 3, rgbaColor=[0.9, 0.2, 0.2, 1])
            z = top + h
        elif name == "cilindro":
            r, length = 0.02, 0.07
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=length)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=length,
                                      rgbaColor=[0.15, 0.4, 0.95, 1])
            z = top + length / 2
        else:                                    # esfera
            r = 0.025
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=r)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=[0.95, 0.75, 0.1, 1])
            z = top + r
        start = [center[0] + dx, center[1], z]
        oid = p.createMultiBody(0.1, col, vis, start)
        p.changeDynamics(oid, -1, lateralFriction=1.5, rollingFriction=0.02, spinningFriction=0.02,
                         linearDamping=0.2, angularDamping=0.5)
        objs.append({"id": oid, "name": name, "start": start})
    return objs, top, zone_c


# ------------------------- Control -------------------------
def move_joint(body, A, name, q):
    """Fija el objetivo de una articulación respetando sus límites."""
    lo, hi = A["lim"][name]
    q = float(np.clip(q, lo, hi))
    A["q"][name] = q
    p.setJointMotorControl2(body, A["j"][name], p.POSITION_CONTROL, targetPosition=q,
                            force=JOINT_FORCE, maxVelocity=MAX_JOINT_VEL)


def pot_to_angle(A, name, pot):
    """Potenciómetro (0 a 1) -> ángulo dentro del rango de la articulación."""
    lo, hi = A["lim"][name]
    f = pot if SIGN[name] > 0 else 1.0 - pot
    return lo + f * (hi - lo)


def release(A):
    if A["cid"] is not None:
        p.removeConstraint(A["cid"])
    A["cid"] = None
    A["held"] = None


def toggle_gripper(body, A, arms, objs):
    A["grip"] = not A["grip"]
    want = A["grip"]
    for i, (op, cl) in A["fingers"].items():
        p.setJointMotorControl2(body, i, p.POSITION_CONTROL,
                                targetPosition=cl if want else op, force=30)
    if want and A["cid"] is None:
        ls = p.getLinkState(body, A["ee"])
        taken = {B["held"] for B in arms.values()}
        best, best_d = None, GRASP_DISTANCE
        for o in objs:
            if o["id"] in taken:
                continue
            d = np.linalg.norm(np.array(p.getBasePositionAndOrientation(o["id"])[0]) - np.array(ls[4]))
            if d < best_d:
                best, best_d = o, d
        if best is not None:
            cpos, corn = p.getBasePositionAndOrientation(best["id"])
            ip, io = p.invertTransform(ls[4], ls[5])
            rp, ro = p.multiplyTransforms(ip, io, cpos, corn)
            A["cid"] = p.createConstraint(body, A["ee"], best["id"], -1, p.JOINT_FIXED,
                                          [0, 0, 0], parentFramePosition=rp,
                                          childFramePosition=[0, 0, 0],
                                          parentFrameOrientation=ro)
            A["held"] = best["id"]
            print(f"    Agarraste: {best['name']}")
    elif not want and A["cid"] is not None:
        release(A)


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    for flag in (p.COV_ENABLE_SHADOWS, p.COV_ENABLE_GUI, p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                 p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
        p.configureDebugVisualizer(flag, 0)
    p.resetDebugVisualizerCamera(2.0, 180, 0.0, [0.52, 0.2, np.pi / 4])
    body = set_up_world()
    arms = analyze_robot(body)

    for side, A in arms.items():
        txt = "  ".join(f"{n}[{A['lim'][n][0]:+.2f},{A['lim'][n][1]:+.2f}]" for n in JOINT_SUFFIXES)
        print(f"Límites brazo {side}: {txt}")
        A["fingers"] = finger_targets(body, A["fingers"])

    if "--teclado" in sys.argv:
        inp = KeyboardInput()
        print("MODO TECLADO (haz clic en la ventana de PyBullet)")
    else:
        if serial is None:
            sys.exit("Falta pyserial: pip uninstall serial  y luego  pip install pyserial")
        try:
            inp = SerialInput(SERIAL_PORT, BAUD)
            print("Leyendo ESP32 en", SERIAL_PORT)
        except Exception as e:
            print("\n" + "!" * 64)
            print("NO PUDE ABRIR EL PUERTO", SERIAL_PORT, "->", e)
            print("1) Cierra el Monitor Serie / Serial Plotter del Arduino IDE")
            print("2) Revisa que SERIAL_PORT sea el correcto. Puertos detectados:")
            try:
                from serial.tools import list_ports
                for pt in list_ports.comports():
                    print("     ", pt.device, "-", pt.description)
            except Exception:
                print("      (no pude listar los puertos)")
            print("(Para probar sin ESP32: python baxter_control.py --teclado)")
            print("!" * 64)
            sys.exit(1)

    # Todos los motores mantienen la posición actual; los dedos empiezan abiertos
    for A in arms.values():
        for name in JOINT_SUFFIXES:
            move_joint(body, A, name, A["q"][name])
        for i, (op, _) in A["fingers"].items():
            p.setJointMotorControl2(body, i, p.POSITION_CONTROL, targetPosition=op, force=30)

    ee_left = np.array(p.getLinkState(body, arms["left"]["ee"])[4])
    objs, table_top, zone_c = spawn_scene((ee_left + np.array([0.0, 0.0, -0.20])).tolist())
    names = {o["id"]: o["name"] for o in objs}

    def reset_object(o):
        for A in arms.values():
            if A["held"] == o["id"]:
                release(A)
        p.resetBasePositionAndOrientation(o["id"], o["start"], [0, 0, 0, 1])
        p.resetBaseVelocity(o["id"], [0, 0, 0], [0, 0, 0])

    def in_zone(o):
        pos = p.getBasePositionAndOrientation(o["id"])[0]
        return (abs(pos[0] - zone_c[0]) < ZONE_HALF and abs(pos[1] - zone_c[1]) < ZONE_HALF
                and pos[2] < table_top + 0.08)

    panel = None
    if "--sin-panel" not in sys.argv:
        try:
            panel = Panel()
        except Exception as e:
            print("No pude abrir el panel de estado (", e, "). Sigo sin él.")

    filt = np.zeros(3)               # jx, jy, rot suavizados
    prev_ctx = None                  # (brazo, modo) anterior
    prev_grip = None
    pot_engaged = False
    pot_ref = 0.0
    t_print = 0.0
    n_loop = 0
    n_zone = 0

    while p.isConnected():
        t_loop = time.time()
        n_loop += 1

        ev = p.getKeyboardEvents()
        if ord("c") in ev and ev[ord("c")] & p.KEY_WAS_TRIGGERED:
            for o in objs:
                reset_object(o)
        for o in objs:
            if p.getBasePositionAndOrientation(o["id"])[0][2] < table_top - 0.3:
                reset_object(o)                  # el objeto cayó al suelo

        jx, jy, pot, rot, arm, grip, mode = inp.read()
        side = "right" if arm > 0.5 else "left"
        m = 1 if mode > 0.5 else 0
        A = arms[side]
        cfg = MODES[m]

        # Cambio de brazo o de modo: nada se mueve solo, el pote hay que "engancharlo"
        if (side, m) != prev_ctx:
            prev_ctx = (side, m)
            filt[:] = 0.0
            pot_engaged = False
            pot_ref = pot
            print(f">>> Brazo {'DERECHO' if side == 'right' else 'IZQUIERDO'} | MODO {m + 1}: {MODE_NAMES[m]}")
            if cfg["pot"]:
                print(f"    Mueve el potenciómetro para retomar el control de {cfg['pot']}")

        # Pinza: cada cambio en la señal del ESP32 = una pulsación
        if prev_grip is None:
            prev_grip = grip
        if grip != prev_grip:
            prev_grip = grip
            toggle_gripper(body, A, arms, objs)

        # Joystick y botones de giro -> velocidad de articulación
        raw = np.array([deadzone(jx), deadzone(jy), float(np.clip(rot, -1, 1))])
        filt += SMOOTH * (raw - filt)
        if not raw.any():
            filt *= 0.5

        for ch, v, speed in (("jx", filt[0], JOY_SPEED), ("jy", filt[1], JOY_SPEED),
                             ("rot", filt[2], ROT_SPEED)):
            name = cfg[ch]
            if name is not None and abs(v) > 1e-3:
                move_joint(body, A, name, A["q"][name] + SIGN[name] * v * speed * DT)

        # Potenciómetro -> ángulo absoluto (solo en los modos que lo usan)
        if cfg["pot"] is not None:
            name = cfg["pot"]
            if not pot_engaged and abs(pot - pot_ref) > POT_ENGAGE:
                pot_engaged = True
            if pot_engaged:
                goal = pot_to_angle(A, name, pot)
                dq = float(np.clip(goal - A["q"][name], -POT_SPEED * DT, POT_SPEED * DT))
                if abs(dq) > 1e-5:
                    move_joint(body, A, name, A["q"][name] + dq)

        # Objetos dentro de la zona verde (los que se sujetan no cuentan)
        held_ids = {B["held"] for B in arms.values()}
        now_zone = sum(1 for o in objs if o["id"] not in held_ids and in_zone(o))
        if now_zone != n_zone:
            n_zone = now_zone
            print(f"    Objetos en la zona verde: {n_zone}/{len(objs)}" +
                  ("  ¡LOS TRES!" if n_zone == len(objs) else ""))

        # Panel de estado (~15 veces por segundo)
        if panel is not None and panel.alive and n_loop % 4 == 0:
            panel.update(body, side, m, A, cfg, (jx, jy, pot, rot), pot_engaged,
                         names.get(A["held"]), n_zone, len(objs), inp.stale())

        if time.time() - t_print > 1.0:          # diagnóstico: una línea por segundo
            t_print = time.time()
            qs = " ".join(f"{n}={A['q'][n]:+.2f}" for n in JOINT_SUFFIXES)
            print(f"[{side} modo{m + 1}] joy=({jx:+.2f},{jy:+.2f}) pot={pot:.2f} rot={int(rot):+d} "
                  f"pinza={int(A['grip'])} | {qs}")
            if inp.stale():
                print("  ! No llegan datos del ESP32 (revisa puerto, cable y que el Monitor Serie esté cerrado)")

        for _ in range(LOOP_DT_STEPS):
            p.stepSimulation()
        rest = DT - (time.time() - t_loop)
        if rest > 0:
            time.sleep(rest)

    if panel is not None:
        panel.close()


if __name__ == "__main__":
    try:
        main()
    except (p.error, KeyboardInterrupt):
        print("Simulación cerrada.")
    finally:
        if p.isConnected():
            p.disconnect()