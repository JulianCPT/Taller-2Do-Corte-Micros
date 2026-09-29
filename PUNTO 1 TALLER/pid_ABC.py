"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python pid_ABC.py

Notes
-----
4 drones take off together, forming a square (swarm) formation.
There are 4 ground points: A, B, C, D (B, C, D form a triangle).
The swarm starts landed at A. A physical 4x4 matrix keypad wired to
an ESP32 sends the letter 'A', 'B', 'C' or 'D' over the serial port
whenever that key is pressed. When the swarm receives a letter (and
is currently landed), it takes off, flies straight to that point,
and lands there -- free navigation, in any order. Pressing '*'
resets the simulation (swarm back to A). Pressing '#' ends the
simulation (saves results and closes).

Requires the ESP32 running teclado_matricial_ESP32.ino, and the
pyserial package on the PC:

    $ python -m pip install pyserial

"""
import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

try:
    import serial
except ImportError:
    serial = None

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 4
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_MAX_DURATION_SEC = 300   # limite de seguridad (la mision normalmente termina antes)
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
DEFAULT_SERIAL_PORT = 'COM3'     # puerto donde esta conectado el ESP32
DEFAULT_BAUD_RATE = 115200       # debe coincidir con Serial.begin() en el ESP32

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        max_duration_sec=DEFAULT_MAX_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB,
        serial_port=DEFAULT_SERIAL_PORT,
        baud_rate=DEFAULT_BAUD_RATE
        ):
    #### Initialize the simulation #############################
    H = .02   # altura de despegue/aterrizaje: casi 0, tocando el piso
    Z_CRUISE = 1.0   # altura de vuelo (crucero) entre despegue y aterrizaje

    #### Puntos en el PISO, alineados a la cuadricula (baldosas) #
    #### TILE = tamano de una baldosa en metros. Ajustalo si en  #
    #### tu simulador las lineas del piso no miden 1m exacto.    #
    TILE = 1.0
    A = np.array([0*TILE, 0*TILE, H])   # punto de partida
    B = np.array([3*TILE, 0*TILE, H])
    C = np.array([4*TILE, 3*TILE, H])
    D = np.array([1*TILE, 3*TILE, H])
    #### Diccionario: letra del teclado -> punto en el piso #######
    POINTS_DICT = {'A': A, 'B': B, 'C': C, 'D': D}
    BASE_POINTS = [A, B, C, D]   # donde van las cajitas blancas

    #### Formacion en cuadrado (enjambre) ########################
    #### E = separacion entre drones dentro del cuadrado (metros)
    E = .18
    SWARM_OFFSETS = np.array([
        [-E/2, -E/2, 0],   # dron 0: atras-izquierda
        [ E/2, -E/2, 0],   # dron 1: atras-derecha
        [-E/2,  E/2, 0],   # dron 2: adelante-izquierda
        [ E/2,  E/2, 0],   # dron 3: adelante-derecha
    ])
    # Si en algun momento usas mas de 4 drones, repite offsets en ciclo
    SWARM_OFFSETS = np.array([SWARM_OFFSETS[i % len(SWARM_OFFSETS)] for i in range(num_drones)])

    #### Todos arrancan a la misma altura (piso), formando el cuadrado, en A
    INIT_XYZS = np.array([A + SWARM_OFFSETS[i] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0, 0] for i in range(num_drones)])

    #### Duracion de cada vuelo punto-a-punto (despega-avanza-aterriza)
    LEG_DURATION_SEC = 4
    N_PHASE = int(control_freq_hz * LEG_DURATION_SEC / 3)

    def vertical(z_from, z_to, xy, n):
        """Sube o baja en el mismo punto (x, y), cambiando solo z."""
        seg = np.zeros((n, 3))
        for i in range(n):
            t = i / (n - 1) if n > 1 else 1
            seg[i, :] = xy[0], xy[1], z_from + (z_to - z_from) * t
        return seg

    def horizontal(p_from, p_to, z, n):
        """Avanza en linea recta manteniendo la altura z constante."""
        seg = np.zeros((n, 3))
        for i in range(n):
            t = i / (n - 1) if n > 1 else 1
            seg[i, :] = p_from[0] + (p_to[0]-p_from[0])*t, p_from[1] + (p_to[1]-p_from[1])*t, z
        return seg

    def build_leg(p_from, p_to):
        """Construye la trayectoria (despega-avanza-aterriza) entre 2 puntos cualquiera."""
        return np.vstack([
            vertical(p_from[2], Z_CRUISE, p_from, N_PHASE),
            horizontal(p_from, p_to, Z_CRUISE, N_PHASE),
            vertical(Z_CRUISE, p_to[2], p_to, N_PHASE),
        ])

    #### Create the environment ################################
    env = CtrlAviary(drone_model=drone,
                        num_drones=num_drones,
                        initial_xyzs=INIT_XYZS,
                        initial_rpys=INIT_RPYS,
                        physics=physics,
                        neighbourhood_radius=10,
                        pyb_freq=simulation_freq_hz,
                        ctrl_freq=control_freq_hz,
                        gui=gui,
                        record=record_video,
                        obstacles=obstacles,
                        user_debug_gui=user_debug_gui
                        )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Dibujar una "cajita blanca" de base en cada punto ########
    #### Muy delgada y pegada al piso real (no elevada), para que
    #### no choque visualmente con las patas del dron al aterrizar
    def draw_bases():
        marker_visual = p.createVisualShape(p.GEOM_BOX,
                                             halfExtents=[.3, .3, .002],
                                             rgbaColor=[1, 1, 1, 1],
                                             physicsClientId=PYB_CLIENT)
        for base_point in BASE_POINTS:
            p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=-1,   # sin colision: el dron no choca con ella
                               baseVisualShapeIndex=marker_visual,
                               basePosition=[base_point[0], base_point[1], .002],
                               physicsClientId=PYB_CLIENT)

    draw_bases()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    #### Abrir el puerto serial hacia el ESP32 (teclado matricial)
    ser = None
    if serial is None:
        print("[AVISO] El paquete 'pyserial' no esta instalado. Corre:")
        print("        python -m pip install pyserial")
        print("        Mientras tanto, puedes probar con el teclado del PC")
        print("        (teclas A, B, C, D) dentro de la ventana de PyBullet.")
    else:
        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=0)
            print(f"Puerto serial {serial_port} abierto a {baud_rate} baudios.")
            print("Esperando a que el ESP32 termine de reiniciar...")
            #### El ESP32 se reinicia solo al abrir el puerto serial y
            #### imprime mensajes de arranque (bootloader). Esperamos y
            #### descartamos ese ruido para que no se confunda con un
            #### comando real (A/B/C/D/#/*).
            time.sleep(2.5)
            ser.reset_input_buffer()
            print("Listo. El teclado matricial ya esta activo.")
        except Exception as e:
            print(f"[AVISO] No se pudo abrir el puerto serial {serial_port}: {e}")
            print("        Verifica el puerto en el Administrador de dispositivos")
            print("        y que el ESP32 este conectado. Mientras tanto, puedes")
            print("        probar con el teclado del PC (A, B, C, D) en PyBullet.")

    print("======================================================")
    print(" Presiona A, B, C o D en el teclado matricial (ESP32)  ")
    print(" para que el enjambre despegue y vuele a ese punto.    ")
    print(" Presiona '*' para REINICIAR la simulacion (el enjambre")
    print(" vuelve a A y borra el progreso). Presiona '#' para    ")
    print(" CERRAR la simulacion (guarda resultados y cierra).    ")
    print(" (Si no hay hardware conectado, tambien puedes usar    ")
    print(" las teclas A/B/C/D/#/* del PC dentro de la ventana de ")
    print(" PyBullet, como respaldo para probar.)                 ")
    print("======================================================")

    #### Run the simulation, navegacion libre A/B/C/D ##########
    action = np.zeros((num_drones,4))
    START = time.time()

    current_key = 'A'                    # donde esta parado el enjambre ahora
    current_point = POINTS_DICT[current_key]
    pending_key = None                   # a donde va volando actualmente
    leg_traj = None
    step_in_leg = 0
    flying = False                       # False = aterrizado; True = en vuelo
    has_departed = False                 # ya salio de A al menos una vez?
    mission_done = False
    serial_buffer = ""

    i = 0
    MAX_STEPS = int(max_duration_sec * control_freq_hz)

    while (not mission_done) and (i < MAX_STEPS):

        #### Leer el puerto serial (no bloqueante) ##################
        requested_key = None
        requested_control = None
        if ser is not None and ser.in_waiting > 0:
            try:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                serial_buffer += data
            except Exception:
                pass
        #### Solo se acepta un comando si llego como LINEA COMPLETA y
        #### es EXACTAMENTE un caracter valido (A/B/C/D/#/*). Esto evita
        #### que ruido o texto de arranque del ESP32 se confunda con un
        #### comando real.
        if '\n' in serial_buffer:
            lineas = serial_buffer.split('\n')
            serial_buffer = lineas[-1]   # guarda el fragmento incompleto para la siguiente vuelta
            for linea in lineas[:-1]:
                token = linea.strip().upper()
                if len(token) == 1:
                    if token in POINTS_DICT:
                        requested_key = token
                    elif token in ('#', '*'):
                        requested_control = token

        #### Respaldo: teclado del PC en la ventana de PyBullet #####
        if gui:
            keys_pressed = p.getKeyboardEvents(physicsClientId=PYB_CLIENT)
            if requested_key is None:
                for letra in POINTS_DICT.keys():
                    code = ord(letra.lower())
                    if code in keys_pressed and (keys_pressed[code] & p.KEY_WAS_TRIGGERED):
                        requested_key = letra
            if requested_control is None:
                for simbolo in ('#', '*'):
                    code = ord(simbolo)
                    if code in keys_pressed and (keys_pressed[code] & p.KEY_WAS_TRIGGERED):
                        requested_control = simbolo

        #### '#' cierra la simulacion; '*' la reinicia ##############
        if requested_control == '#':
            print("Tecla '#': cerrando la simulacion...")
            mission_done = True
            continue
        elif requested_control == '*':
            print("Tecla '*': reiniciando la simulacion...")
            obs, info = env.reset()
            for j in range(num_drones):
                if hasattr(ctrl[j], 'reset'):
                    ctrl[j].reset()
            draw_bases()   # env.reset() borra el mundo fisico; hay que redibujar las cajitas
            action = np.zeros((num_drones,4))
            current_key = 'A'
            current_point = POINTS_DICT[current_key]
            pending_key = None
            leg_traj = None
            step_in_leg = 0
            flying = False
            has_departed = False
            continue

        #### Si esta aterrizado y llego una letra valida, despega ###
        if (not flying) and (requested_key is not None):
            if requested_key == current_key:
                print(f"Ya estoy en el punto {requested_key}.")
            else:
                pending_key = requested_key
                leg_traj = build_leg(current_point, POINTS_DICT[pending_key])
                step_in_leg = 0
                flying = True
                has_departed = True
                print(f"Despegando de {current_key} hacia {pending_key}...")

        #### Posicion objetivo actual (base, sin el offset de formacion)
        if flying:
            base_wp = leg_traj[min(step_in_leg, len(leg_traj) - 1)]
            step_in_leg += 1
            if step_in_leg >= len(leg_traj):
                #### Tramo terminado: aterrizo en el punto solicitado ###
                flying = False
                current_key = pending_key
                current_point = POINTS_DICT[current_key]
                print(f"Aterrizado en {current_key}.")
        else:
            #### Aterrizado, esperando la siguiente letra ################
            base_wp = current_point

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):
            target = base_wp + SWARM_OFFSETS[j]
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=target,
                                                                    target_rpy=INIT_RPYS[j, :]
                                                                    )

        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([base_wp + SWARM_OFFSETS[j], INIT_RPYS[j, :], np.zeros(6)])
                       )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

        i += 1

    #### Close the environment #################################
    if ser is not None:
        ser.close()
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid_ABC") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Free A/B/C/D swarm navigation via a matrix keypad (ESP32, serial), using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 4)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,      help='Whether to add PyBullet\'s default obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--max_duration_sec',   default=DEFAULT_MAX_DURATION_SEC,         type=int,           help='Safety cap on simulation length in seconds (default: 300)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    parser.add_argument('--serial_port',        default=DEFAULT_SERIAL_PORT, type=str,           help='Serial port where the ESP32 is connected (default: "COM3")', metavar='')
    parser.add_argument('--baud_rate',          default=DEFAULT_BAUD_RATE, type=int,           help='Serial baud rate, must match Serial.begin() on the ESP32 (default: 115200)', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))