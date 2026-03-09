import time 
from pymavlink import mavutil
import os
import cv2
import numpy as np
import threading

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

#udp:172.22.96.1:14550 WINDOWS
#udp:127.0.0.1:14550 WSL

PORT = "127.0.0.1:14550"        # dirección de conexión a SITL (ArduPilot)

# PARÁMETROS DE LA MISIÓN
TAKEOFF_ALT_M = 1.8                 # altura de despegue 1.8

# PARÁMETROS DE GRID
GRID_SIZE = 20
CELL_SIZE = 5  # metros

TOPIC = "/world/crop/model/uav_with_gimbal/model/camera/link/camera_link/sensor/camera/image"
SAVE_DIR = "capturas_grid"

latest_frame = None
frame_lock = threading.Lock()


# CAMERA CALLBACK
def image_callback(msg: Image):
    global latest_frame

    width = msg.width
    height = msg.height
    raw = msg.data

    if width == 0 or height == 0 or len(raw) == 0:
        return

    expected_size = width * height * 3
    if len(raw) < expected_size:
        return

    img = np.frombuffer(raw[:expected_size], dtype=np.uint8)
    img = img.reshape((height, width, 3))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    with frame_lock:
        latest_frame = img

def wait_for_first_frame(timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        with frame_lock:
            if latest_frame is not None:
                return True
        time.sleep(0.05)
    return False

def save_latest_image(filename):
    with frame_lock:
        if latest_frame is None:
            print("No hay frame disponible todavía.")
            return False
        img = latest_frame.copy()

    ok = cv2.imwrite(filename, img)
    if ok:
        print(f"Imagen guardada: {filename}")
    else:
        print(f"No se pudo guardar: {filename}")
    return ok

# Funciones auxiliares para MAVLink
def wait_heartbeat(master, timeout=30):
    """Espera un mensaje de heartbeat"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            return True
    return False

# Funciones para misión
def generate_grid_path():
    """Genera waypoints para un grid de 20x20 con celdas de 5m, en patrón serpenteante"""
    waypoints = []

    for row in range(GRID_SIZE):
        if row % 2 == 0:
            cols = range(GRID_SIZE)
        else:
            cols = reversed(range(GRID_SIZE))

        for col in cols:
            x = col * CELL_SIZE
            y = row * CELL_SIZE
            waypoints.append((x, y))

    return waypoints

# Funciones para control de vuelo
def set_mode(master, mode_str: str):
    """Cambia modo a GUIDED/AUTO/LOITER"""
    mode_map = master.mode_mapping()
    if mode_str not in mode_map:
        raise ValueError(f"Modo {mode_str} no disponible. Disponibles: {list(mode_map.keys())}")
    mode_id = mode_map[mode_str]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(1.5)

# Funciones para misión
def arm(master):
    """Arma los motores"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    time.sleep(1.5)

# Funciones para misión
def takeoff(master, alt_m: float):
    """Despegue en GUIDED usando"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt_m
    )
    time.sleep(5)

# Funciones para misión
def send_position_local_ned(master,x, y, z, yaw=0.0):
    """Envía posición objetivo en LOCAL_NED (z debe ir negativo para subir)"""
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,      # posición
        0, 0, 0,      # velocidad ignorada
        0, 0, 0,      # aceleración ignorada
        yaw,
        0
    )
    time.sleep(0.5)

# Resto de funciones para consumo, integración, gráficos, etc
def request_streams(master, rate_hz=100):
    """Pide a ArduPilot que envíe datos a una tasa"""
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1
    )

def set_param(master, name: str, value: float, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32, timeout=3.0):
    """Establece un parámetro en ArduPilot"""
    master.mav.param_set_send( 
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        float(value),
        param_type
    )

    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            print(f"{name} = {msg.param_value}")
            return msg.param_value

    print(f"No confirmé {name}, pero el comando fue enviado.")
    return None


def main():
    master = mavutil.mavlink_connection(PORT)
    ok = wait_heartbeat(master)
    
    if not ok:
        raise RuntimeError("No recibí HEARTBEAT. Revisa que SITL esté corriendo y emitiendo en 14550.")

    print("Conectado a SITL (heartbeat OK).")
    #request_streams(master, rate_hz=100)
    
    # Configurar parámetros de batería para simulación
    set_param(master, "BATT_MONITOR", 4)
    set_param(master, "SIM_BATT_CAP_AH", 6)
    set_param(master, "SIM_BATT_VOLTAGE", 25.2)
    set_param(master, "BATT_AMP_PERVLT", 17)
    
    set_mode(master, "GUIDED")
    arm(master)
    takeoff(master, TAKEOFF_ALT_M)
    time.sleep(8)
    
    # realizar grid
    waypoints = generate_grid_path()
    os.makedirs(SAVE_DIR, exist_ok=True)
    camera_node = Node()
    ok = camera_node.subscribe(Image, TOPIC, image_callback)
    
    if not ok:
        raise RuntimeError(f"No se pudo suscribir al topic: {TOPIC}")

    print(f"Suscrito a: {TOPIC}")   

    for i, wp in enumerate(waypoints, start=1):
        x, y = wp

        print(f"Moviendo a {x},{y}")
        send_position_local_ned(master,47.5-x, 7.5+y, -TAKEOFF_ALT_M, yaw=0.0) #incluye el desfase del cultivo para empezar correctamente el grid
        time.sleep(8.0)  # espera entre comandos para no saturar 
        
        filename = os.path.join(SAVE_DIR, f"wp_{i:03d}_x_{x:.2f}_y_{y:.2f}.png")
        save_latest_image(filename)
        
    # Desarmado al culminar
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

if __name__ == "__main__":
    main()
