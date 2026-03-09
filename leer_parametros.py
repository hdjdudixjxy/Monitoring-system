import time 
import csv
from dataclasses import dataclass
from pymavlink import mavutil
import matplotlib.pyplot as plt
import math

#udp:172.22.96.1:14550 WINDOWS
#udp:127.0.0.1:14550 WSL

PORT = "udp:127.0.0.1:14550"        # dirección de conexión a SITL (ArduPilot)

# PARÁMETROS DE LA MISIÓN
TAKEOFF_ALT_M = 1.8                 # altura de despegue
HOLD_TIME_S = 480                    # tiempo para medir consumo 60*8minutos = 480 segundos

LOG_CSV = "consumo_mision.csv"      # archivo de salida para datos de consumo y altitud

# Estructura para guardar muestras de consumo y altitud
@dataclass
class Sample:
    t: float
    V: float
    mode: str
    rel_alt: float
    x_m: float
    y_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    PWM1: int
    PWM2: int
    PWM3: int
    PWM4: int

# Resto de funciones para consumo, integración, gráficos, etc
def request_streams(master, rate_hz=100):
    """Pide a ArduPilot que envíe datos a una tasa."""
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1
    )

def get_status(master, timeout=1.0):
    """Lee lo necesario para consumo + altitud + modo."""
    V = None
    mode = None
    rel_alt = None
    x_m = y_m = None
    roll_deg = pitch_deg = yaw_deg = None
    pwm1 = pwm2 = pwm3 = pwm4 = None
    
    t0 = time.time()

    while time.time() - t0 < timeout:
        msg = master.recv_match(blocking=True, timeout=timeout)
        
        if msg is None:
            continue
    
        myType = msg.get_type()

        if myType == "SYS_STATUS":
            V = msg.voltage_battery / 1000.0      # mV -> V
            if V <= 0:
                return None
        
        elif myType == "GLOBAL_POSITION_INT":
            rel_alt = msg.relative_alt / 1000.0   # mm -> m
        
        elif myType == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
        
        elif myType == "LOCAL_POSITION_NED":
            x_m = msg.x / 1000.0                       # mm -> m
            y_m = msg.y / 1000.0                       # mm -> m
        
        elif myType == "ATTITUDE":
            roll_deg = math.degrees(msg.roll)
            pitch_deg = math.degrees(msg.pitch)
            yaw_deg = math.degrees(msg.yaw)
            
        elif myType == "SERVO_OUTPUT_RAW":
            pwm1 = msg.servo1_raw
            pwm2 = msg.servo2_raw
            pwm3 = msg.servo3_raw
            pwm4 = msg.servo4_raw
            
        if all(v is not None for v in [
            V, mode, rel_alt, x_m, y_m,
            roll_deg, pitch_deg, yaw_deg,
            pwm1, pwm2, pwm3, pwm4]):
            return V, mode, rel_alt, x_m, y_m, roll_deg, pitch_deg, yaw_deg, [pwm1, pwm2, pwm3, pwm4]
            
    return None

def get_param(master, name: str, timeout=3.0):
    """Lee un parámetro de ArduPilot por su nombre."""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        -1
    )

    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    return None

def set_param(master, name: str, value: float, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32, timeout=3.0):
    """Escribe un parámetro en ArduPilot por su nombre y espera confirmación."""
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
   
    request_streams(master, rate_hz=100)
    
    # Registrar consumo durante HOLD_TIME_S
    samples = []
    start = time.time()
    print(f"Registrando consumo por {HOLD_TIME_S} s...")

    while True:
        t = time.time() - start
        if t > HOLD_TIME_S:
            break
        
        st = get_status(master, timeout=0.5)
        if not st:
            continue
        V, mode, rel_alt, x_m, y_m, roll_deg, pitch_deg, yaw_deg, PWM = st
        samples.append(Sample(t=t, V=V, mode=mode, rel_alt=rel_alt, x_m=x_m, y_m=y_m, roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg, PWM1=PWM[0], PWM2=PWM[1], PWM3=PWM[2], PWM4=PWM[3]))
        
    # Guardar CSV
    with open(LOG_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "V_V", "mode", "rel_alt_m", "x_m", "y_m", "roll_deg", "pitch_deg", "yaw_deg", "PWM1", "PWM2", "PWM3", "PWM4"])
        for s in samples:
            w.writerow([s.t, s.V, s.mode, s.rel_alt, s.x_m, s.y_m, s.roll_deg, s.pitch_deg, s.yaw_deg, s.PWM1, s.PWM2, s.PWM3, s.PWM4])

    print(f"✅ CSV guardado: {LOG_CSV}")


    # Gráficas
    ts = [s.t for s in samples]
    Vs = [s.V for s in samples]
    alts = [s.rel_alt for s in samples]
    x_ms = [s.x_m for s in samples]
    y_ms = [s.y_m for s in samples]
    roll_degs = [s.roll_deg for s in samples]
    pitch_degs = [s.pitch_deg for s in samples]
    yaw_degs = [s.yaw_deg for s in samples]
    pwm1s = [s.PWM1 for s in samples]
    pwm2s = [s.PWM2 for s in samples]
    pwm3s = [s.PWM3 for s in samples]
    pwm4s = [s.PWM4 for s in samples]

    fig, axs = plt.subplots(2, 3, figsize=(12, 8))

    # Voltaje
    axs[0,0].plot(ts, Vs)
    axs[0,0].set_title("Voltaje vs tiempo")
    axs[0,0].set_xlabel("Tiempo (s)")
    axs[0,0].set_ylabel("Voltaje (V)")

    # Altitud
    axs[0,1].plot(ts, alts)
    axs[0,1].set_title("Altitud vs tiempo")
    axs[0,1].set_xlabel("Tiempo (s)")
    axs[0,1].set_ylabel("Altitud (m)")

    # Posición
    axs[0,2].plot(ts, x_ms, label="x")
    axs[0,2].plot(ts, y_ms, label="y")
    axs[0,2].set_title("Posición vs tiempo")
    axs[0,2].set_xlabel("Tiempo (s)")
    axs[0,2].set_ylabel("Posición (m)")
    axs[0,2].legend()

    # Ángulos
    axs[1,0].plot(ts, roll_degs, label="roll")
    axs[1,0].plot(ts, pitch_degs, label="pitch")
    axs[1,0].plot(ts, yaw_degs, label="yaw")
    axs[1,0].set_title("Ángulos vs tiempo")
    axs[1,0].set_xlabel("Tiempo (s)")
    axs[1,0].set_ylabel("Ángulo (°)")
    axs[1,0].legend()

    # PWM
    axs[1,1].plot(ts, pwm1s, label="motor1")
    axs[1,1].plot(ts, pwm2s, label="motor2")
    axs[1,1].plot(ts, pwm3s, label="motor3")
    axs[1,1].plot(ts, pwm4s, label="motor4")
    axs[1,1].set_title("PWM vs tiempo")
    axs[1,1].set_xlabel("Tiempo (s)")
    axs[1,1].set_ylabel("PWM")
    axs[1,1].legend()

    fig.delaxes(axs[1,2])

    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    main()
