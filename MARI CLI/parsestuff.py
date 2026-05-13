import csv
import os
from typing import Optional
from dataclasses import dataclass, asdict
import time
import math

from zmq import Enum

from backend import MAX_RF_SENSITIIVITY

# ------------------ Data Classes ------------------

@dataclass
class RcvFrame:
    address: int
    length: int
    data: str
    rssi: int
    snr: int

@dataclass
class DataFrame:
    type: str # Type of data (e.g. "USER", "IMU", "PID", "MOTOR", "ESC", "BATTERY", "RAW")
    sequence: int # Sequence number for tracking order of messages
    payload: int | float | str | list[int | float] | None # Can be a single number, a list of numbers, or a raw string for unrecognized data types
    uuid: str # Unique identifier for the message (type:sequence)
    
class VoltageLevel(Enum):
    CRITICAL = 0
    WARNING = 1
    NORMAL = 2
    
class PingState(Enum):
    DISCONNECTED = 0
    WAITING = 1
    CONNECTED = 2
    TIMEOUT = 3

@dataclass
class DroneStatus:
    #User inputs
    throttle: int | float = 0.0
    pitch_input: int | float = 0.0
    roll_input: int | float = 0.0
    yaw_input: int | float = 0.0
    
    #Raw sensor values, which may be different from the PID outputs due to noise and filtering
    pitch: int | float = 0.0
    roll: int | float = 0.0
    yaw: int | float = 0.0
    altitude: int | float = 0.0
    temperature: int | float = 0.0  # Temperature in Celsius
    pressure: int | float = 0.0  # Air pressure in hPa/mbar
    
    #PID outputs for each axis, which may be different from the actual motor outputs due to mixing and limits
    pitchP: int | float = 0.0
    pitchI: int | float = 0.0
    pitchD: int | float = 0.0
    pitchError: int | float = 0.00
    
    rollP: int | float = 0.0
    rollI: int | float = 0.0
    rollD: int | float = 0.0
    rollError: int | float = 0.00
    
    yawP: int | float = 0.0
    yawI: int | float = 0.0
    yawD: int | float = 0.0
    yawError: int | float = 0.00
    
    pitchPID: int | float = 0.0
    rollPID: int | float = 0.0
    yawPID: int | float = 0.0
    
    #Delta Time
    dt: float = 0.0000
    
    #Mixed PID outputs for each motor, which may be different from the individual axis PID outputs due to mixing
    motorNW: int | float = 0.0
    motorNE: int | float = 0.0
    motorSW: int | float = 0.0
    motorSE: int | float = 0.0
    
    #Motor output values, which may be different from the PID outputs due to mixing and limits
    motors: list[int | float] = None
    
    #Battery
    battery_status: VoltageLevel = VoltageLevel.NORMAL
    battery_voltage: float = 0.0
    
    #Raw data
    raw_data: str = ""

@dataclass
class RTOS_Status:
    # Task loop rates (Hz)
    flight_rate: int = 0
    tele_tx_rate: int = 0
    tele_rx_rate: int = 0
    housekeeping_rate: int = 0
    
    # CPU usage percentages
    flight_cpu_per: int | float = 0.0
    tele_tx_cpu_per: int | float = 0.0
    tele_rx_cpu_per: int | float = 0.0
    housekeeping_cpu_per: int | float = 0.0
    idle_cpu_per: int | float = 0.0

@dataclass
class PacketStats:
    last_seq: Optional[int] = None
    received_count: int = 0
    missing_count: int = 0
    duplicate_count: int = 0
    reset_count: int = 0

    # Call this method for each received packet with its sequence number to update the stats
    def add(self, seq: int) -> None:
        if self.last_seq is None:
            self.last_seq = seq
            self.received_count += 1
            return

        if seq == self.last_seq:
            self.duplicate_count += 1
            return

        if seq < self.last_seq:
            # likely transmitter reset / reboot / counter reset
            self.reset_count += 1
            self.last_seq = seq
            self.received_count += 1
            return

        gap = seq - self.last_seq
        if gap > 1:
            self.missing_count += gap - 1

        self.last_seq = seq
        self.received_count += 1

    @property
    def expected_count(self) -> int: # Total expected packets based on received and missing counts
        return self.received_count + self.missing_count

    @property
    def pdr(self) -> float: # Packet Delivery Ratio - the percentage of packets received out of the total expected (received + missing)
        if self.expected_count <= 0:
            return 0.0
        return self.received_count / self.expected_count

# ------------------ Parsing Functions ------------------

def parse_number(value: str) -> Optional[int | float]:
    value = value.strip()
    if not value:
        return None

    try:
        if any(ch in value.lower() for ch in (".", "e")):
            return float(value)
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return None


def parse_rcv_line(line: str) -> Optional[RcvFrame]:
    # +RCV=ADDR,LEN,DATA,RSSI,SNR
    # DATA may itself contain commas (RW/MARI free-form payloads), so we use the
    # LEN field to slice the data field exactly instead of comma-splitting it.
    if not line.startswith("+RCV="):
        return None
    try:
        payload = line[len("+RCV="):]

        first_comma = payload.find(",")
        if first_comma < 0:
            return None
        second_comma = payload.find(",", first_comma + 1)
        if second_comma < 0:
            return None

        address = int(payload[:first_comma].strip())
        length = int(payload[first_comma + 1:second_comma].strip())

        data_start = second_comma + 1
        data_end = data_start + length
        if data_end > len(payload):
            return None
        data = payload[data_start:data_end]

        tail = payload[data_end:]
        if not tail.startswith(","):
            return None
        rssi_str, sep, snr_str = tail[1:].rpartition(",")
        if not sep:
            return None
        rssi = int(rssi_str.strip())
        snr = int(snr_str.strip())

        return RcvFrame(address=address, length=length, data=data, rssi=rssi, snr=snr)
    except Exception:
        return None
    
# Example data line: "I:200|10,20,30,100.0" -> DataFrame(type="I", sequence=200, payload=[10,20,30,100.0], uuid="I:200")
# Special cases "Q|..." and "F|..." carry no uuid:sequence prefix.
def parse_data_line(data: str) -> Optional[DataFrame]:

    if "|" not in data:
        return DataFrame(type="RW", sequence=0, payload=data, uuid="RW:0")

    try:
        head, payload = data.split("|", 1)
        head = head.strip()

        # Fine-tuning frames have no uuid/sequence prefix - just "Q|values" or "F|values"
        if head in ("Q", "F"):
            parts = [p.strip() for p in payload.split(",")]
            values = [parse_number(part) for part in parts]
            if any(v is None for v in values):
                return DataFrame(type="RW", sequence=0, payload=payload, uuid="RW:0")
            return DataFrame(type=head, sequence=0, payload=values, uuid=head)

        if ":" not in head:
            return DataFrame(type="RW", sequence=0, payload=data, uuid="RW:0")

        type, seq = head.split(":", 1)
        seq_int = int(seq)

        if type in ("RW", "MARI"):
            return DataFrame(type=type, sequence=seq_int, payload=payload, uuid=head)

        parts = [p.strip() for p in payload.split(",")]
        values = [parse_number(part) for part in parts]
        if any(value is None for value in values):
            return DataFrame(type="RW", sequence=seq_int, payload=payload, uuid=f"RW:{seq_int}")

        return DataFrame(type=type, sequence=seq_int, payload=values, uuid=head)

    except Exception:
        return None
    
'''
Data line types:

- "U": USER - User inputs
- "I": IMU - Raw sensor values
- "P": PID - PID outputs
- "M": MOTOR - Mixed equation outputs for each motor
- "E": ESC - Motor output values
- "B": BATTERY - Battery status
- "RW": RAW - Unrecognized or raw data that doesn't fit other types
- "MARI": MARI - Special type for sunny ping tests, treated as RAW for parsing but can be identified by the "MARI" prefix in the type.
- "C": RTOS CPU usage percentages (flight, tele_tx, tele_rx, housekeeping, idle)
- "R": RTOS task loop rates in Hz (flight, tele_tx, tele_rx, housekeeping)

Format of data for each type:
uuid|payload
uuid format: "msg_type:sequence"
- USER: "U:100|throttle,pitch_input,roll_input,yaw_input"
- IMU: "I:200|pitch,roll,yaw,temperature,pressure"
- PID: "P:300|pitchP,pitchI,pitchD,rollP,rollI,rollD,yawP,yawI,yawD"
- MOTOR: "M:400|motorNW,motorNE,motorSW,motorSE"
- ESC: "E:500|throttleOut,pitchOut,rollOut,yawOut"
- BATTERY: "B:600|Status"
- RAW: "RW:700|data"
- MARI: "MARI:1|payload" (for pinging, treated as RAW)
- C: "C:800|flight_cpu_per,tele_tx_cpu_per,tele_rx_cpu_per,housekeeping_cpu_per,idle_cpu_per"
- R: "R:900|flight_rate,tele_tx_rate,tele_rx_rate,housekeeping_rate"
'''
    
def sort_data_line(frame: DataFrame) -> Optional[DroneStatus]:
    if not frame or not isinstance(frame.payload, (list, str)):
        return None
    
    payload = frame.payload if isinstance(frame.payload, list) else []
    
    match frame.type:
        case "U": #USER
            return DroneStatus(
                throttle=payload[0],
                pitch_input=payload[1],
                roll_input=payload[2],
                yaw_input=payload[3],
            )
        case "I": #IMU
            return DroneStatus(
                pitch=payload[0],
                roll=payload[1],
                yaw=payload[2],
                #temperature=payload[3],
                #pressure=payload[4],
                #altitude=get_altitude_at_pressure(payload[4], payload[3])
            )
        case "P": #PID
            return DroneStatus(
                pitchP=payload[0],
                pitchI=payload[1],
                pitchD=payload[2],
                rollP=payload[3],
                rollI=payload[4],
                rollD=payload[5],
                yawP=payload[6],
                yawI=payload[7],
                yawD=payload[8],
                pitchPID=payload[0] + payload[1] + payload[2],
                rollPID=payload[3] + payload[4] + payload[5],
                yawPID=payload[6] + payload[7] + payload[8]
            )
        case "M": #MOTOR
            return DroneStatus(
                motorNW=payload[0],
                motorNE=payload[1],
                motorSW=payload[2],
                motorSE=payload[3],
            )
        case "E": #ESC
            return DroneStatus(
                motors=payload[:4],
            )
        case "B": #BATTERY
            return DroneStatus(
                battery_status = VoltageLevel(int(payload[0])) if payload else VoltageLevel.NORMAL    
            )
        case "RW" | "MARI": #RAW or MARI ping data
            return DroneStatus(
                raw_data=str(frame.payload)
            )
        case "F": #FINE-TUNING: imu xyz, errors xyz, motors[4], dt
            if len(payload) < 11:
                return None
            return DroneStatus(
                pitch=payload[0],
                roll=payload[1],
                yaw=payload[2],
                pitchError=payload[3],
                rollError=payload[4],
                yawError=payload[5],
                motorNW=payload[6],
                motorNE=payload[7],
                motorSW=payload[8],
                motorSE=payload[9],
                dt=payload[10],
            )
        case "Q": #FINE-TUNING: pitch/roll/yaw USER|I|D + battery_voltage
            if len(payload) < 10:
                return None
            return DroneStatus(
                pitch_input=payload[0],
                pitchI=payload[1],
                pitchD=payload[2],
                roll_input=payload[3],
                rollI=payload[4],
                rollD=payload[5],
                yaw_input=payload[6],
                yawI=payload[7],
                yawD=payload[8],
                battery_voltage=payload[9],
            )

        case _:
            return None
        
        
def update_rtos_status(frame: DataFrame, status: RTOS_Status) -> Optional[RTOS_Status]:
    if not frame or not isinstance(frame.payload, list):
        return None

    payload = frame.payload

    match frame.type:
        case "C": #CPU usage percentages
            if len(payload) < 5:
                return None
            status.flight_cpu_per       = payload[0]
            status.tele_tx_cpu_per      = payload[1]
            status.tele_rx_cpu_per      = payload[2]
            status.housekeeping_cpu_per = payload[3]
            status.idle_cpu_per         = payload[4]
            return status
        case "R": #Task loop rates (Hz)
            if len(payload) < 4:
                return None
            status.flight_rate       = int(payload[0])
            status.tele_tx_rate      = int(payload[1])
            status.tele_rx_rate      = int(payload[2])
            status.housekeeping_rate = int(payload[3])
            return status
        case _:
            return None

#------------------ Utility Functions ------------------

def get_altitude_at_pressure(pressure: float, #hPa/mbar
                             temp: float, #Celsius
                             sea_level_pressure_hpa: float = 1013.25) -> float: 
    if pressure > sea_level_pressure_hpa: # Higher than sea level, which is unlikely and would require a different formula. Return -404.0 for simplicity.
        return -404.0
    if pressure <= 0:
        return -404.0
    if temp < -273.15:
        return -404.0
    if sea_level_pressure_hpa <= 0:
        return -404.0
    
    temp_k = temp + 273.15
    R = 287.05   # J/(kg*K), specific gas constant for dry air
    g = 9.80665  # m/s^2

    return (R * temp_k / g) * math.log(sea_level_pressure_hpa / pressure)

def now_ts() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")

def get_rtt(start_time: float, end_time: float) -> float:
    return (end_time - start_time) * 1000.0

def rssi_okay(rssi: int) -> str:
    rssi_margin = rssi - MAX_RF_SENSITIIVITY
    if rssi_margin >= 20:
        return "Strong"
    elif rssi_margin >= 10:
        return "Fair"
    elif rssi_margin >= 5:
        return "Weak"
    else:
        return "Poor"

def snr_okay(snr: int) -> str:
    if snr >= 10:
        return "Strong"
    elif snr >= 5:
        return "Good"
    elif snr >= 0:
        return "Fair"
    elif snr >= -5:
        return "Weak"
    else:
        return "Poor"

# ------------------- CSV ------------------
def save_status_to_csv(filename: str, msg_type: str, sequence: int, status: DroneStatus, packet: PacketStats) -> None:
    row = asdict(status)


    if not filename:
        filename = f"drone_status_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    if not filename.endswith(".csv"):
        filename += ".csv"


    # make enum easier to save
    if row.get("battery_status") is not None:
        try:
            row["battery_status"] = row["battery_status"].name
        except Exception:
            row["battery_status"] = str(row["battery_status"])

    # Convert motors list to a CSV-safe string
    if row.get("motors") is not None:
        try:
            row["motors"] = "|".join(str(m) for m in row["motors"])
        except Exception:
            row["motors"] = ""
    else:
        row["motors"] = ""

    date,time_str = now_ts().split(" ")
    row["timestamp"] = f"{time_str}"
    row["uuid"] = f"{msg_type}:{sequence}"
    row["pdr"] = packet.pdr if packet else 0.0
    row["raw_data"] = status.raw_data if status.raw_data else ""

    fieldnames = [
        "timestamp", "uuid", "pdr", "raw_data",
        "throttle", "pitch_input", "roll_input", "yaw_input",
        "pitch", "roll", "yaw", "altitude", "temperature", "pressure",
        "pitchP", "pitchI", "pitchD",
        "rollP", "rollI", "rollD",
        "yawP", "yawI", "yawD",
        "pitchPID", "rollPID", "yawPID",
        "motorNW", "motorNE", "motorSW", "motorSE",
        "motors", "battery_status"
    ]

    file_is_empty = (not os.path.exists(filename)) or os.path.getsize(filename) == 0

    with open(filename, "a", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if file_is_empty:
            writer.writeheader()
        writer.writerow(row)

