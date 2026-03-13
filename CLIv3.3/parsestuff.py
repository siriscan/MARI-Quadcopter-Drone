from typing import Optional
from dataclasses import dataclass
import time

@dataclass
class RcvFrame:
    address: int
    length: int
    data: str
    rssi: int
    snr: int


@dataclass
class DroneStatus:
    pitch: int | float
    roll: int | float
    yaw: int | float
    altitude: int | float

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
    if not line.startswith("+RCV="):
        return None
    try:
        payload = line[len("+RCV="):].strip()
        parts = [p.strip() for p in payload.split(",")]
        if len(parts) < 5:
            return None

        address = int(parts[0])
        length = int(parts[1])
        rssi = int(parts[-2])
        snr = int(parts[-1])
        data_parts = parts[2:-2]
        data = ",".join(data_parts).strip()

        return RcvFrame(address=address, length=length, data=data, rssi=rssi, snr=snr)
    except Exception:
        return None
    

def parse_data_line(data: str) -> Optional[DroneStatus]:
# Example line: "10,20,30,100.0"

    try:
        parts = [p.strip() for p in data.split(",")]
        if len(parts) != 4:
            return None
        values = [parse_number(part) for part in parts]
        if any(value is None for value in values):
            return None
        pitch, roll, yaw, altitude = values
        
        return DroneStatus(pitch=pitch, roll=roll, yaw=yaw, altitude=altitude)

    except Exception:
        return None
    
    
def now_ts() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")