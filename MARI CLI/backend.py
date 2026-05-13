from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import serial
from serial.tools import list_ports


from enum import Enum

import parsestuff

CRLF = b"\r\n"

art = r"""
⢼⣿⣿⢏⢐⠖⣐⡂⠑⠉⠉⠭⠽⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⢋⠉⠀⠀⠁⠙⢷⢿⣿⣿⣿⣿⣿⣮⡻⣿⣿⣿⣿⣯⣿⣿⡿
⣿⣿⡯⠫⢕⠾⣻⡷⠕⡂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⠀⠀⠁⢴⣶⣩⣝⡻⢿⣷⣝⢿⣿⣿⣮⣽⣿⣿
⢾⣿⡟⠿⣿⡷⢠⣦⢺⡦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢫⣾⣿⣿⣷⣈⣛⢧⡻⣿⣿⣿⣿⣿
⣹⣿⣿⣿⣖⠂⣈⢥⣬⠁⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣿⣿⣿⣿⣿⣿⣿⡘⢿⣿⣿⣿
⣿⣿⣿⢟⡵⠀⠌⠃⠉⠁⠀⠀⠐⠁⠀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠋⡙⠻⠛⠈⠀⠊⠀⠻⣿⣿
⣿⣟⣵⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣿⠀⠀⠀⠀⢀⣴⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⢀⢴⣦⡄⢀⠀⡈⠻
⣿⣿⡿⠁⠀⡠⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⠾⠿⣛⢿⠀⠀⠀⣠⣾⣿⣿⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡔⣱⡏⠎⠀⠀⠀⠀⠀
⣿⣿⠡⣂⣤⠂⠙⠂⠀⠀⠀⢰⣛⠣⢀⢉⣠⣤⣤⣌⣩⣤⣶⣿⣿⠗⠒⠛⠻⢷⡄⠀⠀⠀⠀⠀⠀⠀⠀⢰⠫⡪⣉⣒⡂⠐⡀⠀⠀⠈
⣿⣡⢞⣫⠂⠀⠀⠀⠀⠀⠀⢿⣿⢂⡘⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣤⣈⢠⣴⢦⠀⠀⠀⠀⠀⠀⠊⡀⡀⠀⠀⣠⣄⣄⢻⣦
⣽⣷⣿⡏⠀⠀⠀⠀⠀⠀⠀⠈⠙⠿⡇⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣈⢰⣷⠇⠀⠀⠀⠀⠀⠈⠁⠺⠷⡐⠛⠛⠈⠀⢿
⣿⠿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣿⣿⣯⣛⡿⠿⠿⢟⣿⣿⣿⣿⡟⣰⡿⠾⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠛⠊⠂⢀⣠⣤
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡶⢿⣿⣿
⢄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⠿⣿⣿⣿⠿⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⢦⠀⠀⠀⠀⠀⠀⠀⠀⠈⠺⣿⣿⡇
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠀
⠁⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡇⢰⣿⣿⣿⣿⡇⢉⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣿⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠀⠀⠀⠀⠀⠀⣿⣿⡈⢿⣿⣿⡿⣡⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠁⠀⠀⠀⠀⠀⢀⣴⣾⢻⣿⡇⠀⠀⠀⠀⠀⣿⣿⣿⣮⠻⢋⣼⣿⣿⣿⠇⠀⠀⠀⠀⠀⢰⣶⣶⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⣠⣿⣿⠿⣧⣻⠀⠀⠀⠀⠀⠀⢻⣿⠟⢉⣤⠤⠙⢿⣿⡟⠀⠀⠀⠀⠀⠀⠘⣿⣹⣿⣿⣆⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⣴⣿⣿⣿⣿⣮⠃⠀⠀⠀⠀⠀⠀⠘⠁⠀⡈⠁⠀⠀⠀⠙⠀⠀⠀⠀⠀⠀⠀⠀⢣⣿⣿⣿⣿⣆⠀⠀⠀⠀⠘⡦⠀⠀⠀⠀
⠀⠀⢀⣾⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⡰⠁⢀⠀⡀⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣦⠀⠀⠀⠀⠀⠀⠀⠀⠘
⢀⣶⣿⣿⣿⣿⣿⣿⣿⣿⠁⠀⠀⠀⠀⠀⠀⠀⣼⠇⣾⡟⢸⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣧⡀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⠿⠿⠿⠿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⢸⣿⣴⣿⠃⠀⢻⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠿⠿⠿⣿⣿⣷⡀⠀⠀⠀⠀⠀⠀
⢯⣀⣤⣄⡀⣼⣷⣶⣬⡻⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠏⠀⠀⠸⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⠸⣫⣶⣾⣿⡆⠀⠈⠉⢳⠀⠀⠀⠀⠀⠀
⠀⠙⠿⣿⢣⣿⣿⣿⣿⣷⠀⠀⠀⠀⠀⠀⠀⣿⡿⠃⠀⠀⠀⠀⠘⢿⣿⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⡇⣴⣶⠞⠁⠀⢀⠀⠀⠀⠀
⠀⠀⠀⠈⣼⣿⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀⠉⠀⠀⠀⠀⠀⠀⠀⠈⠙⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣷⠉⠀⠀⠀⠀⠀⠁⠀⠀⠀
Credit: Omori is made by OMOCAT, LLC
"""

title = "MARI LoRa CLI GUI v5 - by Seth Iris Canonigo"

WATCHDOG_TIMEOUT = 1800 # 30 minutes in seconds
FLUSH_INTERVAL = 50 # flush every 50 lines to keep log file up to date without flushing on every line
DISPLAY_MAX_LINES = 100 # max lines to keep in the display log (older lines will be removed)
MAX_RF_SENSITIIVITY = -129 # max sensitivity in dBm for LoRa (RYLR896) - used for RSSI quality check
SUNNY_PING_ACK = "SUN:%d|ACK|Even if we have no faces, we Shadows still have hearts that can be blackened."

# debug RCV line generator (for testing without hardware)
def debug_RX() -> str:
    import random
    ex_addr = random.randint(0, 50)
    ex_type = random.choice(["U", "I", "P", "M", "E", "B", "RW", "MARI", "C", "R"])
    ex_seq = random.randint(0, 2**32 - 1) # large random sequence number
    
    # Generate correct number of values based on type
    if ex_type == "P":
        # PID type: 9 values (pitchP, rollP, yawP, pitchI, rollI, yawI, pitchD, rollD, yawD)
        ex_payload = ",".join(str(random.randint(0, 9)) for _ in range(9))
    elif ex_type == "B":
        # BATTERY type: 1 value (battery status)
        ex_payload = str(random.randint(0, 2))  # 0=CRITICAL, 1=WARNING, 2=NORMAL
    elif ex_type == "I":
        # IMU type: 5 values (pitch, roll, yaw, temperature, pressure)
        rand_pitch = random.uniform(-180, 180)
        rand_roll = random.uniform(-90, 90)
        rand_yaw = random.uniform(0, 360)
        rand_temp = random.uniform(-20, 40)
        rand_pressure = random.uniform(950, 1050)
        ex_payload = f"{rand_pitch:.1f},{rand_roll:.1f},{rand_yaw:.1f},{rand_temp:.1f},{rand_pressure:.1f}"
    elif ex_type == "RW":
        # RAW type: string payload (for testing, we can just use a fixed string or random text)
        raws = ["all it costs is your love", "Sunny... I love you...", "You'll forgive yourself, won't you? You can do this, Sunny."]
        ex_payload = random.choice(raws)
    elif ex_type == "M":
        # MOTOR type: 4 values (motor1, motor2, motor3, motor4)
        ex_payload = ",".join(str(random.randint(1100, 1800)) for _ in range(4))
    elif ex_type == "MARI":
        # MARI type: string payload (for testing, we can just use a fixed string or random text)
        ex_seq = random.randint(1, 4) # Use sequence to indicate test ID for MARI
        ex_payload = f"{'X' * 2**(ex_seq + 3)}"
    elif ex_type == "C":
        # CPU type: 5 floats (flight, tele_tx, tele_rx, housekeeping, idle CPU%)
        ex_payload = ",".join(f"{random.uniform(0, 100):.1f}" for _ in range(5))
    elif ex_type == "R":
        # RATE type: 4 ints (flight, tele_tx, tele_rx, housekeeping rates in Hz)
        ex_payload = ",".join(str(random.randint(50, 1000)) for _ in range(4))
    else:
        # U, E types: 4 values each
        ex_payload = ",".join(str(random.randint(0, 9)) for _ in range(4))

    ex_uuid = f"{ex_type}:{ex_seq}"
    ex_data = f"{ex_uuid}|{ex_payload}"
    ex_length = len(ex_data) # LoRa modem reports the byte length of the data field
    ex_rssi = random.randint(-120, 0)
    ex_snr = random.randint(-20, 10)

    return f"+RCV={ex_addr},{ex_length},{ex_data},{ex_rssi},{ex_snr}"

def list_serial_ports() -> list[str]:
    ports: list[str] = []
    for p in list_ports.comports():
        desc = f"{p.device}"
        if p.description:
            desc += f" - {p.description}"
        if p.manufacturer:
            desc += f" ({p.manufacturer})"
        ports.append(desc)
    return ports

'''
RSSI is the received signal strength in dBm (negative values, closer to 0 is stronger)
SNR is the signal-to-noise ratio in dB (can be negative, higher is better)

'''
#Change later
def get_link_quality(rssi: int, snr: int) -> str:
    
    sq =  0.6 * (rssi - MAX_RF_SENSITIIVITY) + 2.0 * snr
    
    if rssi == 0 and snr == 0:
        return "No Signal"

    if sq < 20:
        return "Poor"
    elif sq < 40:
        return "Weak"
    elif sq < 60:
        return "Fair"
    elif sq < 80:
        return "Good"
    else:
        return "Strong"

class MariDrone:
    def __init__(self, address: int, network_id: int, payload_len: int, sensor_stat: parsestuff.DroneStatus, rssi: int, snr: int):
        self.address = address # (MARI's own address, used for sending commands)
        self.network_id = network_id
        self.payload_len = payload_len
        self.sensor_stat = sensor_stat
        self.rssi = rssi
        self.snr = snr
        self.link_quality = get_link_quality(rssi, snr)
        self.packet_stats: dict[str, parsestuff.PacketStats] = {
            "U": parsestuff.PacketStats(),
            "I": parsestuff.PacketStats(),
            "P": parsestuff.PacketStats(),
            "M": parsestuff.PacketStats(),
            "E": parsestuff.PacketStats(),
            "B": parsestuff.PacketStats(),
            "RW": parsestuff.PacketStats(),
            "MARI": parsestuff.PacketStats(),
            "C": parsestuff.PacketStats(),
            "R": parsestuff.PacketStats()
            }
        self.rtos_stat: parsestuff.RTOS_Status = parsestuff.RTOS_Status()
        self.sunny_send_time = 0.0
        self.sunny_recv_time = 0.0
        self.sunny_rtt = 0.0
        
        self.ping_state: parsestuff.PingState = parsestuff.PingState.DISCONNECTED
        
        self.pending_sunnys: dict[int, float] = {}

class SerialMonitor:
    def __init__(
        self,
        port: str,
        baud: int,
        raw: bool = False,
        log_path: Optional[str] = None,
        output_cb: Optional[Callable[[str], None]] = None,
        status_cb: Optional[Callable[[tuple[str, parsestuff.DroneStatus]], None]] = None,
        rtos_cb: Optional[Callable[[parsestuff.RTOS_Status], None]] = None,
        debug: bool = False,
        display: bool = True,
        data_path: Optional[str] = None,
        mari: Optional['MariDrone'] = None
    ):
        self.port = port
        self.baud = baud
        self.raw = raw
        self.log_path = log_path
        self.data_path = data_path
        self._stop = threading.Event()
        self._ser: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self.debug = debug
        self.display = display
        self._flush_counter = 0
        self.mari = mari

        self.data = b""

        self._log_fp = open(log_path, "a", encoding="utf-8") if log_path else None
        self._data_fp = open(data_path, "a", encoding="utf-8") if data_path else None
        self._output_cb = output_cb
        self._status_cb = status_cb
        self._rtos_cb = rtos_cb

    # Callbacks for output and status updates
    def set_mari(self, mari: 'MariDrone') -> None:
        self.mari = mari

    def set_output_cb(self, cb: Optional[Callable[[str], None]]) -> None:
        self._output_cb = cb

    # status_cb receives a tuple of (type, DroneStatus) where type is one of "U", "I", "P", "M", "E", "B"
    def set_status_cb(self, cb: Optional[Callable[[tuple[str, parsestuff.DroneStatus]], None]]) -> None:
        self._status_cb = cb

    # rtos_cb receives the rolling RTOS_Status instance whenever a "C" or "R" frame updates it
    def set_rtos_cb(self, cb: Optional[Callable[[parsestuff.RTOS_Status], None]]) -> None:
        self._rtos_cb = cb

    # Update status from a received frame
    def _update_status_from_frame(self, frame: Optional[parsestuff.RcvFrame]) -> None:
        if not frame:
            return

        data_frame = parsestuff.parse_data_line(frame.data)
        if not data_frame:
            return
        
        # For MARI packets and gatekeeping ping state
        if data_frame.type == "MARI" and data_frame.sequence is not None:
            recv_time = time.perf_counter()
            seq = data_frame.sequence
            # Check if we are waiting for this specific ping sequence
            if seq in self.mari.pending_sunnys:
                send_time = self.mari.pending_sunnys.pop(seq) # Remove it from pending
                self.mari.sunny_rtt = parsestuff.get_rtt(send_time, recv_time)
                
                # Verify payload integrity
                expected_full = SUNNY_PING_ACK % seq
                expected_id, expected_payload = expected_full.split("|", 1)
                actual_payload = str(data_frame.payload)
                actual_id = str(data_frame.uuid)

                self._emit(f"Sunny: Received {len(actual_payload)} bytes")
                if actual_payload == expected_payload:
                    self.mari.ping_state = parsestuff.PingState.CONNECTED
                    self._emit(f"Sunny: SUCCESS | Payload matches.")
                    self._emit(f"  Payload: {actual_payload}")
                    self._emit("Mari: Hi Sunny! Cliff-faced as usual, I see.")
                else:
                    self.mari.ping_state = parsestuff.PingState.TIMEOUT
                    self._emit(f"Sunny: FAILURE | Payload mismatch.")
                    self._emit(f"  Expected: {expected_payload!r} from {expected_id} {len(expected_payload)} bytes.")
                    self._emit(f"  Got:      {actual_payload!r} from {actual_id} with {len(actual_payload)} bytes.")
            else:
                if self.display and self.raw == False:
                    self._emit(f"Received unexpected MARI packet with sequence {seq}. Ignoring.")
                return
            
        if self.mari.ping_state != parsestuff.PingState.CONNECTED and data_frame.type not in ("MARI", "RW"):
            # self._emit(f"{data_frame.uuid}")

            return

        if data_frame.type in ("C", "R"): # RTOS CPU usage / task rates
            if parsestuff.update_rtos_status(data_frame, self.mari.rtos_stat) and self._rtos_cb:
                self._rtos_cb(self.mari.rtos_stat)

        if data_frame.type in self.mari.packet_stats and data_frame.sequence is not None:
            self.mari.packet_stats[data_frame.type].add(data_frame.sequence)


        status = parsestuff.sort_data_line(data_frame)
        if status and self._status_cb:
            self._status_cb((data_frame.type, status))
            
        if status and self.data_path:
            try:
                packet_stat = self.mari.packet_stats.get(data_frame.type, parsestuff.PacketStats()) if self.mari else parsestuff.PacketStats()
                parsestuff.save_status_to_csv(self.data_path, data_frame.type, data_frame.sequence, status, packet_stat)
            except Exception as e:
                self._emit(f"system: CSV write failed: {e}")

    #------------------ Serial Communication ------------------
    def open(self) -> None:
        self._stop.clear()
        if not self.debug:
            self._ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.2,
                write_timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.2)

        if self.debug:
            self._reader_thread = threading.Thread(target=self._reader_loop_debug, daemon=True)
        else:
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

        if self._ser:
            try:
                if self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None

    def close_log(self) -> None:
        if self._log_fp:
            try:
                self._log_fp.close()
            except Exception:
                pass
            self._log_fp = None
        self._flush_counter = 0
        self.log_path = None
    
    def close_data(self) -> None:
        if self._data_fp:
            try:
                self._data_fp.close()
            except Exception:
                pass
            self._data_fp = None
        self.data_path = None

    def _emit(self, s: str) -> None:
        if self._output_cb:
            self._output_cb(s)

        if self._log_fp:
            self._log_fp.write(s + "\n")
            self._flush_counter += 1
            if self._flush_counter >= FLUSH_INTERVAL:
                self._log_fp.flush()
                self._flush_counter = 0

    def log(self, user: str, string: str) -> None:
        if self._log_fp:
            self._log_fp.write(f"[{parsestuff.now_ts()}] {user}: {string}\n")
            self._flush_counter += 1
            if self._flush_counter >= FLUSH_INTERVAL:
                self._log_fp.flush()
                self._flush_counter = 0

    def _reader_loop(self) -> None:
        buf = b""
        last_frame: Optional[parsestuff.RcvFrame] = None

        while not self._stop.is_set():
            if not self._ser:
                break
            try:
                chunk = self._ser.read(4096)
            except Exception as e:
                self._emit(f"system: Read failed: {e} @ {parsestuff.now_ts()}")
                break

            if not chunk:
                continue

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.rstrip(b"\r")
                try:
                    text = line.decode(errors="replace")
                except Exception:
                    text = repr(line)

                frame = parsestuff.parse_rcv_line(text)
                data = parsestuff.parse_data_line(frame.data) if frame else None

                # RW messages always print as just their payload string, ignoring display/raw flags
                if data and data.type == "RW":
                    self._emit(str(data.payload))
                elif self.display:
                    if self.raw:
                        if data and data.type != "MARI":
                            self._emit(f"[{parsestuff.now_ts()}] {data.uuid} | {data.payload!r}")
                    else:
                        if frame:
                            self._emit(
                                f"[{parsestuff.now_ts()}] RX: +RCV | addr={frame.address} | len={frame.length} | "
                                f"rssi={frame.rssi} | snr={frame.snr} | data={frame.data!r}"
                            )
                        else:
                            self._emit(f"[{parsestuff.now_ts()}] RX: {text}")

                last_frame = frame
                self.data = last_frame.data.encode() if last_frame else text.encode()
                if last_frame and self.mari:
                    self.mari.rssi = last_frame.rssi
                    self.mari.snr = last_frame.snr
                    self.mari.link_quality = get_link_quality(last_frame.rssi, last_frame.snr)
                self._update_status_from_frame(frame)
                
                self.log("system", text)


    # alternate reader loop for testing without hardware (generates debug lines instead of reading serial)
    def _reader_loop_debug(self) -> None:
        while not self._stop.is_set():
            try:
                text = debug_RX()
                frame = parsestuff.parse_rcv_line(text)
                data = parsestuff.parse_data_line(frame.data) if frame else None

                # RW messages always print as just their payload string, ignoring display/raw flags
                if data and data.type == "RW":
                    self._emit(str(data.payload))
                elif self.display:
                    if self.raw:
                        if data and data.type != "MARI":
                            self._emit(f"[{parsestuff.now_ts()}] {data.uuid} | {data.payload!r}")
                    else:
                        self._emit(f"[{parsestuff.now_ts()}] debug RX: {text}")
                self.data = frame.data.encode() if frame else b""
                if frame and self.mari:
                    self.mari.rssi = frame.rssi
                    self.mari.snr = frame.snr
                    self.mari.link_quality = get_link_quality(frame.rssi, frame.snr)
                self._update_status_from_frame(frame)
                self.log("debug", text)
                time.sleep(0.5)
            except Exception as e:
                self._emit(f"Debug loop error: {e}")
                time.sleep(1.0)


    def send_line(self, s: str) -> None:
        if not self._ser:
            raise RuntimeError("error: Serial port not open")

        data = s.encode("utf-8", errors="replace") + CRLF
        try:
            self._ser.write(data)
            self._ser.flush()
            self.log("system", s)
            self._emit(f"[{parsestuff.now_ts()}] TX: {s}")
        except Exception as e:
            self._emit(f"error: Write failed: {e} @ {parsestuff.now_ts()}")
        


def is_connected(mon: SerialMonitor) -> bool:
    return bool(getattr(mon, "_ser", None)) and mon._ser is not None and mon._ser.is_open


def log_user_input_if_enabled(mon: SerialMonitor, raw_line: str) -> None:
    if raw_line.strip() and getattr(mon, "_log_fp", None):
        try:
            mon.log("user", raw_line)
        except Exception:
            pass

