from __future__ import annotations

"""
CLIv3_textual.py

Textual-based UI for your CLIv3 LoRa Serial Monitor, using the working DataTable+Input
pattern from test3.py (row/column keys + update_cell), so the table can update continuously
while you type on Windows.

How to run:
  pip install textual pyserial rich
  python CLIv3_textual.py --port COM6 --baud 115200

Inside the UI, type commands in the input box (same commands as CLIv3, no leading slash):
  help
  connect COM6 115200
  disconnect
  status
  list
  raw on|off
  log [path]|off
  ping
  reset
  setup915
  send "AT+..."
  target <addr>
  mari ping|connect|doko|oyasumi|ohayo
  quit / exit
"""

import argparse
import shlex
import sys
import threading
import time
from typing import Callable, Optional, TypeAlias

from rich import print as rprint
from rich.text import Text, Style

import serial
from serial.tools import list_ports

from textual.app import App, ComposeResult
from textual.containers import Vertical
from textual.widgets import DataTable, Input, Log
from textual.reactive import reactive

import parsestuff

CRLF = b"\r\n"

art = r"""
################################.  .................................  .#############################
##############################  .......................................  +######################++++
############################+ ............................................ .######################+ 
###########################  ............................................... .####################  
########################## ................................................... +################### 
########################+ .....................................................  ################## 
####################.  . ........................................................ .################ 
#################+  ............................................................... ############### 
###############. ................................................................... +#############.
############## ............................... ......................................  ############+
############  ...............................  ...................... ................. ###########+
########### .................................   .....................  ................. ###########
########## -................................  - .....................   ................. ##########
######### .................................. #+ ..................... # .................. #########
######## .................................. ### ....................  #+ .................  ########
#######  ................................. ####  ................... ###  ................. +#######
######  ................................. #####  .................. #####  ................. #######
#####+ ................................. ######. .........   ..... ####### .................. ######
##### ................................  #######+ ............          .##+ ................. -#####
####. ...............................  #  ###### ............... ##########- ................. #####
#### ...............................    ######## .............. ############  ................  ####
###  .........................      +########### .............       +#######  ................ ####
##.  ............................  +#####+#####+ ...........  #######+  .#####   ..............- ###
##  ............................ .###  .#######+ .. .......  ###########   ###+  ............... +##
#   ..........................  +#+ +##########+ .  ...... +####..########  ###- . .............+ ##
-  ..........................  ## +############+    ....  #####    ######## .###  +. ...........+ ##
  .......................... +#. #######..####. ## .... +######    ############- ##+  ..........-+ #
 .........................  ## .#######    ######+ ..  #########..############# +###- ...........+ +
 ........................ +## .########    ######   -########################## ### # ............+ 
....................... -#### ##########..##################################### ## ++ ............- 
..............   ...  +#################################### #######+##########+ #  #+ .............-
............. .+.  +######################################+ #####+.#.#+##-#### +   +- .............-
............. #################+##################################+ # #.##-### . ###  ..............
............ .###  +############ ##.##-#++#########################+ #.#+#####  ###  ...............
............  ######  ##########- #+ ##.#.+##################################  ###- ................
............. .#######. #########. #..####.################################## ###  .................
..............  ### -###.###########################           #############.   ....................
...............  +####### +####################### +########### +########### .......................
.................  ####### +#########+############ ############ ###########  .......................
...................   .----  #####. ..############# -######### +##########+ ........................
............................  +####  ###############. ####### +##########  .........................
..............................  ######################  ##+ .##########. ...........................
...............................  .###################################  .............................
.................................. -##############################+  ...............................
....................................  .#########################   .................................
........................................    .################+ .....................................
...........................................           -###.    .....................................
............................................ +##           -+  .....................................
............................................ .######+    ####. .....................................
............................................  ################ .....................................
............................................  ################     ..............  .................
.........................................     .-#############- #+-  ..........  +###-   ............
........................................  +#######   ####### .####+          +##########+  .........
......................................  ############### .   ########. ....... #############+  ......
.....................       ...       .##############+ +# -########### ......  ###############  ....
.................  .+########### .... .############+        +######### ....... ################. ...
..............  ################ ....  ########### .+++++++++ +##  #+    .....  ################# ..
...........  +################## ....  ########+ .  ++++++++ +-  +###      .... .################# .
.........  +#################### ....  #######  -++- +++++- +-   ##-#+ +## ..... ################## 
........  ###################### ..... ####   +++++++  ++  +-   ##.##+ -  . ..... ##################
....... +####################### ..... ###  -+++++++++   -++   #+ ## -  +#   .... .#################
.....  ######################### ..... -   ++++++++++++ -++  .#++##+### ###  ..... #################
....  ########################## .....  + -++++++++++++  -   #++##++### ###-  ....  ################
...  ########################### .......+ +++++++++++++    .#####-+####+####  ..... ################
.. .############################ ....... .+++++++++++++ + +#####.+####. #####  ....  ###############
. .############################# ....... ++++++++++++++ +######.#####+- .#####  .... ###############
 .############################## ......  +++++++++++++ .###### ###### ## #####+ ....  ##############
.############################### ...... .++++++++++++  ###### ######  +# -##### . ... +#############
################################ ...... -++++++++++   .#####+######+  .+# #####.   ... #############
###############################+ ...... -++++++++- . -############+ # + #+ ##### #. .. #############
###############################- ...... .++++++.  . -############# #.+. .#- #######- .  ############
###############################. ......  ++++  ..  +############# #########. #######+ . ############
###############################  ....... +  ..... ############### ######-### +#######-   ###########
     .+######################## ................  ############## .###### #############   ###########
    ####+   .################## ................ ##############.##.##### ##############  ###########
   ###########+   ############- ...............  ##############-## -####.##############  #########+ 
  .################   ########  ............  .# ################## +##+################ #####+  -##
  .####################  -###+ ..........  +#### ##############.##-. ##.################  #  .######
 +######################    +.........  ####################### #... ##.################## .########
 ######################        ....  #########################+   .. +##-##################+ #######
+#####################. ........  ############################   .... -#+####################  #####
#####################+ ......  +############################+. .......  +###################### +###
##################### ....   #############################+  ...........  +#####################  ##
#################### ...  ##########################++++   ...............  .-#################### #
################### .  -######################+        .....................    ################### 
###################  #######################   .................................  ##################
################+ -#######################   .....................................  ################
Credit: Omori is made by OMOCAT, LLC
"""

# debug RCV line generator (for testing without hardware)
def debug_RX() -> str:
    import random
    ex_addr = random.randint(0, 50)
    ex_length = random.randint(1, 4)
    ex_data = ",".join(str(random.randint(0, 9)) for _ in range(ex_length))
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

class MariDrone:
    def __init__(self, address: int, network_id: int, payload_len: int, sensor_stat: parsestuff.DroneStatus):
        self.address = address # (MARI's own address, used for sending commands)
        self.network_id = network_id
        self.payload_len = payload_len
        self.sensor_stat = sensor_stat


class SerialMonitor:
    """
    Same SerialMonitor concept as CLIv3, but with an optional output callback so that
    RX/TX lines can be routed into Textual's Log widget instead of the normal terminal.
    """
    def __init__(
        self,
        port: str,
        baud: int,
        raw: bool = False,
        log_path: Optional[str] = None,
        output_cb: Optional[Callable[[str], None]] = None,
        status_cb: Optional[Callable[[parsestuff.DroneStatus], None]] = None,
        debug: bool = False,
        display: bool = True,
    ):
        self.port = port
        self.baud = baud
        self.raw = raw
        self.log_path = log_path
        self._stop = threading.Event()
        self._ser: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self.debug = debug
        self.display = display


        self.data = b""
        self.rssi = 0
        self.snr = 0

        self._log_fp = open(log_path, "a", encoding="utf-8") if log_path else None
        self._output_cb = output_cb
        self._status_cb = status_cb

    def set_output_cb(self, cb: Optional[Callable[[str], None]]) -> None:
        self._output_cb = cb

    def set_status_cb(self, cb: Optional[Callable[[parsestuff.DroneStatus], None]]) -> None:
        self._status_cb = cb

    def _update_status_from_frame(self, frame: Optional[parsestuff.RcvFrame]) -> None:
        if not frame:
            return

        status = parsestuff.parse_data_line(frame.data)
        if status and self._status_cb:
            self._status_cb(status)

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
        self.log_path = None

    def _emit(self, s: str) -> None:
        if self._output_cb:
            self._output_cb(s)
        else:
            rprint(s)

        if self._log_fp:
            self._log_fp.write(s + "\n")
            self._log_fp.flush()

    def log(self, user: str, string: str) -> None:
        if self._log_fp:
            self._log_fp.write(f"[{parsestuff.now_ts()}] {user}: {string}\n")
            self._log_fp.flush()

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

                if self.display:             
                    frame = parsestuff.parse_rcv_line(text) if not self.raw else None
                    
                    if self.raw:
                        self._emit(f"[{parsestuff.now_ts()}] RX: {text}")
                    else:
                        if frame:
                            self._emit(
                                f"[{parsestuff.now_ts()}] RX: +RCV | addr={frame.address} | len={frame.length} | "
                                f"rssi={frame.rssi} | snr={frame.snr} | data={frame.data!r}"
                            )
                        else:
                            self._emit(f"[{parsestuff.now_ts()}] RX: {text}")
                else:
                    frame = parsestuff.parse_rcv_line(text) if not self.raw else None

                last_frame = frame
                self.data = last_frame.data.encode() if last_frame else b""
                self.rssi = last_frame.rssi if last_frame else 0
                self.snr = last_frame.snr if last_frame else 0
                self._update_status_from_frame(frame)
                
                self.log("system", text)

                # keep the UI snappy; don't sleep long here
                time.sleep(0.02)

    # alternate reader loop for testing without hardware (generates debug lines instead of reading serial)
    def _reader_loop_debug(self) -> None:
        while not self._stop.is_set():
            text = debug_RX()
            frame = parsestuff.parse_rcv_line(text)
            if self.display:
                self._emit(f"[{parsestuff.now_ts()}] debug RX: {text}")
            self.data = frame.data.encode() if frame else b""
            self.rssi = frame.rssi if frame else 0
            self.snr = frame.snr if frame else 0
            self._update_status_from_frame(frame)
            self.log("debug", text)
            time.sleep(0.5)


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
        time.sleep(0.05)


def is_connected(mon: SerialMonitor) -> bool:
    return bool(getattr(mon, "_ser", None)) and mon._ser is not None and mon._ser.is_open


def log_user_input_if_enabled(mon: SerialMonitor, raw_line: str) -> None:
    if raw_line.strip() and getattr(mon, "_log_fp", None):
        try:
            mon.log("user", raw_line)
        except Exception:
            pass


# --------------------------- Textual App ---------------------------

class LoRaTextualApp(App):
    CSS = """
    Screen { padding: 1; }
    #main { height: 1fr; }
    DataTable { height: 1fr; }
    Log { height: 20; border: solid $primary; }
    Input { border: solid $primary; }
    
    """

    # reactive fields shown in table
    port: str = reactive("")
    baud: str = reactive("")
    conn: str = reactive("DISCONNECTED")
    last_rssi: int = reactive(0)
    last_snr: int = reactive(0)
    len: int = reactive(0)
    target: str = reactive(0)
    last_data: str = reactive("")
    pitch: float = reactive(0.0)
    roll: float = reactive(0.0)
    yaw: float = reactive(0.0)
    altitude: float = reactive(0.0)

    def __init__(self, mon: SerialMonitor, mari: MariDrone):
        super().__init__()
        self.mon = mon
        self.mari = mari
        self.mari_connected = False
        self.debug_mode = mon.debug
        self._app_thread_id: Optional[int] = None

        # command registry
        CommandFn: TypeAlias = Callable[[list[str]], Optional[bool]]
        self.COMMANDS: dict[str, tuple[str, bool, CommandFn]] = {}

        def command(name: str, help_text: str, requires_conn: bool = False):
            def wrap(fn: CommandFn):
                self.COMMANDS[name.lower()] = (help_text, requires_conn, fn)
                return fn
            return wrap

        # ----- Commands -----

        @command("help", "help [mari]|[command] - show help")
        def cmd_help(argv: list[str]) -> None:
            if not argv:
                self._log_lines(
                    "COMMANDS: connect, disconnect, status, list, debug, raw on|off, log [path]|off, \n"
                    "ping, reset, setup915, send \"AT+...\", target <addr>, mari <cmd>, exit/quit \n"
                    "clear, display\n"
                    "Type 'help <command>' for details on a specific command."
                )
                self._log_lines("Type: help mari   for MARI commands.")
                return
            arg0 = argv[0].lower()
            if arg0 == "mari":
                self._log_lines("MARI COMMANDS: mari ping | mari connect | mari doko | mari oyasumi | mari ohayo")
                return
            entry = self.COMMANDS.get(arg0)
            if entry:
                self._log_lines(f"HELP: {arg0} - {entry[0]}")
            else:
                self._log_lines(f"Unknown topic: {arg0}")

        @command("art", "art - show Mari ASCII art in log")
        def cmd_art(argv: list[str]) -> None:
            self._log_lines(art)

        @command("status", "Show connection status")
        def cmd_status(argv: list[str]) -> None:
            self._log_lines(self._status_string())

        @command("list", "List available serial ports")
        def cmd_list(argv: list[str]) -> None:
            ports = list_serial_ports()
            if not ports:
                self._log_lines("No serial ports found.")
            else:
                self._log_lines("Available serial ports:")
                for p in ports:
                    self._log_lines(f"  {p}")

        @command("connect", "connect [PORT] [BAUD] - connect to serial port")
        def cmd_connect(argv: list[str]) -> None:
            port = argv[0] if len(argv) >= 1 else self.mon.port
            baud = int(argv[1]) if len(argv) >= 2 else self.mon.baud
            if not port:
                self._log_lines("error: Provide a port (e.g. connect COM6 115200) or run with --port.")
                return
            if is_connected(self.mon):
                self.mon.close()
            self.mon.port = port
            self.mon.baud = baud
            try:
                self.mon.open()
                self._log_lines(f"Connected: {self.mon.port} @ {self.mon.baud}")
            except Exception as e:
                self._log_lines(f"error: Failed to open {self.mon.port} @ {self.mon.baud}: {e}")

        @command("disconnect", "Disconnect serial port")
        def cmd_disconnect(argv: list[str]) -> None:
            if is_connected(self.mon):
                p = self.mon.port
                self.mon.close()
                self._log_lines(f"Disconnected from {p}.")
            else:
                self._log_lines("Already disconnected.")

        @command("raw", "raw on|off - toggle raw RX output")
        def cmd_raw(argv: list[str]) -> None:
            if not argv or argv[0].lower() not in ("on", "off"):
                self._log_lines("usage: raw on|off")
                return
            self.mon.raw = (argv[0].lower() == "on")
            self._log_lines(f"raw={self.mon.raw}")
            try:
                self.mon.log("system", f"raw={self.mon.raw}")
            except Exception:
                pass

        @command("log", "log [path]|off - enable/disable logging")
        def cmd_log(argv: list[str]) -> None:
            if not argv:
                path = "log.txt"
                self.mon.close_log()
                self.mon.log_path = path
                self.mon._log_fp = open(path, "a", encoding="utf-8")
                self.mon.log("system", f"Logging enabled -> {path}")
                self._log_lines("system: Logging to log.txt")
                return

            if argv[0].lower() == "off":
                try:
                    self.mon.log("system", "Logging disabled.")
                except Exception:
                    pass
                self.mon.close_log()
                self._log_lines("system: Logging disabled.")
                return

            path = " ".join(argv).strip()
            if not path:
                self._log_lines("usage: log [path]|off")
                return
            self.mon.close_log()
            self.mon.log_path = path
            self.mon._log_fp = open(path, "a", encoding="utf-8")
            self.mon.log("system", f"Logging enabled -> {path}")
            self._log_lines(f"system: Logging to {path}")

        @command("ping", "Send AT (requires connection)", requires_conn=True)
        def cmd_ping(argv: list[str]) -> None:
            self.mon.send_line("AT")

        @command("reset", "Send AT+RESET (requires connection)", requires_conn=True)
        def cmd_reset(argv: list[str]) -> None:
            self.mon.send_line("AT+RESET")

        @command("setup915", "Quick setup for 915MHz (requires connection)", requires_conn=True)
        def cmd_setup915(argv: list[str]) -> None:
            setup = [
                "AT+NETWORKID=18",
                "AT+ADDRESS=1",
                "AT+BAND=915000000",
                "AT+MODE=0",
                "AT+PARAMETER=9,7,1,12",
                "AT+CRFOP=10",
            ]
            for s in setup:
                self.mon.send_line(s)
                time.sleep(0.15)

        @command("send", 'Send AT command to LoRa: send "AT+..." (requires connection)', requires_conn=True)
        def cmd_send(argv: list[str]) -> None:
            if not argv:
                self._log_lines('usage: send "AT+..."')
                return
            at = " ".join(argv).strip()
            if not at.upper().startswith("AT"):
                self._log_lines("error: AT command must start with 'AT'")
                return
            self.mon.send_line(at)

        @command("target", "Set target address for MARI commands")
        def cmd_target(argv: list[str]) -> None:
            if not argv:
                self._log_lines("Usage: target <address>")
                return
            try:
                addr = int(argv[0])
                self.mari.address = addr
                self._log_lines(f"Target address set to {addr}")
            except ValueError:
                self._log_lines("Error: Invalid address")
                
        @command("clear", "Clear the log output")
        def cmd_clear(argv: list[str]) -> None:
            self._log_lines("system: Clearing log output...")
            self.logw.clear()
            self._handle_status_update(parsestuff.DroneStatus(pitch=0.0, roll=0.0, yaw=0.0, altitude=0.0))

        @command("debug", "Toggle debug RX line generator (no hardware needed)")
        def cmd_debug(argv: list[str]) -> None:
            self.debug_mode = not self.debug_mode
            self._log_lines(f"Debug mode {'enabled' if self.debug_mode else 'disabled'}")
            self.mon.close()
            self.mon.debug = self.debug_mode
            if self.debug_mode:
                self.mon.open()  # open() skips serial.Serial() when debug=True
            else:
                if self.mon.port:
                    try:
                        self.mon.open()
                    except Exception as e:
                        self._log_lines(f"error: Failed to open {self.mon.port} @ {self.mon.baud}: {e}")
                else:
                    self._log_lines("Note: No port set. Use 'connect <PORT> <BAUD>' to reconnect. Use 'list' to see available ports.")

        @command("display", "display - Toggle display of received messages")
        def cmd_display(argv: list[str]) -> None:
            self.mon.display = not self.mon.display
            self._log_lines(f"Display of received messages {'enabled' if self.mon.display else 'disabled'}")
        
        @command("mari", "MARI system commands (requires connection)", requires_conn=True)
        def cmd_mari(argv: list[str]) -> None:
            addr = self.mari.address

            mari_ping_TX = "Hi, OMORI! Cliff-faced as usual, I see."
            mari_ping_RX = "You'll forgive yourself... Won't you.. Sunny?"

            if not argv:
                self._log_lines("MARI COMMANDS: ping, connect, doko, oyasumi, ohayo")
                self._log_lines(f"(current target addr={addr})")
                return

            arg = argv[0].lower()

            if arg == "ping":
                # (same behavior as CLIv3)
                self.mon.send_line(f"AT+SEND={addr},{39},{mari_ping_TX}")
                try:
                    self.mon.log("mari", mari_ping_TX)
                except Exception:
                    pass

                # If you want deterministic behavior, parse mon.data, but it's timing-dependent.
                if self.mon.data.decode(errors="replace") == mari_ping_RX:
                    self._log_lines("Mari: We never did get to play at that last recital...")
                else:
                    self._log_lines("Mari: ... (No response)")

            elif arg == "connect":
                self.mari_connected = True
                self._log_lines("Mari: I'm doing fine, thank you for asking. I hope you're doing well too.")
                try:
                    self.mon.log("mari", 'AT+MARI="Connected"')
                except Exception:
                    pass

            elif arg == "doko":
                if not self.mari_connected:
                    self._log_lines("Mari: ... (Not connected to MARI system)")
                    return
                self._log_lines("Mari: My sensors are all functioning within normal parameters.")

            elif arg == "oyasumi":
                if not self.mari_connected:
                    self._log_lines("Mari: ... (Not connected to MARI system)")
                    return
                self._log_lines("Mari: ...")

            elif arg == "ohayo":
                if not self.mari_connected:
                    self._log_lines("Mari: ... (Not connected to MARI system)")
                    return
                self._log_lines("Mari: ...")

            else:
                self._log_lines(f"error: Unknown MARI command: {arg}")

        @command("exit", "Exit program")
        def cmd_exit(argv: list[str]) -> bool:
            return True

        @command("quit", "Exit program")
        def cmd_quit(argv: list[str]) -> bool:
            return True

    # ----- UI plumbing -----

    def compose(self) -> ComposeResult:
        with Vertical(id="main"):
            yield DataTable(id="table")
            yield Log(id="log")
            yield Input(
                placeholder="Enter command (help, list, debug, connect COM6 115200, send \"AT+...\", quit)",
                id="cmd",
                highlighter=None,
            )

    def on_mount(self) -> None:
        self.table = self.query_one("#table", DataTable)
        self.logw = self.query_one("#log", Log)
        self._app_thread_id = threading.get_ident()

        # Route SerialMonitor output into the Log widget from any thread
        self.mon.set_output_cb(self._write_monitor_output)
        self.mon.set_status_cb(self._queue_status_update)

        # Columns/rows with explicit keys (avoids the RowKey/ColumnKey errors you hit)
        self.table.add_column("Metric", key="metric", width=20)
        self.table.add_column("", key="value", width=15)

        for key, label in [
            ("debug", "Debug Mode"),
            ("display", "Display RX"),
            ("conn", "Connection"),
            ("port", "Port"),
            ("baud", "Baud"),
            ("rssi", "Last RSSI (dBm)"),
            ("snr", "Last SNR"),
            ("data", "Last Data"),
            ("len", "Last Length (bytes)"),
            ("target", "MARI Target Addr"),
            ("pitch", "Pitch"),
            ("roll", "Roll"),
            ("yaw", "Yaw"),
            ("alt", "Altitude"),
        ]:
            self.table.add_row(label, "-", key=key)

        self.query_one("#cmd", Input).focus()
        self.set_interval(0.25, self._tick)

        # Auto-start debug mode if launched with --debug
        if self.debug_mode:
            self.mon.open()
            self._log_lines("Debug mode active (simulated RX, no serial required).")

        # Initial status
        self.logw.write_line(art)
        self._log_lines("""\n                    ---------------------------------
                        MARI LoRa CLI v3.3 Textual UI
                        Created by Seth Iris Canonigo
                        GitHub: siriscan
                    ---------------------------------""")
        self._log_lines("\nTextual UI started. Type 'help' for commands.")

    def _handle_status_update(self, status: parsestuff.DroneStatus) -> None:
        self.mari.sensor_stat = status

    def _write_monitor_output(self, line: str) -> None:
        if threading.get_ident() == self._app_thread_id:
            self.logw.write_line(line)
        else:
            self.call_from_thread(self.logw.write_line, line)

    def _queue_status_update(self, status: parsestuff.DroneStatus) -> None:
        if threading.get_ident() == self._app_thread_id:
            self._handle_status_update(status)
        else:
            self.call_from_thread(self._handle_status_update, status)

    def _status_string(self) -> str:
        if is_connected(self.mon):
            return f"CONNECTED {self.mon.port} @ {self.mon.baud}"
        return "NOT CONNECTED"

    def _tick(self) -> None:
        # update reactive fields from mon + mari
        if is_connected(self.mon):
            self.conn = "CONNECTED"
        elif self.debug_mode:
            self.conn = "DEBUG"
        else:
            self.conn = "DISCONNECTED"
        self.target = self.mari.address
        self.display = self.mon.display
        self.port = self.mon.port or "-"
        self.baud = str(self.mon.baud) if self.mon.baud else "-"
        self.last_rssi = int(self.mon.rssi)
        self.last_snr = int(self.mon.snr)
        self.last_data = self.mon.data.decode(errors="replace") if self.mon.data else ""

        self.pitch = float(self.mari.sensor_stat.pitch)
        self.roll = float(self.mari.sensor_stat.roll)
        self.yaw = float(self.mari.sensor_stat.yaw)
        self.altitude = float(self.mari.sensor_stat.altitude)

        # refresh table cells by keys (stable even if you sort later)
        self.table.update_cell("debug", "value", Text("ON", style="bold green") if self.debug_mode else Text("OFF", style="dim red"))
        self.table.update_cell("conn", "value", Text("CONNECTED", style="bold green") 
            if self.conn == "CONNECTED" else Text("DEBUG", style="bold yellow") if self.conn == "DEBUG" else Text("DISCONNECTED", style="bold red"))
        self.table.update_cell("port", "value", Text(self.port, style="italic green") if self.port != "-" and self.conn == "CONNECTED" else Text("-", style="dim red"))
        self.table.update_cell("display", "value", Text("ON", style="bold green") if self.display else Text("OFF", style="dim red"))
        self.table.update_cell("baud", "value", self.baud)
        self.table.update_cell("rssi", "value", str(self.last_rssi))
        self.table.update_cell("snr", "value", str(self.last_snr))
        self.table.update_cell("len", "value", str(len(self.last_data)))
        self.table.update_cell("target", "value", self.target)
        self.table.update_cell("data", "value", self.last_data[:120])  # prevent super wide rows
        self.table.update_cell("pitch", "value", f"{self.pitch:.2f}")
        self.table.update_cell("roll", "value", f"{self.roll:.2f}")
        self.table.update_cell("yaw", "value", f"{self.yaw:.2f}")
        self.table.update_cell("alt", "value", f"{self.altitude:.2f}")

    def _log_lines(self, s: str) -> None:
        self.logw.write_line(s)

    def on_input_submitted(self, event: Input.Submitted) -> None:
        raw = event.value
        event.input.value = ""
        if not raw.strip():
            return

        # log user input if enabled
        log_user_input_if_enabled(self.mon, raw)

        self._log_lines(f"> {raw}")

        try:
            parts = shlex.split(raw.strip())
        except ValueError as e:
            self._log_lines(f"error: {e}")
            return

        cmd_name = parts[0].lower()
        argv = parts[1:]

        entry = self.COMMANDS.get(cmd_name)
        if not entry:
            self._log_lines("error: Unknown command. Type 'help'.")
            self._log_lines('hint: To send LoRa AT commands, use: send "AT+..."')
            return

        help_text, needs_conn, fn = entry
        if needs_conn and not is_connected(self.mon):
            self._log_lines(f"error: '{cmd_name}' requires an active connection. Use 'connect' first.")
            return

        try:
            should_exit = fn(argv)
            if should_exit:
                self.exit()
        except Exception as e:
            self._log_lines(f"error: command '{cmd_name}' failed: {e}")

    def on_shutdown_request(self) -> None:
        # Called when user closes window / Ctrl+C etc.
        try:
            self.mon.log("system", "UI shutdown requested.")
        except Exception:
            pass
        try:
            self.mon.close()
            self.mon.close_log()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description="CLIv3 Textual UI (LoRa Serial Monitor)")
    ap.add_argument("--port", help="Serial port (e.g. COM6)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--raw", action="store_true", help="Raw output (no +RCV parsing)")
    ap.add_argument("--log", help="Log output to a file (append)")
    ap.add_argument("--debug", action="store_true", help="Start in debug mode (simulated RX, no serial required)")
    ap.add_argument("--display", action="store_true", help="Display RX messages")   
    args = ap.parse_args()

    mon = SerialMonitor(
        port=args.port or "",
        baud=args.baud,
        raw=args.raw,
        log_path=args.log,
        debug=args.debug,
        display=not args.raw
    )

    mari = MariDrone(
        address=0,  # default target address for MARI commands
        network_id=18,
        payload_len=12,
        sensor_stat=parsestuff.DroneStatus(pitch=0.0, roll=0.0, yaw=0.0, altitude=0.0),
    )

    app = LoRaTextualApp(mon=mon, mari=mari)
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
