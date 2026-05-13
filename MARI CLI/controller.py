import shlex
import threading
import time
from typing import Callable, Optional

import parsestuff

from backend import SerialMonitor, MariDrone, list_serial_ports, is_connected, art

HELP_TEXT = """
COMMANDS:
  list                       List available serial ports
  help                       Show this help text
  connect [PORT] [BAUD]      Connect to serial port (e.g. connect COM6 115200)
  disconnect                 Close serial port
  display                    Toggles console display (ignores RW-type messages)
  status                     Show serial connection status
  baud <rate>                set serial baud rate (9600, 19200, 38400, 57600, 115200)
  ping                       send a SUNNY ping command to MARI Drone (requires connection)
  mari <arg>                 Send MARI command (requires serial connection) (requires ping connection)
                               Ex) mari setup - Quick setup example for 915MHz
                                   mari override - manually set MARI ping state to CONNECTED (for debug)
  clear                      clear the console log output, reset drone status, and clears graphs
  target <address>           Set target address for MARI commands
  raw on|off                 Toggle raw mode
  log <path>|off             Enable logging to file, or disable (no path => log.txt)
  plog <path>|off            enable/disable logging of parsed drone status to a file
  test                       Send AT to LoRa module (requires serial connection)
  reset                      Send AT+RESET (requires serial connection)                  
  send <arg>                 Send AT command to LoRa (requires serial connection).
                             arg must start with "AT+..." (Refer to RYLR998/RYLR498 AT Command Manual)
                               Ex) send "AT" - pings the LoRa module
                                   send "AT+PARAMETER=7,9,4,15" - sets RF parameters for LoRa
  art                        Show MARI ASCII art
  stopwatch <arg>            control a stopwatch timer in the GUI
    start|stop|reset 
  exit / quit                Exit program
"""

MARI_HELP_TEXT = """
MARI COMMANDS
  mari                        Show MARI command help
  mari <arg>                  MARI system commands
    arg:
      [sunny|ping] [1|2|3|4]  Ping MARI system
      time                 Send time command to MARI
      reset pid            Reset PID values
      gains                Query current PID gains
      kp <pitch|roll|yaw> <float>  Set Kp gain
      ki <pitch|roll|yaw> <float>  Set Ki gain
      kd <pitch|roll|yaw> <float>  Set Kd gain
      tare                 Tare IMU sensors
      ohayo                Arm motors
      oyasumi              Disarm motors
      setup                Quick setup example for 915MHz
"""

class CommandController:
    """
    Handles parsing and executing user commands completely independent of the GUI framework.
    Uses callbacks to instruct the UI on what to render (e.g., printing text, clearing the screen).
    """
    def __init__(
        self, 
        mon: SerialMonitor, 
        mari: MariDrone, 
        print_cb: Callable[[str], None], 
        clear_cb: Callable[[], None], 
        quit_cb: Callable[[], None],
        stopwatch_cb: Callable[[str], None]
    ):
        self.mon = mon
        self.mari = mari
        
        # Callbacks to speak to the GUI
        self.print_cb = print_cb            
        self.clear_cb = clear_cb            
        self.quit_cb = quit_cb              
        self.stopwatch_cb = stopwatch_cb    
        
        self.COMMANDS: dict[str, tuple[str, bool, Callable[[list[str]], None]]] = {}
        
        self._register_commands()

    def _command(self, name: str, help_text: str, requires_conn: bool = False):
        """Decorator to cleanly register commands to the internal dictionary."""
        def wrap(fn):
            self.COMMANDS[name.lower()] = (help_text, requires_conn, fn)
            return fn
        return wrap

    def _register_commands(self):
        @self._command("help", "help [mari]|[command] - show help")
        def cmd_help(argv: list[str]) -> None:
            if not argv:
                self.print_cb(HELP_TEXT)
                self.print_cb("Type: help mari   for MARI commands.")
                return
            arg0 = argv[0].lower()
            if arg0 == "mari":
                self.print_cb(MARI_HELP_TEXT)
                return
            entry = self.COMMANDS.get(arg0)
            if entry:
                self.print_cb(f"HELP: {arg0} - {entry[0]}")
            else:
                self.print_cb(f"Unknown topic: {arg0}")

        @self._command("art", "art - show Mari ASCII art in console")
        def cmd_art(argv: list[str]) -> None:
            self.print_cb(art)
            

        @self._command("status", "Show connection status to serial port")
        def cmd_status(argv: list[str]) -> None:
            if is_connected(self.mon):
                self.print_cb(f"CONNECTED {self.mon.port} @ {self.mon.baud}")
            else:
                self.print_cb("NOT CONNECTED")

        @self._command("list", "List available serial ports")
        def cmd_list(argv: list[str]) -> None:
            ports = list_serial_ports()
            if not ports:
                self.print_cb("No serial ports found.")
            else:
                self.print_cb("Available serial ports:")
                for p in ports:
                    self.print_cb(f"  {p}")

        @self._command("connect", "connect [PORT] [BAUD] [override] - connect to serial port (override: force reconnect with last settings)")
        def cmd_connect(argv: list[str]) -> None:
            override = "override" in [arg.lower() for arg in argv]
            
            # Filter out 'override' from argv for normal argument parsing
            filtered_argv = [arg for arg in argv if arg.lower() != "override"]
            
            port = filtered_argv[0] if len(filtered_argv) >= 1 else self.mon.port
            baud_str = filtered_argv[1] if len(filtered_argv) >= 2 else str(self.mon.baud)
            
            try:
                baud = int(baud_str)
            except ValueError:
                self.print_cb(f"error: Invalid baud rate '{baud_str}'")
                return
            
            if not port:
                self.print_cb("error: Provide a port (e.g. connect COM6 115200) or run with --port.")
                return
            
            if is_connected(self.mon):
                if override:
                    self.mon.close()
                    self.print_cb("Forcing reconnection...")
                else:
                    self.print_cb("Already connected. Use 'connect override' to force reconnection.")
                    return
            
            self.mon.port = port
            self.mon.baud = baud
            try:
                self.mon.open()
                self.print_cb(f"Connected: {self.mon.port} @ {self.mon.baud}")
            except Exception as e:
                self.print_cb(f"error: Failed to open {self.mon.port} @ {self.mon.baud}: {e}")

        @self._command("disconnect", "Disconnect serial port")
        def cmd_disconnect(argv: list[str]) -> None:
            if is_connected(self.mon):
                p = self.mon.port
                self.mon.close()
                self.print_cb(f"Disconnected from {p}.")
            else:
                self.print_cb("Already disconnected.")

        @self._command("baud", "baud <rate> - set serial baud rate (9600, 19200, 38400, 57600, 115200)")
        def cmd_baud(argv: list[str]) -> None:
            ALLOWED_BAUDS = [9600, 19200, 38400, 57600, 115200]
            
            if not argv:
                self.print_cb(f"Current baud rate: {self.mon.baud}")
                self.print_cb(f"Allowed rates: {', '.join(map(str, ALLOWED_BAUDS))}")
                return
            
            try:
                baud = int(argv[0])
            except ValueError:
                self.print_cb(f"error: invalid baud rate '{argv[0]}'")
                return
            
            if baud not in ALLOWED_BAUDS:
                self.print_cb(f"error: baud rate {baud} not allowed")
                self.print_cb(f"Allowed rates: {', '.join(map(str, ALLOWED_BAUDS))}")
                return
            
            if is_connected(self.mon):
                self.print_cb("error: cannot change baud rate while connected. Use 'disconnect' first.")
                return
            
            self.mon.baud = baud
            self.print_cb(f"Baud rate set to {baud}")

        @self._command("raw", "raw on|off - toggle raw RX output")
        def cmd_raw(argv: list[str]) -> None:
            if not argv or argv[0].lower() not in ("on", "off"):
                self.print_cb("usage: raw on|off")
                return
            self.mon.raw = (argv[0].lower() == "on")
            self.print_cb(f"raw={self.mon.raw}")
            try:
                self.mon.log("system", f"raw={self.mon.raw}")
            except Exception:
                pass

        @self._command("plog", "plog [path]|off - toggle parsed drone data logging")
        def cmd_plog(argv: list[str]) -> None:
            if not argv:
                path = f"drone_status_{time.strftime('%Y%m%d_%H%M%S')}.csv"
                self.mon.close_data()
                self.mon.data_path = path
                self.mon.log("system", f"Parsed logging to {path}")
                self.print_cb(f"system: Parsed logging to {path}")
                return

            if argv[0].lower() == "off":
                try:
                    self.mon.log("system", "Parsed logging disabled.")
                except Exception:
                    pass
                self.mon.close_data()
                self.print_cb("system: Parsed logging disabled.")
                return

            path = " ".join(argv).strip()
            if not path:
                path = f"drone_status_{time.strftime('%Y%m%d_%H%M%S')}.csv.csv"

            self.mon.close_data()
            self.mon.data_path = path
            self.mon.log("system", f"Parsed logging to {path}")
            self.print_cb(f"system: Parsed logging to {path}")

        @self._command("log", "log [path]|off - enable/disable logging")
        def cmd_log(argv: list[str]) -> None:
            if not argv:
                path = "log.txt"
                self.mon.close_log()
                self.mon.log_path = path
                self.mon._log_fp = open(path, "a", encoding="utf-8")
                self.mon.log("system", f"Logging enabled -> {path}")
                self.print_cb("system: Logging to log.txt")
                return

            if argv[0].lower() == "off":
                try:
                    self.mon.log("system", "Logging disabled.")
                except Exception:
                    pass
                self.mon.close_log()
                self.print_cb("system: Logging disabled.")
                return

            path = " ".join(argv).strip()
            if not path:
                self.print_cb("usage: log [path]|off")
                return
            self.mon.close_log()
            self.mon.log_path = path
            self.mon._log_fp = open(path, "a", encoding="utf-8")
            self.mon.log("system", f"Logging enabled -> {path}")
            self.print_cb(f"system: Logging to {path}")

        @self._command("ping", "Send SUNNY ping with a set 88-byte payload (requires connection)", requires_conn=True)
        def cmd_ping(argv: list[str]) -> None:
            addr = self.mari.address
            type = "SUN"
            ping_seq = 5
            ping_id = f"{type}:{ping_seq}"
            ping_TX = f"{ping_id}|SUNNY"
            len_payload = len(ping_TX)
            self.mari.pending_sunnys[ping_seq] = time.perf_counter()
            self.mari.ping_state = parsestuff.PingState.WAITING
            
            self.mon.send_line(f"AT+SEND={addr},{len_payload},{ping_TX}")
            self.print_cb(f"Mari: Sent sunny ping test: {ping_id} to address {addr}. Payload length: {len_payload} bytes.")
            try:
                self.mon.log("mari", f"Sent sunny ping test: {ping_id} to address {addr}. Payload length: {len_payload} bytes.")
            except Exception:
                pass
        
        @self._command("test", "Send AT command (requires connection)", requires_conn=True)
        def cmd_test(argv: list[str]) -> None:
            self.mon.send_line("AT")

        @self._command("reset", "Send AT+RESET (requires connection)", requires_conn=True)
        def cmd_reset(argv: list[str]) -> None:
            self.mon.send_line("AT+RESET")
            self.print_cb("Sent AT+RESET command to LoRa module.")

        @self._command("send", 'Send AT command to LoRa: send "AT+..." (requires connection)', requires_conn=True)
        def cmd_send(argv: list[str]) -> None:
            if not argv:
                self.print_cb('usage: send "AT+..."')
                return
            at = " ".join(argv).strip()
            if not at.upper().startswith("AT"):
                self.print_cb("error: AT command must start with 'AT'")
                return
            self.mon.send_line(at)

        @self._command("target", "Set target address for MARI commands")
        def cmd_target(argv: list[str]) -> None:
            if not argv:
                self.print_cb("Usage: target <address>")
                return
            try:
                addr = int(argv[0])
                self.mari.address = addr
                self.print_cb(f"Target address set to {addr}")
            except ValueError:
                self.print_cb("Error: Invalid address")
                
        @self._command("clear", "Clear the log output")
        def cmd_clear(argv: list[str]) -> None:
            self.print_cb("system: Clearing log output...")
            self.clear_cb()
            
            # Reset the drone status
            status = parsestuff.DroneStatus(
                throttle=0.0, pitch_input=0.0, roll_input=0.0, yaw_input=0.0,
                pitch=0.0, roll=0.0, yaw=0.0, altitude=0.0, temperature=0.0, pressure=0.0,
                pitchP=0.0, pitchI=0.0, pitchD=0.0,
                rollP=0.0, rollI=0.0, rollD=0.0,
                yawP=0.0, yawI=0.0, yawD=0.0,
                pitchPID=0.0, rollPID=0.0, yawPID=0.0,
                motorNW=0.0, motorNE=0.0, motorSW=0.0, motorSE=0.0,
                motors=[0.0, 0.0, 0.0, 0.0]
            )
            self.mari.sensor_stat = status

        @self._command("debug", "Toggle debug RX line generator (no hardware needed)")
        def cmd_debug(argv: list[str]) -> None:
            self.mon.debug = not self.mon.debug
            self.print_cb(f"Debug mode {'enabled' if self.mon.debug else 'disabled'}")
            self.mon.close()
            
            if self.mon.debug:
                self.print_cb("system: Debug mode enabled.")
                self.mon.log("system", "Debug mode enabled.")
                self.mon.open()
            else:
                self.print_cb("system: Debug mode disabled.")
                self.mon.log("system", "Debug mode disabled. Attempting to reconnect to serial port...")
                if self.mon.port:
                    try:
                        self.mon.open()
                    except Exception as e:
                        self.print_cb(f"error: Failed to open {self.mon.port} @ {self.mon.baud}: {e}")
                else:
                    self.print_cb("Note: No port set. Use 'connect <PORT> <BAUD>' to reconnect. Use 'list' to see available ports.")

        @self._command("display", "display - Toggle display of received messages")
        def cmd_display(argv: list[str]) -> None:
            self.mon.display = not self.mon.display
            self.print_cb(f"Display of received messages {'enabled' if self.mon.display else 'disabled'}")
            self.mon.log("system", f"Display of received messages {'enabled' if self.mon.display else 'disabled'}")

        @self._command("time", "time [debug|reader] <speed> - Set loop speed (1-10, where 1=0.1s, 10=1.0s)")
        def cmd_time(argv: list[str]) -> None:
            if len(argv) < 1:
                self.print_cb("usage: time <speed> (for debug loop) or time debug|reader <speed>")
                return
            
            try:
                if len(argv) == 1:
                    speed = int(argv[0])
                    interval = speed * 0.1
                    self.mon.debug_sleep_interval = interval
                    self.print_cb(f"Debug loop speed set to {interval:.1f}s (level {speed})")
                elif len(argv) >= 2:
                    loop_type = argv[0].lower()
                    speed = int(argv[1])
                    interval = speed * 0.1
                    
                    if loop_type == "debug":
                        self.mon.debug_sleep_interval = interval
                        self.print_cb(f"Debug loop speed set to {interval:.1f}s (level {speed})")
                    elif loop_type == "reader":
                        self.mon.reader_sleep_interval = interval
                        self.print_cb(f"Reader loop speed set to {interval:.1f}s (level {speed})")
                    else:
                        self.print_cb("error: Unknown loop type. Use 'debug' or 'reader'")
            except ValueError:
                self.print_cb("error: Speed must be an integer (1-10)")

        @self._command("mari", "MARI system commands (requires connection)", requires_conn=True)
        def cmd_mari(argv: list[str]) -> None:
            addr = self.mari.address          
            if not argv:
                self.print_cb(MARI_HELP_TEXT)
                self.print_cb(f"(current target addr={addr})")
                return
            arg = argv[0].lower()
            
            if arg == "sunny" or arg == "ping":
                try:
                    test_id = int(argv[1]) if len(argv) >= 2 else 1
                except ValueError:
                    self.print_cb("usage: mari ping [1|2|3|4]")
                    return

                type = "SUN"
                ping_id = f"{type}:{test_id}"
                ping_TX = f"{ping_id}|SUNNY"
                len_payload = len(ping_TX)

                self.mari.pending_sunnys[test_id] = time.perf_counter()
                self.mari.ping_state = parsestuff.PingState.WAITING

                self.mon.send_line(f"AT+SEND={addr},{len_payload},{ping_TX}")
                self.print_cb(
                    f"Mari: Sent sunny ping test: {ping_id} to address {addr}. Payload length: {len_payload} bytes."
                )
                try:
                    self.mon.log(
                        "mari",
                        f"Sent sunny ping test: {ping_id} to address {addr}. Payload length: {len_payload} bytes."
                    )
                except Exception:
                    pass

            elif arg == "time":
                mari_seq = int(time.time()) & 0xFF  # Use lower 8 bits (0-255)
                payload = f"MARI:{mari_seq:03d}|time"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent time command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent time command (seq={mari_seq})")
                except Exception:
                    pass
            elif arg == "realtime":
                mari_seq = int(time.time()) & 0xFF  # Use lower 8 bits (0-255)
                payload = f"MARI:{mari_seq:03d}|realtime"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent realtime command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent realtime command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "reset" and len(argv) >= 2 and argv[1].lower() == "pid":
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|reset pid"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent reset PID command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent reset PID command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "gains":
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|gains"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent gains query command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent gains query command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "kp":
                if len(argv) < 3:
                    self.print_cb("usage: mari kp <pitch|roll|yaw> <float>")
                    return
                axis = argv[1].lower()
                if axis not in ["pitch", "roll", "yaw"]:
                    self.print_cb("error: axis must be 'pitch', 'roll', or 'yaw'")
                    return
                try:
                    value = float(argv[2])
                except ValueError:
                    self.print_cb("error: value must be a float")
                    return
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|kp {axis} {value}"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent kp {axis} {value} command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent kp {axis} {value} command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "ki":
                if len(argv) < 3:
                    self.print_cb("usage: mari ki <pitch|roll|yaw> <float>")
                    return
                axis = argv[1].lower()
                if axis not in ["pitch", "roll", "yaw"]:
                    self.print_cb("error: axis must be 'pitch', 'roll', or 'yaw'")
                    return
                try:
                    value = float(argv[2])
                except ValueError:
                    self.print_cb("error: value must be a float")
                    return
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|ki {axis} {value}"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent ki {axis} {value} command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent ki {axis} {value} command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "kd":
                if len(argv) < 3:
                    self.print_cb("usage: mari kd <pitch|roll|yaw> <float>")
                    return
                axis = argv[1].lower()
                if axis not in ["pitch", "roll", "yaw"]:
                    self.print_cb("error: axis must be 'pitch', 'roll', or 'yaw'")
                    return
                try:
                    value = float(argv[2])
                except ValueError:
                    self.print_cb("error: value must be a float")
                    return
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|kd {axis} {value}"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent kd {axis} {value} command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent kd {axis} {value} command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "tare":
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|tare"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent tare command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent tare command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "ohayo":
                if self.mari.ping_state != parsestuff.PingState.CONNECTED:
                    self.print_cb("Mari: ... (Not connected to MARI system)")
                    return
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|ohayo"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent ohayo (arm motors) command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent ohayo (arm motors) command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "oyasumi":
                if self.mari.ping_state != parsestuff.PingState.CONNECTED:
                    self.print_cb("Mari: ... (Not connected to MARI system)")
                    return
                mari_seq = int(time.time()) & 0xFF
                payload = f"MARI:{mari_seq:03d}|oyasumi"
                len_payload = len(payload)
                self.mon.send_line(f"AT+SEND={addr},{len_payload},{payload}")
                self.print_cb(f"Mari: Sent oyasumi (disarm motors) command (seq={mari_seq})")
                try:
                    self.mon.log("mari", f"Sent oyasumi (disarm motors) command (seq={mari_seq})")
                except Exception:
                    pass

            elif arg == "override":
                self.mari.ping_state = parsestuff.PingState.CONNECTED
                self.print_cb("Mari: I'm doing fine, thank you for asking. I hope you're doing well too.")
                self.mon.log("mari", "Ping state manually overridden to CONNECTED by user command.")
                try:
                    self.mon.log("mari", "Mari connected. Sensors nominal.") 
                except Exception:
                    pass

            elif arg == "setup":
                setup = [
                    "AT+NETWORKID=18",
                    "AT+ADDRESS=1",
                    "AT+BAND=915000000",
                    "AT+MODE=0",
                    "AT+PARAMETER=7,7,1,8", #9,7,1,12 Default 
                    "AT+CRFOP=10",
                ]
                self.print_cb("Mari: Setting up 915MHz configuration...")
                for s in setup:
                    self.mon.send_line(s)
                    time.sleep(0.15)
                self.print_cb("Mari: Setup complete.")
            else:
                self.print_cb(f"error: Unknown MARI command: {arg}")

        @self._command("stopwatch", "stopwatch start|stop|reset - control stopwatch timer")
        def cmd_stopwatch(argv: list[str]) -> None:
            if not argv:
                self.print_cb("usage: stopwatch start|stop|reset")
                return
            
            action = argv[0].lower()
            if action in ["start", "stop", "reset"]:
                self.stopwatch_cb(action) # Delegates the action to the GUI
            else:
                self.print_cb(f"error: Unknown stopwatch command: {action}. Use 'start', 'stop', or 'reset'.")
                self.mon.log("system", f"error: Unknown stopwatch command: {action}.")

        @self._command("exit", "Exit program")
        def cmd_exit(argv: list[str]) -> None:
            self.quit_cb()

        @self._command("quit", "Exit program")
        def cmd_quit(argv: list[str]) -> None:
            self.quit_cb()

    def execute(self, raw_cmd: str) -> None:
        """Parses and executes a raw input string from the GUI."""
        if not raw_cmd.strip():
            return
            
        # Log user input if file logging is active
        if getattr(self.mon, "_log_fp", None):
            try:
                self.mon.log("user", raw_cmd)
            except Exception:
                pass

        self.print_cb(f"> {raw_cmd}")

        try:
            parts = shlex.split(raw_cmd.strip())
        except ValueError as e:
            self.print_cb(f"error: {e}")
            return

        cmd_name = parts[0].lower()
        argv = parts[1:]

        entry = self.COMMANDS.get(cmd_name)
        if not entry:
            self.print_cb("error: Unknown command. Type 'help'.")
            self.print_cb('hint: To send LoRa AT commands, use: send "AT+..."')
            return

        help_text, needs_conn, fn = entry
        if needs_conn and not is_connected(self.mon):
            self.print_cb(f"error: '{cmd_name}' requires an active connection. Use 'connect' first.")
            return

        try:
            fn(argv)
        except Exception as e:
            self.print_cb(f"error: command '{cmd_name}' failed: {e}")