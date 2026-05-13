import sys
import argparse
import parsestuff
from PyQt6.QtWidgets import QApplication

from backend import SerialMonitor, MariDrone
from gui import FastLoRaGUI

def main():
    ap = argparse.ArgumentParser(description="LoRa Serial Monitor GUI")
    ap.add_argument("--port", help="Serial port (e.g. COM6)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--debug", action="store_true", default=False, help="Start in debug mode")
    ap.add_argument("--display", action="store_true", default=False, help="Start with display enabled")
    args = ap.parse_args()

    mari = MariDrone(
        address=0,
        network_id=18,
        payload_len=12,
        sensor_stat=parsestuff.DroneStatus(
                throttle=0.0, pitch_input=0.0, roll_input=0.0, yaw_input=0.0
                ,pitch=0.0, roll=0.0, yaw=0.0, altitude=0.0
                ,temperature=0.0, pressure=0.0
                ,pitchP=0.0, pitchI=0.0, pitchD=0.0,
                rollP=0.0, rollI=0.0, rollD=0.0,
                yawP=0.0, yawI=0.0, yawD=0.0
                ,raw_data=""
                ,motorNW=1000.0, motorNE=1000.0, motorSW=1000.0, motorSE=1000.0
                ,motors=[0.0, 0.0, 0.0, 0.0]),
        rssi=0,
        snr=0
    )

    mon = SerialMonitor(
        port=args.port or "",
        baud=args.baud,
        debug=args.debug,
        display=args.display,
        mari=mari
    )

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = FastLoRaGUI(mon=mon, mari=mari)
    gui.showMaximized()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()