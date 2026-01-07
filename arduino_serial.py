import serial, time

class ArduinoComm:
    def __init__(self, port=None, baud=9600, timeout=1.0, simulate=True):
        self.simulate = simulate
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        if not simulate and port is not None:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            time.sleep(2)

    def send_angles(self, angles_deg):
        # angles_deg: list of floats (degrees) length = number of joints
        msg = ",".join([f"{float(a):.2f}" for a in angles_deg]) + "\n"
        if self.simulate:
            print("[SIM] Sending to Arduino:", msg.strip())
            return True
        if self.ser is None:
            raise RuntimeError("Serial port not open and not in simulate mode")
        try:
            self.ser.write(msg.encode('utf-8'))
            return True
        except Exception as e:
            print("Serial write failed:", e)
            return False

    def close(self):
        if self.ser is not None:
            self.ser.close()
