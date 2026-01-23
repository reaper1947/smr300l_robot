import serial
import time

class SerialData:
    def __init__(self, port, baudrate=115200, interval=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for XIAO to reset
        self.interval = interval
        self.sequence = 0

    def compute_checksum(self, data: str) -> int:
        checksum = 0
        for ch in data:
            checksum ^= ord(ch)
        return checksum

    def create_message(self):
        payload = f"DATA{self.sequence}"
        checksum = self.compute_checksum(payload)
        return f"{payload}|{checksum}\n"

    def send_once(self):
        message = self.create_message()
        self.ser.write(message.encode('ascii'))
        # print(f"[TX] {message.strip()}")
        self.sequence += 1

    def read_response(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            # if line:
            #     print(f"[RX] {line}")

    def send_loop(self):
        try:
            while True:
                self.send_once()
                time.sleep(self.interval)
                self.read_response()
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            self.ser.close()


# if __name__ == "__main__":
#     port = "/dev/xiao"  # Update this to your actual port
#     sender = SerialData(port)
#     sender.send_loop()
