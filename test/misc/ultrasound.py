import serial
import time

PORT = "/dev/ttyACM2"
BAUD = 9600

ser = serial.Serial(
port=PORT,
baudrate=BAUD,
bytesize=8,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
timeout=0.5
)

def checksum_ok(frame):
    # frame length = 10
    # checksum = low byte of sum of first 9 bytes
    calc = sum(frame[0:9]) & 0xFF
    return calc == frame[9]

def parse_frame(frame):
    s1 = (frame[1] << 8) | frame[2]
    s2 = (frame[3] << 8) | frame[4]
    s3 = (frame[5] << 8) | frame[6]
    s4 = (frame[7] << 8) | frame[8]
    return s1, s2, s3, s4

print("DYP 4 ultrasonic sensor live reader started")
print("Port:", PORT)
print("Baud:", BAUD)
print("Press Ctrl+C to stop")

# while True:
#     # Trigger read command
#     ser.reset_input_buffer()
#     ser.write(bytes.fromhex("55"))
#     ser.flush()

#     time.sleep(0.1)

#     data = ser.read(10)

#     if len(data) != 10:
#         print("Bad/no frame:", data.hex(" "))
#         time.sleep(0.2)
#         continue

#     if data[0] != 0xFF:
#         print("Wrong header:", data.hex(" "))
#         time.sleep(0.2)
#         continue

#     if not checksum_ok(data):
#         print("Checksum fail:", data.hex(" "))
#         time.sleep(0.2)
#         continue

#     s1, s2, s3, s4 = parse_frame(data)

#     print(f"S1: {s1:4d} mm | S2: {s2:4d} mm | S3: {s3:4d} mm | S4: {s4:4d} mm")

#     time.sleep(0.2)


while True:
    ser.reset_input_buffer()
    ser.write(bytes.fromhex("55"))
    ser.flush()
    time.sleep(0.1)

    # sync on header instead of blind read(10)
    if ser.read(1) != b'\xff':
        time.sleep(0.2)
        continue
    rest = ser.read(9)
    if len(rest) != 9:
        time.sleep(0.2)
        continue
    data = b'\xff' + rest

    if not checksum_ok(data):
        print("Checksum fail:", data.hex(" "))
        time.sleep(0.2)
        continue

    s1, s2, s3, s4 = parse_frame(data)
    def fmt(v): return "err" if v >= 0xFFF0 else f"{v:4d}"
    print(f"S1: {fmt(s1)} | S2: {fmt(s2)} | S3: {fmt(s3)} | S4: {fmt(s4)} mm")
    time.sleep(0.2)