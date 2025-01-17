import serial
ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.1)
print(f"Using port: {ser.name}")
ihex_data = []
with open("blink.ihex", "r") as f:
    for line in f:
        ihex_data.append(line)

print(f"Got hex data: {ihex_data}")
while True:
    for d in ihex_data:
        ser.write(d.encode('utf-8'))
        r = ser.read(1000)
        print(r.decode('utf-8'))
