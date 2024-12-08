import serial
ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
print(f"Using port: {ser.name}")
msg = b":10010000214601360121470136007EFE09D2190140\n"
while True:
    ser.write(msg)
    r = ser.read(1000)
    print(r.decode('utf-8'))
