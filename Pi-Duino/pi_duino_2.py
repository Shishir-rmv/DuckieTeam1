import serial, time
port = "/dev/ttyACM0"
rate = 9600


s1 = serial.Serial(port, rate, timeout=0)

s1.write("add23")
time.sleep(1)
result = s1.readline()
print(result)
