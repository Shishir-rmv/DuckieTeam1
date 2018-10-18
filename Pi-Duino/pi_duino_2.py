import serial, time, sys
port = "/dev/ttyACM0"
rate = 9600


s1 = serial.Serial(port, rate, timeout=2)

#time.sleep(2)

send = "add" + str(sys.argv[1] ) + str(sys.argv[2])

s1.write(send.encode())
time.sleep(1)
result = s1.readline()
print(result.decode("utf-8"))
