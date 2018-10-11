import serial
port = "/dev/ttyACM0"

s1 = serial.Serial(port, 9600)

s1.flushInput()

while True:
	if s1.inWaiting() > 0:
		inputValue = s1.read(1)
		print(ord(inputValue))
