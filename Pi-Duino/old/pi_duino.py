import serial, time
port = "/dev/ttyACM0"
rate = 9600


s1 = serial.Serial(port, rate, timeout=0)
s1.flushInput()

s = [0]

if s1.isOpen():
	s1.flush()
	print("starting")
	s1.readline()
	for x in range(10):
		s1.write(b'add23')
		# read from serial
		print("Going to read")
		read_serial = s1.readline()
		# cast it to integer and print 
		s[0] = str(read_serial)
		print("x:" + str(x))
		print("Computing: " + s[0])

s1.close()
