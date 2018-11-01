from multiprocessing import Process, Value
import serial, time

# this will be the process that we split off for Dmitry to do computer vision work in
# we sue shared memory to make passing information back and fourth
def vision(distLeft, distRight, angle, go):
    while (go):
    	# Do Dmitry stuff

# main method
if __name__ == '__main__':

	# Define and split off the computer vision subprocess _________________________________
	# define doubles
	distLeft = Value('d', -9.99)
	distRight = Value('d', -9.99)
	angle = Value('d', 0.0)
	
	# define boolean to act as an off switch
	go = ('b', True)

	# define and start the computer vision process
    p = Process(target=vision, args=(distLeft, distRight, angle, go))
    p.start()
    # _____________________________________________________________________________________

	# open the serial port to the Arduino
	s1 = serial.Serial(port, rate, timeout=0)
	s1.flushInput()

	# Do controller stuff
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

	# once we're all done, send the kill switch and join the vision process
    go.value = False
    p.join()
