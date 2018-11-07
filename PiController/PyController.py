from multiprocessing import Process, Value
import serial, time

#global variables
rate = 9600
s1 = serial.Serial(port, rate, timeout=0)
motorL = 0 # motor speeds
motorR = 0
pingD = 0 #ping distance
qeL = 0 # ir counts
qeR = 0
timeoutSecs = 1
#below variables only needed if pi doing QE distance calculations
PPR = 8
WHEEL_BASE = 137 #current approximation in mm, chassis constant
WHEEL_CIRCUMFERENCE = 219.9115


# this will be the process that we split off for Dmitry to do computer vision work in
# we sue shared memory to make passing information back and fourth
def vision(distLeft, distRight, angle, go):
    while (go):
    	# Do Dmitry stuff


#function to grab encoder data 
#have to decide if the arduino will return just increments or already calculate the distance per wheel and theta itself
def getEncoder():
	send = 'irr'
	s1.write(send.encode())
	response = (s1.readline(timeout=timeoutSecs)).decode("utf-8")

	if (!response):
		print ("No result received from Arduino on getEncoder call")
	else
		qe = response.split(' ')
		qeL = qe[0]
		qeR = qe[1]


#get ping distance
def getPing():
	send = 'png'
	s1.write(send.encode())
	response = s1.readline(timeout=timeoutSecs).decode("utf-8")

	if (!response):
		print ("No result received from Arduino on getPing call")
	else
		pingD = response


#stop motors
def stop():
	send = 'stp'
	s1.write(send.encode())


#set motor speed
def setMotors(motorSpeedL, motorspeedR):
	send = 'mtr' + str(motorSpeedL) + str(motorspeedR)
	s1.write(send.encode())

	#update the global variables once they're written to serial
	motorL = motorSpeedL
	motorR = motorSpeedR


def runController():
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
	s1.flushInput()

	# Do controller stuff
	response = ""

	count = 0
	running = True

	if s1.isOpen():
		s1.flush()
		print("PyController starting")
		
		while (running):
			#check distance to lines on either side & angle in lane
			#compute wheel speed adjustments based off of current speed and required corrections
			#set wheels to corrected speed
			#consider sleeping until a timeDelta has passed?

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

	#stop vehicle process. Set motor speeds to 0, close down serial port, and kill vision thread.
	setMotors(0,0)
	s1.close()
	# once we're all done, send the kill switch to the inner vision loop and join the vision process
    go.value = False
    p.join()


def runTracker():


# main method
if __name__ == '__main__':

	#runController()
	runTracker()
