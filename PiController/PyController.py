from multiprocessing import Process, Value
import serial, json, math
from datetime import datetime

#global variables

port = "/dev/ttyACM0"
rate = 9600
s1 = serial.Serial(port, rate, timeout=10)
motorL = 0  # motor speeds
motorR = 0
#ping distance
pingD = 0
L_ENC_DIST = 0  # change in wheel distances and associated angle change
R_ENC_DIST = 0
SPEED = 0
last_time = 0


ENC_DELTA_THETA = 0
ENC_DELTA_X = 0

THETA = 0
X = 0
last_X = 0
Y = 0

#below variables only needed if pi doing QE distance calculations
PPR = 8
#current approximation in mm, chassis constant
WHEEL_BASE = 137   
WHEEL_CIRCUMFERENCE = 219.9115


# this will be the process that we split off for Dmitry to do computer vision work in
# we sue shared memory to make passing information back and fourth
def vision(distLeft, distRight, angle, go):
    while (go):
    	# Do Dmitry stuff
    	pass


#function to grab encoder data 
#have to decide if the arduino will return just increments or already calculate the distance per wheel and theta itself

def getEncoder():
	send = 'irr'
	s1.write(send.encode())
	result = s1.readline().decode("utf-8")

	if (not result):
		print ("No result received from Arduino on getEncoder call")
	else:
		#result = (s1.readline()).decode("utf-8")
		qe = result.split(' ')
		# for debugging
		print("from encoder call: %s" % str(result))
		l_enc = qe[0]
		r_enc = qe[1]

		L_ENC_DIST = l_enc * WHEEL_CIRCUMFERENCE/PPR
		R_ENC_DIST = r_enc * WHEEL_CIRCUMFERENCE/PPR

 		#update the change in avg position and current heading
		ENC_DELTA_X = (L_ENC_DIST + R_ENC_DIST)/2;
		ENC_DELTA_THETA = math.atan2((R_ENC_DIST-L_ENC_DIST)/2, WHEEL_BASE/2);

 		#update overall global positioning
		THETA += ENC_DELTA_THETA;
		X += ENC_DELTA_X*math.cos(THETA);
		Y += ENC_DELTA_X*math.sin(THETA);



#get ping distance
def getPing():
	send = 'png'
	s1.write(send.encode())
	response = s1.readline().decode("utf-8")

	if (not response):
		print ("No result received from Arduino on getPing call")
	else:
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

	print("PyController starting")
	# open the serial port to the Arduino
	s1.flushInput()

	# Do controller stuff
	response = ""

	count = 0
	running = True

	if s1.isOpen():
		s1.flush()
		
		# while (running):
			#check distance to lines on either side & angle in lane
			#compute wheel speed adjustments based off of current speed and required corrections
			#set wheels to corrected speed
			#consider sleeping until a timeDelta has passed?
	#output = Kp*(goal-X)-Kd*SPEED

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
	print("PyTracer starting")
	# open the serial port to the Arduino
	s1.flushInput()

	# Do controller stuff
	response = ""

	count = 0
	running = True
	stopAt = 10
	records = {}

	if s1.isOpen():
		s1.flush()
		start = datetime.now()
		
		# while we're still within our window of execution
		while ((datetime.now() - start).total_seconds() < stopAt):
			# get data from arduino
			getEncoder()
			speed()

			# store it in the array
			records[float((datetime.now() - start).total_seconds())] = {"L_ENC_DIST" : L_ENC_DIST, "R_ENC_DIST" : R_ENC_DIST, "SPEED" : SPEED,
				"ENC_DELTA_THETA" : ENC_DELTA_THETA, "ARD_THETA" : ARD_THETA, "ARD_X" : ARD_X, "ARD_Y" : ARD_Y}

		#dump data to file
		print("dumping (%d) records to a JSON in the Logs folder" % len(records))
		with open('../Logs/tracer_%s.json' % str(datetime.now()), 'w') as fp:
			json.dump(records, fp, indent=4)

	# once finished
	setMotors(0,0)
	s1.close()

def speed():
	time = datetime.now()

	SPEED = (X - last_X)/((time - last_time).total_seconds())

	last_time = time
	last_X = X


def runManual():
	print("Manual controller starting")
	print("beware, no error checking involved")
	# open the serial port to the Arduino
	s1.flushInput()

	if s1.isOpen():
		s1.flush()
		cmd = ""
		
		# while we're still within our window of execution
		while (cmd != "999"):
			cmd = input('Enter Pi command:')
			
			# encode and send the command
			s1.write(cmd.encode())

			# receive and print the response
			response = s1.readline().decode("utf-8")

	# once finished
	setMotors(0,0)
	s1.close()

# main method
if __name__ == '__main__':

	#runController()
	runTracker()
	# runManual
