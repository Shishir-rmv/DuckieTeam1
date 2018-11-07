from multiprocessing import Process, Value
import serial, time

#global variables
rate = 9600
s1 = serial.Serial(port, rate, timeout=0)
motorL = 0 # motor speeds
motorR = 0
pingD = 0 #ping distance
L_ENC_DIST = 0 # change in wheel distances and associated angle change
R_ENC_DIST = 0
ENC_DELTA_THETA = 0
ARD_THETA = 0 #arduinos coordinates and direction
ARD_X = 0
ARD_Y = 0
THETA = 0
X = 0
Y = 0

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
	time.sleep(1)
	result = (s1.readline()).decode("utf-8")
	qe = result.split(' ')
	L_ENC_DIST = qe[0]
	R_ENC_DIST = qe[1]
	ENC_DELTA_THETA = qe[2]
	ARD_THETA = qe[3]					
	ARD_X = qe[4]
	ARD_Y = qe[5]

def updateArd():
	send = 'upd' + str(int(THETA))+str(int(X))+str(int(Y)) 
	s1.write(send.encode())

#get ping distance
def getPing():
	send = 'png'
	s1.write(send.encode())
	time.sleep(1)
	pingD = (s1.readline()).decode("utf-8")

#stop motors
def stop():
	send = 'stp'
	s1.write(send.encode())

#set motor speed
def setMotors(motorSpeedL, motorspeedR):
	send = 'mtr' + str(motorSpeedL) + str(motorspeedR)
	s1.write(send.encode())
	time.sleep(1)

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
