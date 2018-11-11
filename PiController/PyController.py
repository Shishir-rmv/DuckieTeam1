from multiprocessing import Process, Value
import serial, json, math, threading
from datetime import datetime
from PiController.duckvision import vision



#global variables
port = "/dev/ttyACM0"
rate = 2000000
s1 = serial.Serial(port, rate, timeout=10)
motorL = 0  # motor speeds
motorR = 0
#ping distance
pingD = 0
L_ENC_DIST = 0  # change in wheel distances and associated angle change
R_ENC_DIST = 0
SPEED = 0
l_enc = 0
r_enc = 0
last_time = datetime.now()


ENC_DELTA_THETA = 0
ENC_DELTA_X = 0

THETA = 0
X = 0
last_X = 0
Y = 0

# note there are 3 flags in this script: go, drive, and see
#   drive is used to prevent the machine from moving before we want it to
#   see is used as a remote killswitch to kill the vision process from the controller process
#   running is used to kill the controller loop at any time we want to stop (could be a killswitch, or be graceful)
drive = False

#below variables only needed if pi doing QE distance calculations
PPR = 32
#current approximation in mm, chassis constant
WHEEL_BASE = 137   
WHEEL_CIRCUMFERENCE = 219.9115


#function to grab encoder data 
#have to decide if the arduino will return just increments or already calculate the distance per wheel and theta itself

def getEncoder():
    global THETA
    global X
    global Y
    global WHEEL_BASE
    global WHEEL_CIRCUMFERENCE
    global PPR
    global L_ENC_DIST
    global R_ENC_DIST
    global l_enc
    global r_enc
    send = 'irr'
    s1.write(send.encode())
    result = s1.readline().decode("utf-8")

    if (not result):
        print ("No result received from Arduino on getEncoder call")
    else:
        #result = (s1.readline()).decode("utf-8")
        qe = result.split(',')
        # for debugging
        print("from encoder call: %s" % str(result))
        if(len(qe) >= 2):
            l_enc = int(qe[0])
            r_enc = int(qe[1])
        L_ENC_DIST = l_enc * WHEEL_CIRCUMFERENCE/PPR
        R_ENC_DIST = r_enc * WHEEL_CIRCUMFERENCE/PPR

        #update the change in avg position and current heading
        ENC_DELTA_X = (L_ENC_DIST + R_ENC_DIST)/2
        ENC_DELTA_THETA = math.atan2((R_ENC_DIST-L_ENC_DIST)/2, WHEEL_BASE/2)

        #update overall global positioning
        THETA += ENC_DELTA_THETA
        X += ENC_DELTA_X
        Y += ENC_DELTA_X*math.sin(THETA)


def speed():
    global X
    global last_time
    global last_X
    time = datetime.now()
    
    SPEED = (X - last_X)/((time - last_time).total_seconds())
    
    last_time = time
    last_X = X


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
def setMotors(motorSpeedL, motorSpeedR):
    send = 'mtr' +"0"+str(motorSpeedL) +"0"+ str(motorSpeedR)
    s1.write(send.encode())
    print(send)

    #update the global variables once they're written to serial
    motorL = motorSpeedL
    motorR = motorSpeedR


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
            cmd = input('Enter Pi cmd (\'999\' to quit):')
            
            # encode and send the command
            s1.write(cmd.encode())

            # receive and print the response
            response = s1.readline().decode("utf-8")

    # once finished
    setMotors(0,0)
    s1.close()


def runTracker():
    global X
    global Y
    global ENC_DELTA_THETA
    global SPEED
    global L_ENC_DIST
    global R_ENC_DIST
    global l_enc
    global r_enc
    print("PyTracer starting")
    # open the serial port to the Arduino
    s1.flushInput()

    # Do controller stuff
    response = ""

    count = 0
    running = True
    stopAt = 120
    records = {}

    if s1.isOpen():
        s1.flush()
        start = datetime.now()
        setMotors(400,400)
        
        # while we're still within our window of execution
        while (((datetime.now() - start).total_seconds() < stopAt) and (X<100)):
            # get data from arduino
            getEncoder()
            speed()

            # store it in the array
            records[float((datetime.now() - start).total_seconds())] = {"L_ENC_DIST" : L_ENC_DIST, "R_ENC_DIST" : R_ENC_DIST, "SPEED" : SPEED,
                                "ENC_DELTA_THETA" : ENC_DELTA_THETA, "x" : int(X), "Y" : Y, "l_enc" : l_enc, "r_enc" : r_enc}

        #dump data to file
        print("dumping (%d) records to a JSON in the Logs folder" % len(records))
        with open('../Logs/tracer_%s.json' % str(datetime.now()).replace(" ", "_").replace(":","."), 'w') as fp:
            json.dump(records, fp, indent=4)

    # once finished
    stop()
    s1.close()


def starter():
    # use this to make it start moving when we want it to
    input("Press Enter to start")
    move = True


def runController(mapNum):
    # Define and split off the computer vision subprocess _________________________________
    # define doubles
    x1 = Value('d', -9.99)
    y1 = Value('d', -9.99)
    x2 = Value('d', -9.99)
    y2 = Value('d', -9.99)
    slope = Value('d', 0.0)
    
    # define boolean to act as an off switch
    see = ('b', True)

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(see, x1, x2, y1, y2, slope))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # split off the starter thread so the machine can passively calibrate itself before we start
    starter_thread = threading.Thread(target=starter)
    starter_thread.start()

    # open the serial port to the Arduino & initialize
    s1.flushInput()
    response = ""
    count = 0
    running = True

    if s1.isOpen():
        s1.flush()

    # open state machine data for reading
    # states = json.load("StateMachine/map%d" % mapNum)
        
    # this is the main logic loop where we put all our controlling equations/code
    try:
        while (running):

            # for debugging:
            vDist = 0
            vSlope = 0
            print("Time elapsed: %d" % datetime.now().total_seconds())
            print("IR:\tpos: (%f,%f), angle: %f" % (X, Y, THETA))
            print("Camera:\t xDst: %d, slope:%d" % (vDist, vSlope))

            #check distance to lines on either side & angle in lane
            #compute wheel speed adjustments based off of current speed and required corrections
            #set wheels to corrected speed
            #consider sleeping until a timeDelta has passed?
            # compare state machine data to current data and see if we need to initiate a turn?
            
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False

    #output = Kp*(goal-X)-Kd*SPEED

    #stop vehicle process. Set motor speeds to 0, close down serial port, and kill vision thread.
    setMotors(0,0)
    s1.close()
    # once we're all done, send the kill switch to the inner vision loop and join the vision process
    see.value = False
    starter_thread.join()
    vision_process.join()


# main method
if __name__ == '__main__':
    mode = int(input("Which mode would you like to run? \n 1 or 2: Controller \n 3: Tracker \n 4: Manual"))
    #run lane navigation
    if(mode == 1 or mode == 2):
        runController(mode)

    #run tracker mode
    if(mode == 3):
        runTracker()

    #run manual mode
    if(mode == 4):
        runManual()

    else:
        print("Wat")

    print("All done")
