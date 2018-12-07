from multiprocessing import Process, Value
import serial, json, math, threading, time, pdb
from datetime import datetime
from enhancedduckvision import vision


#global variables
port = "/dev/ttyACM0"
rate = 115200
s1 = serial.Serial()
s1.port = port
s1.baudrate = rate
s1.timeout = 1


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

# note there are 3 flags in this script: go, move, and see
#   move is used to prevent the machine from moving before we want it to
#   see is used as a remote killswitch to kill the vision process from the controller process
#   running is used to kill the controller loop at any time we want to stop (could be a killswitch, or be graceful)
move = False

#below variables only needed if pi doing QE distance calculations
PPR = 32
#current approximation in mm, chassis constant
WHEEL_BASE = 137   
WHEEL_CIRCUMFERENCE = 219.9115


# to prevent the Pi from getting too far ahead of the arduino
def write(cmd):
    s1.write(cmd.encode())
    s1.flush()

def read():
    bytesToRead = s1.inWaiting()
    s1.read(bytesToRead)

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
    write('irr\n')
    r1 = s1.read(1)
    r2 = s1.read(1)
    l_enc = int.from_bytes(r1, byteorder = 'little', signed = False)
    r_enc = int.from_bytes(r2, byteorder = 'little', signed = False)
   
    # for debugging
    print("from encoder call: %d %d" % (l_enc, r_enc))
 
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


def comm_speed_test():
    i = 0 
    while(i<20):
        start_time = datetime.now()
        getEncoder()
        end_time = datetime.now()
        print("time for comm: %s" % str(end_time - start_time))
        i=i+1



#get ping distance
def getPing():
    write('png\n')
    response = s1.read(1)

    if (not response):
        print ("No result received from Arduino on getPing call")
    else:
        pingD = response


#set motor speed
def setMotors(motorSpeedL, motorSpeedR):
    send = 'mtr' +"0"+str(motorSpeedL) +"0"+ str(motorSpeedR)+"\n"
    write(send)
    #print(send)

    #update the global variables once they're written to serial
    motorL = motorSpeedL
    motorR = motorSpeedR


def runManual():
    print("Manual controller starting")
    print("beware, no error checking involved")
    # open the serial port to the Arduino
    s1.flushInput()

    if s1.isOpen():
        cmd = ""
        
        # while we're still within our window of execution
        while (cmd != "999"):
            cmd = input('Enter Pi cmd (\'999\' to quit):')
            
            # encode and send the command
            write(cmd+'\n')

            # receive and print the response
            response = s1.read(1)

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
        start = datetime.now()
        setMotors(200,200)
        
        # while we're still within our window of execution
        while (((datetime.now() - start).total_seconds() < stopAt) and (X<1200)):
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
    global move
    # use this to make it start moving when we want it to
    input("\nPress Enter to start\n")
    move = True
    print("Starter thread finished")

def serialReader():
    print("Starting serial thread")
    while (True):
        # read the "label" byte
        r1 = s1.read(1)
        # read the "data" byte
        r2 = s1.read(1)
        arg2 = int.from_bytes(r1, byteorder = 'little', signed = False)
        print("Label: %s, Data: %i" % (r1, arg2))
        # this is only for debugging
    print("Ending serial thread")

def visionController():
    global move
    # Define and split off the computer vision subprocess _________________________________
    # define doubles
    vOffset = Value('i', 0)

    
    # define boolean to act as an off switch
    see = Value('b', True)

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, see))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # split off the starter thread & serial reader threads so the machine can passively calibrate itself before we start
    starter_thread = threading.Thread(target=starter)
    starter_thread.start()

    # serial_thread = threading.Thread(target=serialReader)
    # serial_thread.start()

    # open the serial port to the Arduino & initialize
    s1.flushInput()
    response, state = "", "0"
    oldVal, now = -999, 0
    running, stateChange, odometry, flag = True, False, True, True

    # in mm/sec
    vRef = 0

    if s1.isOpen():
        s1.flush()


    # assume "stp" and "srt" and then "vrf" similar to "mtr". 
    # Pass "vrf" and two 4 character numbers afterwards. 
    # So cast the vOfset to a 4 character string, and pad up to 8 characters total with 0's.

    # this is the main logic loop where we put all our controlling equations/code
    try:
    #if(True):
        while (running):
            # only do this if we have changed state in our state machine?

            # if the starter thread has changed this global variable to allow movement
            if (move):
                if(flag):
                    # send initial calibration
                    # TODO: ASK WHAT'S A GOOD VREF.
                    print("SENDING: srt0000%s" % str(vRef).zfill(4))
                    write("srt0000%s\n" % str(vRef).zfill(4))
                    print("Finished writing start")
                    flag = False

                # check for visual error changes
                now = vOffset.value
                if (now != oldVal):
                    oldVal = now
                    # print("Camera:\t vOffset: %d" % (now))
                    print("SENDING: ver0000%s" % str(now).zfill(4))
                    write("ver0000%s\n" % str(now).zfill(4))
                    print("Finished writing update")
                
            
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False

    #stop vehicle process. Set motor speeds to 0, close down serial port, and kill vision thread.
    print("SENDING: stp")
    write("stp")
    s1.close()
    # once we're all done, send the kill switch to the inner vision loop and join the vision process
    see.value = False

    # join the starter and serial threads, kill vision
    starter_thread.join()
    print("Starter thread joined")
    # serial_thread.terminate()
    print("Serial thread joined")
    vision_process.terminate() 
    print("Vision process terminated")

def runController(mapNum):
    # Define and split off the computer vision subprocess _________________________________
    # define doubles
    vOffset = Value('i', 0)

    
    # define boolean to act as an off switch
    see = Value('b', True)

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, see))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # split off the starter thread so the machine can passively calibrate itself before we start
    starter_thread = threading.Thread(target=starter)
    starter_thread.start()

    # open the serial port to the Arduino & initialize
    s1.flushInput()
    response, state = "", "0"
    count, e, oldR, oldL = 0, 0, 0, 0
    running, stateChange, odometry = True, False, True

    if s1.isOpen():
        s1.flush()

    # open state machine data for reading
    with open("StateMachine/map%s.json" % mapNum, 'r') as f:
        machine = json.load(f)

    # this is the main logic loop where we put all our controlling equations/code
    try:
        while (running):
            # only do this if we have changed state in our state machine

            if (stateChange):
                state = machine[state]["next"]
                # set our controller mode for this state
                if(machine[state]["mode"] == "odometry"):
                    odometry = True
                    # communicate the mode down to the arduino
                    write("odo\n")
                else:
                    odometry = False
                    # communicate the mode down to the arduino
                    write("vis\n")

                # send speed calibration words down to arduino
                #TODO: calculate what value to start motors at
                #cmd = "cal"+"0"+str(motorStartL)+"0"+str(motorStartdR)+'\n'
                write(cmd)
                stateChange = False

            # for debugging:
            print("Camera:\t vOffset: %d" % (vOffset.value))
            # vDist = 0
            # vSlope = 0
            # print("Time elapsed: %d" % datetime.now().total_seconds())
            # print("IR:\tpos: (%f,%f), angle: %f" % (X, Y, THETA))
            # print("Camera:\t xDst: %d, slope:%d" % (vDist, vSlope))

            # if we're using an odometer-based controller
            if (odometry):
                #pdb.set_trace()
                if (machine[state]["act"] == "laneFollow"):
                    # if we've reached our stop condition (total distance forward)
                    if (Y >= machine[state]["stopCondition"]):
                        write("stp\n")
                        stateChange = True
                    else:
                        # query the QE
                        getEncoder()

                        # calculate the error based on the distance deltas and state machine specified curve constant
                        e = (R_ENC_DIST - oldR) - machine[state]["c"]*(L_ENC_DIST - oldL)

                        # send computed error down to the arduino
                        cmd = "err%s0000\n" % str(e).zfill(4)
                        write(cmd)
                        
                        # update x and y values to compute delta next iteration
                        oldR, oldL = X, Y

                # else, we're doing a blind turn
                else:
                    cmd = "trn%s0000\n" % str(machine[state]["c"]).zfill(4)
                    write(cmd)
                    


            # if we're using a visual controller
            else:
                i = 0; #delete this
                # check current visual positions
                # compute error of vehicle in lane

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
    print("Starter thread joined")
    vision_process.join() 
    print("Vision Process joined")


# this is the method that you will edit to do the hard-coded demos on Friday
# it reads in state information from a JSON file that you can change, 
# and will pass that down to the arduino to execute the 3 steps to stop on the red line
def hardCoded(mapNum):
    print("HardController starting")

    # split off the starter thread so the machine can passively calibrate itself before we start
    starter_thread = threading.Thread(target=starter)
    starter_thread.start()

    # open the serial port to the Arduino & initialize variables
    s1.flushInput()

    #don't know if we need these variables, depending on whether or not we'll have IR sensors working
    count, oldR, oldL = 0, 0, 0

    # make sure the serial connection is ready for communication
    if s1.isOpen():
        s1.flush()

    # open state machine data for reading
    with open("StateMachine/map%d.json" % mapNum, 'r') as f:
        machine = json.load(f)

    # this is the main logic loop where we put all our controlling equations/code
    try:
        # this while serves as a delay until the enter key is pressed. Then, the code below will execute
        while (not move):
            pass

        #TODO:
        #start in state 0
        state = "0"
        #go the first long stretch based on distance (or time?)
        print("Running State 1")


        #change to the next state
        state = "1"
        #navigate a turn
        print("Running State 2")


        #change to the next state
        state = "2"
        #go the rest of the straight distance
        print("Running State 3")


    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")

    #stop vehicle process: Set motor speeds to 0, and close down serial port.
    setMotors(0,0)
    s1.close()
    # once we're all done, send the kill switch to the inner vision loop and join the vision process
    starter_thread.join()
    print("Starter thread joined")

# main method
if __name__ == '__main__':
    s1.open()
    time.sleep(1)

    mode = int(input("Which mode would you like to run? " + 
        "\n 1 or 2: Hard-coded " + 
        "\n 3: Tracker " + 
        "\n 4: Manual " + 
        "\n 5: Controller " + 
        "\n 6: Basic Vision Controller " + 
        "\n 7: Comm speed test \n"))
    #run lane navigation
    if(mode == 1 or mode == 2):
        # runController(mode)
        hardCoded(mode)

    #run tracker mode
    elif(mode == 3):
        runTracker()

    #run manual mode
    elif(mode == 4):
        runManual()

    #run manual mode
    elif(mode == 5):
        # hardCoded(mode)
        runController(str(mode - 4))

    # run basic vision tester
    elif(mode == 6):
        visionController()

    elif(mode == 7):
        comm_speed_test()

    else:
        print("Wat")

    print("All done")
