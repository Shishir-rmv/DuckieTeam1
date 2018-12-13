from datetime import datetime
from multiprocessing import Process, Value

import json
import math
import networkx as nx
import serial
import threading
import time
from enhancedduckvision import vision

# global variables
port = "/dev/ttyACM0"
rate = 9600
s1 = serial.Serial()
s1.port = port
s1.baudrate = rate
s1.timeout = 1

goSerial = True
serial_msg_counter = 0
lastStart
DG = nx.DiGraph()

# print("Write Timeout: %d" % s1.write_timeout)
motorL = 0  # motor speeds
motorR = 0
# ping distance
pingD = 0
L_ENC_DIST = 0  # change in wheel distances and associated angle change
R_ENC_DIST = 0
SPEED = 0
l_enc = 0
r_enc = 0
last_time = datetime.now()

# vision variables
vOffset = Value('i', 0)
stopLine = Value('b', False)
greenLight = Value('b', False)
see = Value('b', True)

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

# below variables only needed if pi doing QE distance calculations
PPR = 32
# current approximation in mm, chassis constant
WHEEL_BASE = 137
WHEEL_CIRCUMFERENCE = 219.9115


# to prevent the Pi from getting too far ahead of the arduino
def write(cmd):
    print("SENDING: %s" % cmd)
    # print("Write - Before Encoding")
    encoded = cmd.encode()
    # print("Write - Before Writing")
    s1.write(encoded)
    # print("Write - After Writing")
    s1.flush()
    # print("Write - After Flush")


def read():
    bytesToRead = s1.inWaiting()
    s1.read(bytesToRead)


# function to grab encoder data
# have to decide if the arduino will return just increments or already calculate the distance per wheel and theta itself

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
    l_enc = int.from_bytes(r1, byteorder='little', signed=False)
    r_enc = int.from_bytes(r2, byteorder='little', signed=False)

    # for debugging
    print("from encoder call: %d %d" % (l_enc, r_enc))

    L_ENC_DIST = l_enc * WHEEL_CIRCUMFERENCE / PPR
    R_ENC_DIST = r_enc * WHEEL_CIRCUMFERENCE / PPR

    # update the change in avg position and current heading
    ENC_DELTA_X = (L_ENC_DIST + R_ENC_DIST) / 2
    ENC_DELTA_THETA = math.atan2((R_ENC_DIST - L_ENC_DIST) / 2, WHEEL_BASE / 2)

    # update overall global positioning
    THETA += ENC_DELTA_THETA
    X += ENC_DELTA_X
    Y += ENC_DELTA_X * math.sin(THETA)


def speed():
    global X
    global last_time
    global last_X
    time = datetime.now()

    SPEED = (X - last_X) / ((time - last_time).total_seconds())

    last_time = time
    last_X = X


# get ping distance
def getPing():
    write('png\n')
    response = s1.read(1)

    if (not response):
        print("No result received from Arduino on getPing call")
    else:
        pingD = response


# set motor speed
def setMotors(motorSpeedL, motorSpeedR):
    send = 'mtr' + "0" + str(motorSpeedL) + "0" + str(motorSpeedR) + "\n"
    write(send)
    # print(send)

    # update the global variables once they're written to serial
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
            write(cmd + '\n')

            # receive and print the response
            response = s1.read(1)

    # once finished
    setMotors(0, 0)
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
        setMotors(200, 200)

        # while we're still within our window of execution
        while (((datetime.now() - start).total_seconds() < stopAt) and (X < 1200)):
            # get data from arduino
            getEncoder()
            speed()

            # store it in the array
            records[float((datetime.now() - start).total_seconds())] = {"L_ENC_DIST": L_ENC_DIST,
                                                                        "R_ENC_DIST": R_ENC_DIST, "SPEED": SPEED,
                                                                        "ENC_DELTA_THETA": ENC_DELTA_THETA, "x": int(X),
                                                                        "Y": Y, "l_enc": l_enc, "r_enc": r_enc}

        # dump data to file
        print("dumping (%d) records to a JSON in the Logs folder" % len(records))
        with open('../Logs/tracer_%s.json' % str(datetime.now()).replace(" ", "_").replace(":", "."), 'w') as fp:
            json.dump(records, fp, indent=4)

    # once finished
    stop()
    s1.close()

def greenChanger():
    global greenLight

    time.sleep(1)
    greenLight.value = False

def starter(vRef):
    global move
    global lastStart
    global greenLight

    # use this to make it start moving when we want it to
    input("\nPress Enter to start\n")
    write("srt0000%s\n" % str(vRef).zfill(4))
    move = True
    lastStart = datetime.now()

    # temporary to allow us to test redline detection with enter key
    # time.sleep(3)
    # greenLight.value = False

    print("Starter thread finished")


def serialReader():
    global goSerial
    print("Starting serial thread")
    while (goSerial):
        if (s1.in_waiting):
            # read the "label" byte
            serialIn = s1.read(20)
            # read the "data" byte
            # r2 = s1.read(20)
            # arg2 = int.from_bytes(r2, byteorder = 'little', signed = False)
            print("From Arduino: " + serialIn.decode('utf-8'))
            # print("SERIAL: %s" % r1)
            # this is only for debugging
    print("Ending serial thread")

def makeGraph():
    global DG
    # initialize graph
    DG = nx.DiGraph()

    nodes = [1,2,3,4,5,6,7,8,9,10,11,12]

    # weights are rough estimates, change to introduce bias later
    edges = {"1,12," : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "S"}}},
            "1,4," : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "LS"}}}, #to prefer going straight
            "2,4," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"actions": "RS"}}},
            "2,8," : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "S"}}},
            "3,8," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"actions": "RS"}}},
            "3,12" : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "LS"}}},
            "4,7," : {"weight": 4, "attrs" : {"fast", False, "map":{"actions": "LSLS"}}},
            "4,11" : {"weight": 3, "attrs" : {"fast", False, "map":{"actions": "RSRS"}}},
            "5,3," : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "LS"}}},
            "5,7," : {"weight": 4, "attrs" : {"fast", False, "map":{"actions": "SLS"}}},
            "6,3," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"actions": "RS"}}},
            "6,11" : {"weight": 3.5, "attrs" : {"fast", False, "map":{"actions": "SRS"}}},
            "7,10" : {"weight": 8, "attrs" : {"fast", True, "map":{"actions": "SLFLS"}}},
            "7,1," : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "LS"}}},
            "8,10" : {"weight": 8, "attrs" : {"fast", True, "map":{"actions": "LSLFLS"}}},
            "8,6," : {"weight": 3, "attrs" : {"fast", False, "map":{"actions": "RSRS"}}},
            "9,1," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"actions": "RS"}}},
            "9,6," : {"weight": 3.5, "attrs" : {"fast", False, "map":{"actions": "SRS"}}},
            "10,2" : {"weight": 2, "attrs" : {"fast", False, "map":{"actions": "LS"}}},
            "10,5" : {"weight": 4, "attrs" : {"fast", False, "map":{"actions": "SLS"}}},
            "11,2" : {"weight": 1.5, "attrs" : {"fast", False, "map":{"actions": "RS"}}},
            "11,9" : {"weight": 7, "attrs" : {"fast", True, "map":{"actions": "SRFRS"}}},
            "12,9" : {"weight": 6.5, "attrs" : {"fast", True, "map":{"actions": "RSRFRS"}}},
            "12,5" : {"weight": 4, "attrs" : {"fast", False, "map":{"actions": "LSLS"}}}}

    wEdges = []
    for edge in edges:
        wEdges.append((edge[0], edge[1], edge[2]))

    xyts = {0: {'X': 0, 'Y': 0, 'T': 0}, 
            1: {'X': 0, 'Y': 0, 'T': 0}, 
            2: {'X': 0, 'Y': 0, 'T': 0}, 
            3: {'X': 0, 'Y': 0, 'T': 0}, 
            4: {'X': 0, 'Y': 0, 'T': 0}, 
            5: {'X': 0, 'Y': 0, 'T': 0}, 
            6: {'X': 0, 'Y': 0, 'T': 0}, 
            7: {'X': 0, 'Y': 0, 'T': 0}, 
            8: {'X': 0, 'Y': 0, 'T': 0}, 
            9: {'X': 0, 'Y': 0, 'T': 0}, 
            10: {'X': 0, 'Y': 0, 'T': 0}, 
            11: {'X': 0, 'Y': 0, 'T': 0} 
            12: {'X': 0, 'Y': 0, 'T': 0}}
    # nx.set_node_attributes(G, attrs)


    # construct graph
    DG.add_nodes_from(nodes)
    DG.add_weighted_edges_from(wEdges)


def visionController():
    global move
    global goSerial
    global serial_msg_counter
    global lastStart

    # vision variables to share between processes
    global vOffset
    global stopLine
    global greenLight
    global see

    lastStart = datetime.now()
    starterThreads = []
    start = time.time()
    # Define and split off the computer vision subprocess _________________________________
    # define doubles

    stopped = False

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, see, stopLine, greenLight))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # in mm/sec
    vRef = 30

    # split off the starter thread & serial reader threads so the machine can passively calibrate itself before we start
    starterThreads.append(threading.Thread(target=starter, args=(vRef,)))
    starterThreads[0].start()

    serial_thread = threading.Thread(target=serialReader)
    serial_thread.start()

    # open the serial port to the Arduino & initialize
    s1.flushInput()
    response, state = "", "0"
    oldVal, now = -999, 0
    running, flag = True, True

    if s1.isOpen():
        s1.flush()

    # assume "stp" and "srt" and then "vrf" similar to "mtr".
    # Pass "vrf" and two 4 character numbers afterwards. 
    # So cast the vOfset to a 4 character string, and pad up to 8 characters total with 0's.

    # this is the main logic loop where we put all our controlling equations/code
    try:
        # if(True):
        while (running):
            # only do this if we have changed state in our state machine?
            # if the starter thread has changed this global variable to allow movement
            if (move):
                if (greenLight.value and not stopped):
                    print("LOOPING HERE")
                # if(flag):
                #     # send initial calibration
                #     write("srt0000%s\n" % str(vRef).zfill(4))
                #     # print("Finished writing start")
                #     flag = False

                if (stopLine.value and not stopped and (datetime.now() - lastStart).seconds > 2):
                    print("Stopper middle: stp")
                    write("stp")
                    stopped = True
                    # spawn off a starter thread to only let the bot move again if we want it to
                    # sThread = threading.Thread(target=starter, args=(vRef,))
                    # starterThreads.append(sThread)
                    # move = False
                    # sThread.start()

                elif (stopped and greenLight.value):
                    stopped = False
                    lastStart = datetime.now()
                    print("Its green, Starting again")
                    print("SENDING: srt")
                    write("srt0000%s\n" % str(vRef).zfill(4))
                    greenLight.value = False

                else:
                    # check for visual error changes
                    now = vOffset.value
                    if (now != oldVal):
                        oldVal = now
                        print("New vError:\t vOffset: %d" % (now))
                        # print("SENDING: ver0000%s" % str(now).zfill(4))
                        serial_msg_counter += 1
                        # print("SENT %d Messages to Arduino" % serial_msg_counter)
                        end = time.time()
                        # print("%d seconds elapsed" % (end - start))
                        write("ver0000%s\n" % str(now).zfill(4))
                        # print("inWaiting: %i, outWaiting %i" % (s1.in_waiting, s1.out_waiting))
                        # print("Finished writing update"


    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False

    # stop vehicle process. Set motor speeds to 0, close down serial port, and kill vision thread.
    # print("SENDING: stp")
    write("stp")
    s1.close()
    # once we're all done, send the kill switch to the inner vision loop and join the vision process
    see.value = False

    # join the starter and serial threads, kill vision
    for starterThread in starterThreads:
        starterThread.join()
    print("Starter threads joined")
    goSerial = False
    serial_thread.join()
    print("Serial thread joined")
    vision_process.terminate()
    print("Vision process terminated")


# this will visually navigate until the stop condition is reached
def vNav():
    global vOffset
    # global oldValue
    global stopLine
    
    oldVal = 0
    stopped = False
    
    while (not stopped):
        if (stopLine.value and not stopped and (datetime.now() - lastStart).seconds > 2):
            print("Red line detected by vNav()")
            write("stp")
            stopped = True

        else:
            # check for visual error changes
            now = vOffset.value
            if (now != oldVal):
                oldVal = now
                write("ver0000%s\n" % str(now).zfill(4))


def runController():
    global move
    # Define and split off the computer vision subprocess _________________________________
    # vision variables to share between processes
    global vOffset
    global stopLine
    global greenLight
    global see

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, see, stopLine, greenLight))
    vision_process.start()
    # _____________________________________________________________________________________
    print("PyController starting")

    # to be given to us by instructors before the demo
    # path = [1, 5, 7, 2, 9, 3, 12, 6, 8, 10, 1]
    # a test path
    path = [10,7]
    pathCounter = 0

    vRef = 30
    fastVRef = 60

    # big left turn
    bigRadius = 0.45
    # small right turn
    smallRadius = 0.2

    # read in the state machine graph
    with open("../peripherals/graph.json", 'r') as f:
        read = json.load(f)

    # re-convert to graph
    DG = nx.node_link_graph(read, directed=True, multigraph=False, attrs=None)

    # split off the starter thread so the machine can passively calibrate itself before we start
    starter_thread = threading.Thread(target=starter, args=(vRef))
    starter_thread.start()

    serial_thread = threading.Thread(target=serialReader)
    serial_thread.start()

    # open the serial port to the Arduino & initialize
    s1.flushInput()

    running = True

    if s1.isOpen():
        s1.flush()

    # open state machine data for reading
    with open("StateMachine/map%s.json" % mapNum, 'r') as f:
        machine = json.load(f)

    # TODO: Calibrating
    # calibrate what X and Y we are at according to our initial state
    # print("calibrating position now")
    # write("cal%s%s" % (stateX, stateY))

    # this is the main logic loop where we put all our controlling equations/code
    try:
        # calibrate the robot
        write("cal%s%s\n" % str(vRef).zfill(4), str(vRef).zfill(4))

        # wait until we want the robot to move
        while (not move):
            pass

        # loop overall segments in our given route
        for segment in range(len(path) - 1):
            # debugging
            print("About to navigate %s to %s" % (path[segment], path[segment + 1]))

            # compute the path from the segment start state to its finish state
            route = nx.dijkstra_path(DG, path[segment], path[segment + 1])
            # navigate the current segment's route

            # debugging
            print("Path plan is: %s" % str(route))
            for currentState in range(len(route) - 1):
                # by performing all of the actions in the route
                # when the last action is completed, the next state will happen in the parent for loop
                actionMap = edges[str(currentState) + "," + str(currentState + 1)]["attrs"]["map"]

                # wait until we see a green light to begin our action sequence
                while (not greenLight.value):
                    time.sleep(.2)

                for action in range(len(actionMap)):
                    # go straight
                    if (actionMap[action] == "S" or actionMap[action] == "F"):
                        # using vision, start moving. Args: dist, initial vRef
                        write("srt0000%s\n" % str(vRef).zfill(4))

                        # navigate visually until the stop condition
                        vNav()

                        # if we're on the last action
                        if (action == len(actionMap) - 1):
                            vNav(vOffset, stopLine)

                    elif (actionMap[action] == "R"):
                        # blind turn
                        write("rtn%s0045" % str(smallRadius).zfill(4))
                        # wait for arduino to respond?
                    elif (actionMap[action] == "L"):
                        # blind turn
                        write("ltn%s0045" % str(bigRadius).zfill(4))
                        # wait for arduino to respond?

    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False

    # output = Kp*(goal-X)-Kd*SPEED

    # stop vehicle process. Set motor speeds to 0, close down serial port, and kill vision thread.
    setMotors(0, 0)
    s1.close()
    # once we're all done, send the kill switch to the inner vision loop and join the vision process
    see.value = False
    goSerial = False
    starter_thread.join()
    print("Starter thread joined")
    serial_thread.join()
    print("Serial thread joined")
    vision_process.join()
    print("Vision Process joined")

def turn(rTurn, radius):
    if (rTurn):
        write("rtn%s0045" % str(radius).zfill(4))
    else:
    write("ltn%s0045" % str(radius).zfill(4))

def smallTest():
    global move
    global goSerial
    global serial_msg_counter
    global lastStart

    # vision variables to share between processes
    global vOffset
    global stopLine
    global greenLight
    global see

    lastStart = datetime.now()
    starterThreads = []
    start = time.time()
    # Define and split off the computer vision subprocess _________________________________
    # define doubles

    stopped = False

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, see, stopLine, greenLight))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # in mm/sec
    vRef = 30

    # split off the starter thread & serial reader threads so the machine can passively calibrate itself before we start
    starterThreads.append(threading.Thread(target=starter, args=(vRef,)))
    starterThreads[0].start()

    serial_thread = threading.Thread(target=serialReader)
    serial_thread.start()

    # rtn00.20045

    # open the serial port to the Arduino & initialize
    s1.flushInput()
    response, state = "", "0"
    oldVal, now = -999, 0
    running, flag = True, True

    if s1.isOpen():
        s1.flush()

    # assume "stp" and "srt" and then "vrf" similar to "mtr".
    # Pass "vrf" and two 4 character numbers afterwards. 
    # So cast the vOfset to a 4 character string, and pad up to 8 characters total with 0's.

    # this is the main logic loop where we put all our controlling equations/code
    try:
        # wait until we want the robot to move
        print("CONTROLLER: waiting for user to permit movement")
        while (not move):
            pass

        # begin visual navigation. This will stop at a red line
        print("CONTROLLER: Starting vNav()")
        vnav()

        # wait until we see a green light to go again
        print("CONTROLLER: waiting until we see a green light")
        while (not greenLight.value):
            pass

        # spawn a thread to switch greenLight off 1 second from now
        print("CONTROLLER: spawning greenLight changer thread")
        greenChanger.append(threading.Thread(target=greenChanger))
        greenChanger.start()

        # change turn radius here
        print("CONTROLLER: performing turn")
        radius = .2
        # args: [rTurn (boolean, if this is a right turn. False = left turn)], [radius of turn]
        turn(True, .2)

        # continue visually navigating afterwards (you'll probably want to kill it gracefully eventually)
        print("CONTROLLER: Starting vNav()")
        vNav()

    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False

    # stop vehicle process. Set motor speeds to 0, close down serial port, and kill vision thread.
    # print("SENDING: stp")
    write("stp")
    s1.close()
    # once we're all done, send the kill switch to the inner vision loop and join the vision process
    see.value = False

    # join the starter and serial threads, kill vision
    for starterThread in starterThreads:
        starterThread.join()
    print("Starter threads joined")
    goSerial = False
    serial_thread.join()
    print("Serial thread joined")
    greenChanger.join()
    print("Green thread joined")
    vision_process.terminate()
    print("Vision process terminated")


# main method
if __name__ == '__main__':
    s1.open()
    time.sleep(1)

    mode = int(input("Which mode would you like to run? " +
                     "\n 1: Main Controller " +
                     "\n 2: Vision Controller " +
                     "\n 3: Manual Mode " +
                     "\n 4: test Mode\n"))
    # run lane navigation
    if (mode == 1):
        runController()

    # run tracker mode
    elif (mode == 2):
        visionController()

    # run manual mode
    elif (mode == 3):
        runManual()

    # run test
    elif (mode == 4):
        smallTest()

    else:
        print("Wat")

    print("All done")
