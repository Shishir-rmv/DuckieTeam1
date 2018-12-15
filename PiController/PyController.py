import json, math, serial, threading, time, pdb
from datetime import datetime
from multiprocessing import Process, Value
import networkx as nx
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

lastStart = datetime.now()
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
vOffsetOld = Value('i', 0)
stopLine = Value('b', False)
greenLight = Value('b', False)
see = Value('b', True)

ENC_DELTA_THETA = 0
ENC_DELTA_X = 0

edges = {}
xyts = {}

serialD = False

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
    print("sending: %s" % cmd)
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

def makeGraph():
    global DG
    global edges
    global xyts
    # initialize graph
    DG = nx.DiGraph()

    nodes = [1,2,3,4,5,6,7,8,9,10,11,12]

    # weights are rough estimates, change to introduce bias later
    edges = {"1,12": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "BS"}}},
             "1,4": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "LS"}}},  # to prefer going straight
             "2,4": {"weight": 1.5, "attrs": {"fast": False, "map": {"actions": "RS"}}},
             "2,8": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "BS"}}},
             "3,8": {"weight": 1.5, "attrs": {"fast": False, "map": {"actions": "RS"}}},
             "3,12": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "LS"}}},
             "4,7": {"weight": 4, "attrs": {"fast": False, "map": {"actions": "LSLS"}}},
             "4,11": {"weight": 3, "attrs": {"fast": False, "map": {"actions": "RSRS"}}},
             "5,3": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "LS"}}},
             "5,7": {"weight": 4, "attrs": {"fast": False, "map": {"actions": "BSLS"}}},
             "6,3": {"weight": 1.5, "attrs": {"fast": False, "map": {"actions": "RS"}}},
             "6,11": {"weight": 3.5, "attrs": {"fast": False, "map": {"actions": "BRS"}}},
             "7,10": {"weight": 8, "attrs": {"fast": True, "map": {"actions": "S"}}},
             "7,1": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "LS"}}},
             "8,10": {"weight": 8, "attrs": {"fast": True, "map": {"actions": "LSLFLS"}}},
             "8,6": {"weight": 3, "attrs": {"fast": False, "map": {"actions": "RSRS"}}},
             "9,1": {"weight": 1.5, "attrs": {"fast": False, "map": {"actions": "RS"}}},
             "9,6": {"weight": 3.5, "attrs": {"fast": False, "map": {"actions": "SRS"}}},
             "10,2": {"weight": 2, "attrs": {"fast": False, "map": {"actions": "LS"}}},
             "10,5": {"weight": 4, "attrs": {"fast": False, "map": {"actions": "BSLS"}}},
             "11,2": {"weight": 1.5, "attrs": {"fast": False, "map": {"actions": "RS"}}},
             "11,9": {"weight": 7, "attrs": {"fast": True, "map": {"actions": "BSRFRS"}}},
             "12,9": {"weight": 6.5, "attrs": {"fast": True, "map": {"actions": "RSRFRS"}}},
             "12,5": {"weight": 4, "attrs": {"fast": False, "map": {"actions": "LSLS"}}}
             }
    
    wEdges = []
    for edge, val in edges.items():
        wEdges.append((int(edge.split(',')[0]), int(edge.split(',')[1]), val["weight"]))

    # TODO: assign radii based upon the constants that Johnathan and Bhavesh give me
    xyts = {1: {'X': 105.5, 'Y': 133.5, 'T': 0, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            2: {'X': 185, 'Y': 156.5, 'T': 180, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 60}, 
            3: {'X': 133, 'Y': 186, 'T': 270, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            4: {'X': 156, 'Y': 221, 'T': 90, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            5: {'X': 185, 'Y': 274, 'T': 180, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            6: {'X': 105.5, 'Y': 251, 'T': 0, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            7: {'X': 15, 'Y': 186, 'T': 270, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            8: {'X': 68, 'Y': 156.5, 'T': 180, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            9: {'X': 38, 'Y': 103, 'T': 270, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': }, 
            10: {'X': 274, 'Y': 103, 'T': 90, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            11: {'X': 251, 'Y': 186, 'T': 270, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}, 
            12: {'X': 221, 'Y': 1335, 'T': 0, 'radiusR': .2, 'speedR': 45, 'radiusL': .45, 'speedL': 55}}
    nx.set_node_attributes(DG, xyts)


    # construct graph
    DG.add_nodes_from(nodes)
    DG.add_weighted_edges_from(wEdges)


# set motor speed
def setMotors(motorSpeedL, motorSpeedR):
    # send = 'mtr' + "0" + str(motorSpeedL) + "0" + str(motorSpeedR) + "\n"
    # write(send)
    # # print(send)

    # # update the global variables once they're written to serial
    # motorL = motorSpeedL
    # motorR = motorSpeedR

    pass


# this will visually navigate until the stop condition is reached
def vNav(lookingForD):
    global vOffset
    global vOffsetOld
    global stopLine
    global lastStart
    global serialD
    
    lastStart = datetime.now()

    stopped = False
    
    while (not stopped):
        if (lookingForD):
            if (serialD):
                serialD = False
                lookingForD = False
                stopLine.value = False
        
        # print("vNav() not looking for D, loop is: " + str(stopLine.value and not stopped and (datetime.now() - lastStart).seconds > 1))
        else:
            if (stopLine.value and not stopped and (datetime.now() - lastStart).seconds > 5):
                # print("vNav() not looking for D should stop")
                print("Red line detected by vNav()")
                write("stp")
                stopped = True

        if (not stopped):
            # check for visual error changes
            # print("stopline.value: %s, stopped: %s, timing: %s" % (stopLine.value, stopped, (datetime.now() - lastStart).seconds > 1))
            old = vOffsetOld.value
            now = vOffset.value

            if (now != old):
                old = now
                write("ver0000%s\n" % str(now).zfill(4))


def turn(rTurn, radius, speed):
    global serialD
    global stopLine

    if (rTurn):
        write("rtn%s%s" % (str(radius).zfill(4), str(speed).zfill(4)))
    else:
        write("ltn%s%s" % (str(radius).zfill(4), str(speed).zfill(4)))


def calibrate(node):
    global DG
    write("cal%s0000\n" % str(DG.nodes['X']).zfill(4))
    write("car%s0000\n" % str(DG.nodes['Y']).zfill(4))
    write("cat%s0000\n" % str(DG.nodes['T']).zfill(4))


def greenThread():
    global greenLight

    time.sleep(1)
    greenLight.value = False


def starter(vRef):
    global move
    global lastStart
    global greenLight

    # use this to make it start moving when we want it to
    input("\nPress Enter to start\n")
    # write("srt0000%s\n" % str(vRef).zfill(4))
    move = True
    lastStart = datetime.now()

    # temporary to allow us to test redline detection with enter key
    # time.sleep(3)
    # greenLight.value = False

    print("Starter thread finished")


def serialReader(s1):
    global goSerial
    global serialD

    print("Starting serial thread")
    while (goSerial):
        if (s1.in_waiting):
            serialIn = str(s1.read(20).decode('utf-8'))
            print("From Arduino: %s" + serialIn)
            print("type is: " + str(type(serialIn)))
            print(serialIn)

            if ("D" in serialIn):
                print("THERES A D")
                serialD = True

    print("Ending serial thread")


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


def visionController():
    global move
    global goSerial
    global serial_msg_counter
    global lastStart
    global s1

    # vision variables to share between processes
    global vOffset
    global vOffsetOld
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
    vision_process = Process(target=vision, args=(vOffset, vOffsetOld, see, stopLine, greenLight))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # in mm/sec
    vRef = 30

    # split off the starter thread & serial reader threads so the machine can passively calibrate itself before we start
    starterThreads.append(threading.Thread(target=starter, args=(vRef,)))
    starterThreads[0].start()

    serial_thread = threading.Thread(target=serialReader, args=(s1,))
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

        while (not move):
            pass

        write("srt0000%s\n" % str(vRef).zfill(4))

        while (running):
            # only do this if we have changed state in our state machine?
            # if the starter thread has changed this global variable to allow movement
            if (move):
                #if (greenLight.value and not stopped):
                #    print("LOOPING HERE")
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


def runController():
    global move
    global goSerial
    global lastStart
    global s1
    global serialD
    global DG
    global edges
    global xyts

    # Define and split off the computer vision subprocess _________________________________
    # vision variables to share between processes
    global vOffset
    global vOffsetOld
    global stopLine
    global greenLight
    global see

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, vOffsetOld, see, stopLine, greenLight))
    vision_process.start()
    # _____________________________________________________________________________________
    print("PyController starting")

    # to be given to us by instructors before the demo
    # path = [1, 5, 7, 2, 9, 3, 12, 6, 8, 10, 1]
    # a test path
    # path = [9,4]
    path = [8, 6, 3]

    vRef = 30
    fastVRef = 60

    # big left turn
    bigRadius = 0.45
    # small right turn
    smallRadius = 0.2

    greenChangers = []

    # read in the state machine graph
    # with open("../peripherals/graph.json", 'r') as f:
    #     read = json.load(f)

    # initialize the graph
    makeGraph()


    # split off the starter thread so the machine can passively calibrate itself before we start
    starter_thread = threading.Thread(target=starter, args=(vRef,))
    starter_thread.start()

    serial_thread = threading.Thread(target=serialReader, args=(s1,))
    serial_thread.start()

    # open the serial port to the Arduino & initialize
    s1.flushInput()

    running = True
    controllerCounter = 0

    if s1.isOpen():
        s1.flush()

    # this is the main logic loop where we put all our controlling equations/code
    try:
        # calibrate the robot
        # calibrate(path[0])

        # wait until we want the robot to move
        print("CONTROLLER %d: waiting for user to permit movement" % controllerCounter)
        controllerCounter += 1
        while (not move):
            pass

        # wait until we see a green light to begin our action sequence
        # print("CONTROLLER %d: waiting until we see a green light"  % controllerCounter)
        # while (not greenLight.value):
        #     pass

        # spawn a thread to switch greenLight off 1 second from now
        # print("CONTROLLER %d: spawning greenLight changer thread"  % controllerCounter)
        # controllerCounter += 1
        # greenChangers.append(threading.Thread(target=greenThread))
        # greenChangers[-1].start()

        # loop overall segments in our given route
        for segment in range(len(path) - 1):
            # debugging
            print("CONTROLLER %d: About to navigate %s to %s" % (controllerCounter, path[segment], path[segment + 1]))
            controllerCounter += 1

            # compute the path from the segment start state to its finish state
            route = nx.dijkstra_path(DG, path[segment], path[segment + 1])
            # navigate the current segment's route

            # debugging
            print("CONTROLLER %d: Path plan is: %s" % (controllerCounter, str(route)))
            controllerCounter += 1
            for currentState in range(len(route) - 1):
                # by performing all of the actions in the route
                # when the last action is completed, the next state will happen in the parent for loop
                # pdb.set_trace()
                actionMap = edges[str(route[currentState]) + "," + str(route[currentState+1])]["attrs"]["map"]['actions']

                # debugging
                print("CONTROLLER %d: Action map: %s" % (controllerCounter, str(actionMap)))
                controllerCounter += 1

                for action in range(len(actionMap)):
                    # debugging
                    print("CONTROLLER %d: current action is: %s" % (controllerCounter, str(actionMap[action])))
                    controllerCounter += 1

                    # go straight
                    if (actionMap[action] == "S" or actionMap[action] == "F"):
                        write("srt0000%s\n" % str(vRef).zfill(4))
                        lastStart = datetime.now()

                        # using vision, start moving. Args: initial vRef
                        # only sent srt's for the first action
                        if (action != 0):
                            if (actionMap[action-1] == 'R' or actionMap[action-1] == 'L' or actionMap[action-1] == 'B'):
                                # navigate visually until the stop condition
                                print("CONTROLLER %d: Starting vNav() with waitForD"  % controllerCounter)
                                vNav(True)

                        else:
                            print("CONTROLLER %d: Starting vNav()"  % controllerCounter)
                            vNav(False)

                        # wait until we see a green light to go again
                        print("CONTROLLER %d: waiting until we see a green light"  % controllerCounter)
                        controllerCounter += 1
                        while (not greenLight.value):
                            pass
                        lastStart = datetime.now()

                        # spawn a thread to switch greenLight off 1 second from now
                        print("CONTROLLER %d: spawning greenLight changer thread"  % controllerCounter)
                        controllerCounter += 1
                        greenChangers.append(threading.Thread(target=greenThread))
                        greenChangers[-1].start()

                    elif (actionMap[action] == "R"):
                        # if (action == 0):
                        print("CONTROLLER %d: Writing SRT"  % controllerCounter)
                        controllerCounter += 1
                        write("srt0000%s\n" % str(vRef).zfill(4))

                        print("CONTROLLER %d: performing blind right turn"  % controllerCounter)
                        controllerCounter += 1
                        # blind turn
                        # pdb.set_trace()
                        turn(True, xyts[route[currentState]]['radiusR'], xyts[route[currentState]]['speedR'])

                    elif (actionMap[action] == "L"):
                        # if (action == 0):
                        print("CONTROLLER %d: Writing SRT"  % controllerCounter)
                        controllerCounter += 1
                        write("srt0000%s\n" % str(vRef).zfill(4))
                        print("CONTROLLER %d: performing blind left turn"  % controllerCounter)
                        controllerCounter += 1
                        # blind turn
                        turn(False, xyts[route[currentState]]['radiusL'], xyts[route[currentState]]['speedL'])

                    elif(actionMap[action] == "B"):
                        # if (action == 0):
                        print("CONTROLLER %d: Writing SRT"  % controllerCounter)
                        controllerCounter += 1
                        write("srt0000%s\n" % str(vRef).zfill(4))
                        # blind straight (can use the turning code with no radius)
                        # since we know this will be the first call after an intersection that we want to go straight through
                        write("ltn0001%s\n" % str(vRef).zfill(4))

                        print("CONTROLLER %d: Blind Straight. Waiting for the arduino to transmit the D"  % controllerCounter)
                        controllerCounter += 1

                        # wait for the blind turn to finish
                        while (not serialD):
                            pass

                        serialD = False
                        stopLine.value = False

                    print("CONTROLLER %d: Action is finished"  % controllerCounter)
                    controllerCounter += 1
                print("CONTROLLER %d: Segment is finished"  % controllerCounter)
                controllerCounter += 1
            print("CONTROLLER %d: Plan is finished"  % controllerCounter)
            controllerCounter += 1

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
    # join all green light threads
    for greenie in greenChangers:
        greenie.join()
    starter_thread.join()
    print("Starter thread joined")
    serial_thread.join()
    print("Serial thread joined")
    vision_process.join()
    print("Vision Process joined")


def smallTest():
    global move
    global goSerial
    global lastStart
    global s1
    global serialD

    # vision variables to share between processes
    global vOffset
    global vOffsetOld
    global stopLine
    global greenLight
    global see

    lastStart = datetime.now()
    starterThreads = []
    greenChangers = []
    start = time.time()
    # Define and split off the computer vision subprocess _________________________________
    # define doubles

    stopped = False

    # define and start the computer vision process
    vision_process = Process(target=vision, args=(vOffset, vOffsetOld, see, stopLine, greenLight))
    vision_process.start()
    # _____________________________________________________________________________________

    print("PyController starting")

    # in mm/sec
    vRef = 30

    # split off the starter thread & serial reader threads so the machine can passively calibrate itself before we start
    starterThreads.append(threading.Thread(target=starter, args=(vRef,)))
    starterThreads[0].start()

    serial_thread = threading.Thread(target=serialReader, args=(s1,))
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
        print("CONTROLLER 1: waiting for user to permit movement")
        while (not move):
            pass

        # begin visual navigation. This will stop at a red line
        print("CONTROLLER 2: Starting vNav()")
        vNav(False)

        # wait until we see a green light to go again
        print("CONTROLLER 3: waiting until we see a green light")
        while (not greenLight.value):
            pass
        lastStart = datetime.now()

        # spawn a thread to switch greenLight off 1 second from now
        print("CONTROLLER 4: spawning greenLight changer thread")
        greenChangers.append(threading.Thread(target=greenThread))
        greenChangers[-1].start()

        # change turn radius here
        print("CONTROLLER 5: performing turn")
        radius = .2
        speed = 45
        # args: [rTurn (boolean, if this is a right turn. False = left turn)], [radius of turn]
        turn(True, radius, speed)

        # continue visually navigating afterwards (you'll probably want to kill it gracefully eventually)
        print("CONTROLLER 6: Starting vNav()")
        vNav(True)

        # wait until we see a green light to go again
        print("CONTROLLER 7: waiting until we see a green light")
        while (not greenLight.value):
            pass
        lastStart = datetime.now()

        # spawn a thread to switch greenLight off 1 second from now
        print("CONTROLLER 8: spawning greenLight changer thread")
        greenChangers.append(threading.Thread(target=greenThread))
        greenChangers[-1].start()

        # change turn radius here
        print("CONTROLLER 9: performing turn")
        radius = .45
        speed = 55
        # args: [rTurn (boolean, if this is a right turn. False = left turn)], [radius of turn]
        turn(False, radius, speed)

        # continue visually navigating afterwards (you'll probably want to kill it gracefully eventually)
        print("CONTROLLER 10: Starting vNav()")
        vNav(True)

        # wait until we see a green light to go again
        print("CONTROLLER 7: waiting until we see a green light")
        while (not greenLight.value):
            pass
        lastStart = datetime.now()

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
    greenChangers[0].join()
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
                     "\n 4: Test Mode\n"))

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
