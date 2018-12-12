from multiprocessing import Process, Value
from enhancedduckvision import vision

if __name__ == '__main__':
    try:
        vOffset = Value('i', 0)
        # define boolean to act as an off switch
        see = Value('b', True)
        vIntersection = Value('i', 0)
        vision_process = Process(target=vision, args=(vOffset, see))
        vision_process.start()
        running = True;
        oldVal = -999
        while(running):
            if (vOffset.value != oldVal):
                oldVal = vOffset.value
                # print("Camera:\t vOffset: %d" % (vOffset.value))
            if (vIntersection == 1):
                print("Approching Intersection")

    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False
        see.value = False
        vision_process.terminate()
