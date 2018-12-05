from multiprocessing import Process, Value
from duckvision import vision

if __name__ == '__main__':
    try:
        vOffset = Value('i', 0)
        # define boolean to act as an off switch
        see = Value('b', True)
        vision_process = Process(target=vision, args=(vOffset, see))
        vision_process.start()
        running = True;
        oldVal = -999
        while(running):
            if (vOffset.value != oldVal):
                oldVal = vOffset.value
                # print("Camera:\t vOffset: %d" % (vOffset.value))

    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False
        see.value = False
        vision_process.terminate()
