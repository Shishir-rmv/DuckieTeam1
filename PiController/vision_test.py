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
        while(running):
            print("Camera:\t vOffset: %d" % (vOffset.value))
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, gracefully exiting...")
        running = False
        see.value = False
        vision_process.join()
