from datetime import datetime
import serial

#global variables
port = "/dev/ttyACM0"
rate = 115200
s1 = serial.Serial()
s1.port = port
s1.baudrate = rate
s1.timeout = 2

# to prevent the Pi from getting too far ahead of the arduino
def write(cmd):
    start = datetime.now()
    s1.write(cmd.encode())
    total = (datetime.now() - start).total_seconds()
    print("Total INNER write call: %f" % total)

    start = datetime.now()
    s1.flush()
    total = (datetime.now() - start).total_seconds()
    print("Total flush call: %f" % total)

def read():
    start = datetime.now()
    bytesToRead = s1.inWaiting()
    total = (datetime.now() - start).total_seconds()
    print("Total inWaiting call: %f" % total)

    start = datetime.now()
    response = s1.read(bytesToRead)
    total = (datetime.now() - start).total_seconds()
    print("Total read call: %f" % total)

    return response

# main method
if __name__ == '__main__':
    s1.open()
    for x in range (10):
        start = datetime.now()
        write("irr")
        total = (datetime.now() - start).total_seconds()
        print("Total write call: %f" % total)

        start = datetime.now()
        response = read()
        total = (datetime.now() - start).total_seconds()
        print("Response:%s\nTotal read call: %f\n" % (response, total))
