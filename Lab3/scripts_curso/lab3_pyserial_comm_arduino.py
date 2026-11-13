
import serial
import time
import datetime

def read_and_wait(ser, wait_time):
    output = ""
    flag = True
    start_time = time.time()

    while flag:
        # Wait until there is data waiting in the serial buffer
        if ser.in_waiting > 0:
            # Read data out of the buffer until a carriage return / new line is found
            serString = ser.readline()

            # Print the contents of the serial data
            try:
                output = serString.decode("Ascii")
                print(serString.decode("Ascii"))
            except:
                pass
        else:
            deltat = time.time() - start_time
            if deltat>wait_time:
                flag = False
                
    return output        



def main():
    print("Starting")

    ser = serial.Serial('COM3', baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0, stopbits=serial.STOPBITS_ONE)
    print("COM port in use:  {0}".format(ser.name))
    
    serString = ""  # Used to hold data coming over UART

    for k in range(1,10000):
        #print('going to P3')
        #ser.write(b'MOVE P3\r')
        read_and_wait(ser,2)


        print('iteration ', k)
    
    # closing and housekeeping    
    ser.close()

    print('housekeeping completed - exiting')


########################################################################    
if __name__ == "__main__":
    main()
