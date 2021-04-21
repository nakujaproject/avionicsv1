import io

import pynmea2
import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

while 1:
    try:
        line = sio.readline()
        msg = pynmea2.parse(line)
        value = repr(msg)
        print(repr(msg))
        file1 = open("gps_data.txt","a")
        file1.write(value)
        file1.write("\n")
        file1.close()
    except serial.SerialException as e:
        print('Device error:{}'.format(e))
        continue
    except pynmea2.ParseError as e:
        print('parse error: {}'.format(e))
        continue
