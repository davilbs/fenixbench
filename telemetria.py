import serial

ser = serial.Serial("/dev/pts/1")
ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS
ser.stopbits = serial.STOPBITS_ONE
if ser.is_open:
    try:
        ser.write(str.encode("echo Hi\n"))
    except Exception as e:
        print(e)
    finally:
        ser.close()
else:
    ser.open()
    try:
        ser.write("start")
    except Exception as e:
        print(e)
    finally:
        ser.close()

    