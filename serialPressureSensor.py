'''
https://www.luisllamas.es/controlar-arduino-con-python-y-la-libreria-pyserial/

https://engineersportal.com/blog/2018/2/25/python-datalogger-reading-the-serial-output-from-arduino-to-analyze-data-using-pyserial
https://www.youtube.com/watch?v=n6HSA9KOJN0
https://electronut.in/plotting-real-time-data-from-arduino-using-python/
https://www.instructables.com/id/Plotting-real-time-data-from-Arduino-using-Python-/
http://www.toptechboy.com/tutorial/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/
'''


import serial
import time

tivaCserial = serial.Serial('/dev/ttyACM0',9600)
tivaCserial .flushInput()
time.sleep(2)
#rawString = tivaCserial.readline()
#print("Presión recibida",rawString)
#tivaCserial.close  					#Se cierra el puerto serie


while True:		
	try:
		ser_bytes = tivaCserial.readline()
		decoded_bytes = float( ser_bytes[0:len(ser_bytes)-2].decode("utf-8") )
        print(decoded_bytes)

    except:
    	print("Interrrupción por teclado")
    	break
