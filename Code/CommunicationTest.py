import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct



#initialize serial
esp32 = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.003) #initiate communication with the ESP32
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003) #initiate communication with the arduino
time.sleep(3) #since code is restarted gives time for arduino
arduino.reset_input_buffer() #clears any old log before reading data
print("\nSerial started\n") #confirms this connection
line = 0x00000000

#get first measurement

#send blank control value
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
arduino.write(sendPulses) #send top value to Arduino

f = 0
fDir = 1



try:
    while True:
        
        #Read encoder values
        esp32.reset_input_buffer()
        line = bytearray()
        while len(line) < 6: #read until line is filled
            line.extend(esp32.read(6 - len(line))) #reads response from Arduino
        #convert stream to independent variables
        if len(line) != 6:
            print("values dropped")
            print(list(line))
        else:
            angle1, angle2, angle3 = struct.unpack('>HHH', line)
            #convert positionRaw to meters and angleRaw to radians
            print(angle1, angle2)
#             angle1 = angle1 - correction
#             theta = ((angle1*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
#             print(angle1, angle2, angle3)
#             print(theta)
        esp32.reset_input_buffer()
        
        
        #Frequency sweep
#         time.sleep(0.0005) # wait 5ms, for use without ESP32 connected
        
        #f = f + 20*fDir #increment f
        
       # if f > 500: #toggle direction
          #  fDir = -1
       # elif f < -500:
      #      fDir = 1
            

        #convert frequency to timer top value
        if f > 2 or f < -2:
            conversion = (0.5/f)/(1.0/250000.0)
            pulses = conversion
        elif f >= 0:
            pulses = 65535
        elif f < 0:
            pulses = -65535
        
#         print(round(pulses))
        
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        arduino.write(sendPulses) #send top value to Arduino
#         cereal.write('\n') #indent

        
        #wait for position value
        line = bytearray()
        while len(line) < 2: #read until line is filled
            line.extend(arduino.read(2 - len(line))) #reads response from Arduino
        #convert stream to independent variables
        if len(line) != 2:
            print("values dropped")
            print(list(line))
        else:
            positionRaw = struct.unpack('>h', line)[0]
            #convert positionRaw to meters and angleRaw to radians
#             print(round(theta, 3))
#             print(round(correction))
#             print(positionRaw)
        arduino.reset_input_buffer()
        
        
#         theta = theta - np.clip(x/20, a_min=-0.05, a_max=0.05)
#         correction = correction + x/50
        

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    print("\nControl loop stopped\n")
    arduino.write(0xFFFFFFFF) #send stop command to Arduino
    #cereal.write('\n') #indent
    time.sleep(0.2) #wait 200ms
    arduino.close() #ends connection between devices525