import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import sys
import os
import re


def angleRead(cor, cor2):
    
    #Read encoder values
    line1 = bytearray()
    while len(line1) < 6: #read until line is filled
        line1.extend(esp32.read(6 - len(line1))) #reads response from Arduino
    #convert stream to independent variables
    if len(line1) != 6:
        print("values dropped")
        print(list(line1))
        return 0
    else:
        angle1, angle2, angle3 = struct.unpack('>HHH', line1)
        
        #convert positionRaw to meters and angleRaw to radians
        angle1 = angle1 - cor
        angle2 = angle2 - cor2
        
        #convert to radians
        thetaRead = -((angle1*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
        theta2Read = -((angle2*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
        #             print(round(theta, 3))
        #             print(round(correction))
        
        #add angles to fit equations
        theta2Read = theta2Read + thetaRead
        
        #filter bad angles out
        if angle1 > 17000:
            thetaRead = 100 #Value for marking missed reading
            print("F1")
        if angle2 > 17000:
            theta2Read = 100 #Value for marking missed reading
            print("F2")
        
#         print(angle1, angle2)
        esp32.reset_input_buffer()
        return thetaRead, theta2Read
 
def positionRead():
    #wait for position value
    line2 = bytearray()
    while len(line2) < 2: #read until line is filled
        line2.extend(arduino.read(2 - len(line2))) #reads response from Arduino
    #convert stream to independent variables
    if len(line2) != 2:
        print("values dropped")
        print(list(line2))
        arduino.reset_input_buffer()
        return 0
    else:
        positionRaw = struct.unpack('>h', line2)[0]
        #convert positionRaw to meters and angleRaw to radians
        x1 = (positionRaw*0.638175)/6400 #based on distance measurement (m) for 6400 steps
#         print(positionRaw)
        arduino.reset_input_buffer()
        return x1

print("program started")

# Define the file path
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, "control_matrices2.txt")

def load_control_data(filename):
    with open(filename, 'r') as f:
        content = f.read()

    mapping = {
        'A': r'--- Matrix A \(Linearized\) ---\s*(.*?)(?=---|\Z)',
        'B': r'--- Matrix B ---\s*(.*?)(?=---|\Z)',
        'C': r'--- Matrix C ---\s*(.*?)(?=---|\Z)',
        'D': r'--- Matrix D ---\s*(.*?)(?=---|\Z)',
        'Kd': r'--- LQR Gain \(Kd\) ---\s*(.*?)(?=---|\Z)',
        'Ld': r'--- Kalman Gain \(Ld\) ---\s*(.*?)(?=---|\Z)'
    }

    results = {}

    for var_name, pattern in mapping.items():
        match = re.search(pattern, content, re.DOTALL)
        if match:
            raw_text = match.group(1)
            
            clean_text = re.sub(r'\*', '', raw_text) 
            
            # Remove brackets and replace newlines with spaces to keep numbers separate
            clean_text = clean_text.replace('[', '').replace(']', '').replace('\n', ' ')
            
            # Convert string numbers to float list
            # .split() handles multiple spaces automatically
            data_list = [float(x) for x in clean_text.split() if x]
            
            # Reconstruct the shapes
            if var_name == 'A':
                results[var_name] = np.array(data_list).reshape(6, 6)
            elif var_name == 'B':
                results[var_name] = np.array(data_list).reshape(6, 1)
            elif var_name == 'C':
                results[var_name] = np.array(data_list).reshape(3, 6)
            elif var_name == 'D':
                results[var_name] = np.array(data_list).reshape(3, 1)
            elif var_name == 'Kd':
                results[var_name] = np.array(data_list).reshape(1, 6)
            elif var_name == 'Ld':
                results[var_name] = np.array(data_list).reshape(6, 3)

    return results

# Usage
data = load_control_data(file_path)

# --- Variable data assignment --- #
A = data['A'] 
B = data['B'] 
C = data['C']
D = data['D']
Kd = data['Kd'] 
Ld = data['Ld']
mcartVal = 0.969 - 0.059
Ts = 1/100

print("Successfully loaded Matrix A with shape:", A.shape)
# print(A)

#-----CONTROL CODE-------#
line = 0x00000000
positionRaw = 0x0000
angleRaw = 0x0000
x = 0 #cart position
theta = 0 #pendulum angle
xDiv = 0 #cart velocity
f = 0
conversion = 0
pulses = 0
#thetaDiv = 0
u = np.array([[0.0]]) #control force
Ymeas = np.array([[0], [0], [0]]) #measured states (no thetaDiv)
Yest = np.array([[0], [0], [0], [0], [0], [0]]) #estimated states
Ylast = np.array([[0], [0], [0], [0], [0], [0]]) #previous state storage
YfinalEst = np.array([[0], [0], [0], [0], [0], [0]]) #previous state storage

#angle corrections
# downVal = 4872 - 8192 #for pendulum 1
# if downVal > 8192:
#     correction = downVal - 8192
# elif downVal == 8192:
#     correction = 0
# else:
#     correction = downVal + 8192

correction = 9323.9 -3.3

correction2 = 7577 -0 #for pendulum 2

#initialize serial
esp32 = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.003) #initiate communication with the ESP32
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003) #initiate communication with the arduino
time.sleep(3) #since code is restarted gives time for arduino
arduino.reset_input_buffer() #clears any old log before reading data
print("\nSerial started\n") #confirms this connection
line = 0x00000000

time.sleep(0.1)


#get first angle measurements
#Read encoder values
theta1, theta2 = angleRead(correction, correction2)

if theta1 == 100:
    theta1 = 0
if theta2 == 100:
    theta2 = 0

# theta2 = theta2 + np.clip(x/10, a_min=-0.07, a_max=0.07) #correct angle towards center
# theta1 = theta1 + np.clip(x/10, a_min=-0.07, a_max=0.07) #correct angle towards center

#send low frequency control value to trigger arduino response
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
arduino.write(sendPulses) #send top value to Arduino

#get position from arduino
x = positionRead()

#store initial measurements in a matrix
Ylast = np.array([[theta1], [0], [theta2], [0], [x], [0]])

loop = 0 #loop counting variable

try:
    while True:
        if loop < 250: #only count until loop 251
            loop = loop + 1
        
        #calculate predicted states (Kalman filter part 1)
        Yest = A @ Ylast + B @ u

        theta1, theta2 = angleRead(correction, correction2)#read angle
#        correctionSend = correction + np.clip(x/20, a_min=-0.05, a_max=0.05) #correct angle towards center

        #Use kalman estimates if angles are dropped
        if theta1 == 100:
            theta1 = Yest[0].item()
        if theta2 == 100:
            theta2 = Yest[2].item()
        
        #load measurements into matrix (using position from last loop)
        Ymeas = np.array([[theta1], [theta2], [x]])
        
        #Factor in measured states(Kalman filter part 2) (@ for matrix multiplication)
        YfinalEst = Yest + Ld @ (Ymeas - C @ Yest)
        Ylast = YfinalEst.copy() #store values for next loop
        
        #calculate control force
        u = -Kd @ YfinalEst * 1.5
#         print(round(u.item(),1))
        u = np.clip(u, a_min=-5, a_max=5)


        #MAY NEED TO IMPLEMENT PID CORRECTIONS FOR POSITION DRIFT
        
        #convert force to velocity and frequency (u.item takes value from 1x1 array)
#         aCart = u.item() / mcartVal #calculate target cart acceleration
#         xDivLast = xDiv #store last speed
#         xDiv = xDivLast + aCart * Ts #calulate target cart speed
        
        xDiv = YfinalEst[5].item() #pull velocity from kalman filter estimation instead of calculating force separately
        
        if loop < 20: #ramp force up to full
            xDiv = xDiv*(loop/20.0)
        
        xDiv = np.clip(xDiv, a_min=-5, a_max=5)
#         print(round(xDiv,2))

        if abs(theta1) > 1:
            xDiv = 0.0;
            break
        
        f = -(xDiv * 6400) / 0.638175 #conversion based on measured distance per pulse
#         print(round(f))
        
        #convert frequency to timer top value
        if f > 2 or f < -2:
            conversion = (0.5/f)/(1.0/250000.0)
            pulses = conversion
        elif f >= 0:
            pulses = 65535
        elif f < 0:
            pulses = -65535
            
        #send timer value to arduino
        arduino.reset_output_buffer()
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        arduino.write(sendPulses) #send top value to Arduino
        
        x = positionRead() #read position from arduino
        correction = correction + x/80 #correct correction value slightly to tune to center
#         print(round(correction,1))
        
        
        

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    print("\nControl loop stopped\n")
    arduino.write(0xFFFFFFFF) #send stop command to Arduino
    time.sleep(0.05) #wait 50ms
    arduino.close() #ends connection between devices

    sys.exit()
