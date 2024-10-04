import serial # FOR CONTROLLING FROM SERIAL PORT
import keyboard # FOR TAKING KEYS INPUTS FROM KEYBOARD
 
# SELECTING SERIAL PORT
arduino = serial.Serial("COM15", 9600, timeout=1) 

while True:
    if keyboard.is_pressed('w'):
        # GIVING COMMAND 
        arduino.write(b'w')
        # CREATING A LOOP WHICH WILL RUN  TILL THE KEY W IS PRESSED 
        while keyboard.is_pressed('w'):
            pass
        # IF W IS RELEASED THEN , IT WILL EXIT FROM LOOP AND TRASNMIT M WHICH WILL STOP THE CAR
        arduino.write(b'm')
        
     # SIMILIARLLY A DID THIS FOR A , S & D    
    if keyboard.is_pressed('a'):
        arduino.write(b'a')
         
        while keyboard.is_pressed('a'):
            pass
        arduino.write(b'm')
        
         
    if keyboard.is_pressed('d'):
        arduino.write(b'd')
         
         
        while keyboard.is_pressed('d'):
            pass
        arduino.write(b'm')
         
         
    if keyboard.is_pressed('s'):
        arduino.write(b's')
         
         
        while keyboard.is_pressed('s'):
            pass
        arduino.write(b'm') 
    if keyboard.is_pressed('c'):
        arduino.write(b'c')
         
         
        while keyboard.is_pressed('c'):
            pass
        arduino.write(b'm') 
    if keyboard.is_pressed('o'):
        arduino.write(b'o')
         
         
        while keyboard.is_pressed('o'):
            pass
        arduino.write(b'm')

    if keyboard.is_pressed('b'):
        arduino.write(b'b')
         
         
        while keyboard.is_pressed('b'):
            pass
        arduino.write(b'm')
    if keyboard.is_pressed('l'):
        arduino.write(b'l')
         
         
        while keyboard.is_pressed('l'):
            pass
        arduino.write(b'm')
    if keyboard.is_pressed('k'):
        arduino.write(b'k')
         
         
        while keyboard.is_pressed('k'):
            pass
        arduino.write(b'm')
    if keyboard.is_pressed('u'):
        arduino.write(b'u')
         
         
        while keyboard.is_pressed('u'):
            pass
        arduino.write(b'm')
    if keyboard.is_pressed('i'):
        arduino.write(b'i')
         
         
        while keyboard.is_pressed('i'):
            pass
        arduino.write(b'm')