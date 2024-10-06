#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from serial import Serial

# Open serial connection to Arduino (adjust the port as necessary)
arduino = Serial("/dev/ttyACM0", 9600, timeout=1) 

def move(data):
    # Parse JSON data from the ROS message
    data = json.loads(data.data)

    if data['Ahead'] == 'true':
        rospy.loginfo('Moving towards detected object')
        val = data['Centre'][0] - data['Object_pos'][0]
        if 0 < val < 106:
            arduino.write(b'a')  # Go ahead and move left slightly
            arduino.write(b'l')
            arduino.write(b'z')
        elif 106 < val < 212:
            arduino.write(b'a')
            arduino.write(b'l')
            arduino.write(b'y')
        elif 212 < val < 320:
            arduino.write(b'a')
            arduino.write(b'l')
            arduino.write(b'x')
        elif 320 < val < 426:
            arduino.write(b'a')
            arduino.write(b'r')
            arduino.write(b'x')
        elif 426 < val < 532:
            arduino.write(b'a')
            arduino.write(b'r')
            arduino.write(b'y')
        elif 532 < val < 640:
            arduino.write(b'a')
            arduino.write(b'r')
            arduino.write(b'z')

    elif data['Ahead'] == 'false':
        arduino.write(b'b')  # Move ahead for 3 seconds

    elif data['Ahead'] == 'detected':
        arduino.write(b'c')  # Object detected but stationary

if __name__ == '__main__':
    rospy.init_node('move_node', anonymous=True)
    rospy.Subscriber('detect', String, move)
    rospy.spin()
