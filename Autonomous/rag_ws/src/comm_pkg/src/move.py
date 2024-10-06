#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from serial import Serial

arduino = Serial("/dev/ttyACM0", 9600, timeout=1) 

def move(data):
    data = json.loads(data.data)

    if data.data['Ahead'] == 'true':
        rospy.loginfo('yaay', 'detect')
        val = data.data['Centre'] - data.data['Object_pos']
        if 0 < val < 106:
            arduino.write(['a', 'l', 'z'])
        elif 106 < val < 212:
            arduino.write(['a', 'l', 'y'])
        elif 212 < val < 320:
            arduino.write(['a', 'l', 'x'])
        elif 320 < val < 426:
            arduino.write(['a', 'r', 'x'])
        elif 426 < val < 532:
            arduino.write(['a', 'r', 'y'])
        elif 532 < val < 640:
            arduino.write(['a', 'r', 'z'])

    elif data.data['Ahead'] == 'false':
        arduino.write('b') # go ahead for 3 seconds
    
    elif data.data['Ahead'] == 'detected':
        arduino.write('c')
    return


if __name__ == '__main__':

    rospy.init_node('move_node', anonymous=True)
    rospy.Subscriber('detect', String, move)
    rospy.spin()