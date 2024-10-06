#!/usr/bin/env python3
 
import rospy
from std_msgs.msg import String
import json

from pymongo import MongoClient
from os import getenv

# Replace with your MongoDB URI
client = MongoClient("mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0")  
print(client)
db = client["safai"]


def msgCallBack(info):
    rospy.loginfo("%s", info.data)
    record = json.loads(info.data)
    collection = db[record['username']]
    del record['username']
    collection.insert_one(record)
    return

if __name__ == '__main__':      # the main function
    rospy.init_node("web_node", anonymous=True)

    web_sub = rospy.Subscriber("object_info", String, msgCallBack, queue_size=10) #topic is object_info
    rospy.spin()