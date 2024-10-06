#!/usr/bin/env python3
 
import rospy
from std_msgs.msg import String
import json
from datetime import datetime
from pymongo import MongoClient
from os import getenv

# Replace with your MongoDB URI
client = MongoClient("mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0")  
print(client)
db = client["safai"]
now = datetime.now()


def msgCallBack(info):
    rospy.loginfo("%s", info.data)
    record = json.loads(info.data)
    collection = db["gobi_waste_records"]

    record["Location"] = "23 N 87.4 E"
    record['Time'] = now.strftime("%H:%M:%S"),  # Convert time to string
    record['Date'] = now

    collection.insert_one(record)
    return

if __name__ == '__main__':      # the main function
    rospy.init_node("web_node", anonymous=True)

    web_sub = rospy.Subscriber("object_info", String, msgCallBack, queue_size=10) #topic is object_info
    rospy.spin()