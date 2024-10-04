from pymongo import MongoClient
from datetime import datetime

client = MongoClient("mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0")  # Replace with your MongoDB URI

db = client["safai"]
collection = db["gobi_waste_records"]

now = datetime.now()

# Convert time to string before inserting
record = {
    "Class": "Plastic",
    "isBiodegradable": False,
    "Location": "23 N 87.4 E",
    "Time": now.strftime("%H:%M:%S"),  # Convert time to string
    "Date": now  # Date will be stored as a datetime object
}

result = collection.insert_one(record)

print("Record inserted with ID:", result.inserted_id)
