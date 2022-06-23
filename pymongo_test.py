import json

import pymongo

# Initialize the database and collection
client = pymongo.MongoClient("localhost", 27017)
db = client.PythonTestDB
collection = db.test_collection

# Find a random document or entry in the collection and count total number of documents
print("DB : ", db)
print("Collection : ", collection.find_one())
print("Collection count : ", collection.count_documents({}))

# Create a dummy data and add it to the collection
dummy_Data_json = '{"DateTime": "2000-01-01 00:05:41", "MAC": "00025b00ff01", "SerialNumber": "24", "License": "98 76 98 45 b0 60 0b 6a c1 1a e0 b3 39 54 6d c1 a7 ee 81 6d 79 a2 34 40 e7 04 54 02 6d 3a 17 7c 27 0a 3c 1f 39 a3 ea c7 c5 0b 84 75 72 dd 54 6f 74 b6 19 9a 4d 44 ad f8 9d 48 cd 70 d2 27 74 f8", "PCBA Sr-Number": "0", "Region": 1, "Trim coarse": "11", "Trim fine": "0", "Trim": "0", "DateTime Headset": "", "Special-Flags": {"usb_autoboot": true}}'
dummy_data_json_obj = json.loads(dummy_Data_json)
# collection.insert_one(dummy_data_json_obj)
print("Collection count : ", collection.count_documents({}))

# Use the update method to update data in an existing object.
myquery = {"SerialNumber": "23"}
newValues = {"$set": {"Region": "2"}}
collection.update_many(
    myquery, newValues
)  # note that you can also use update_one and it will update the first occurrence that it finds
print("Collection count : ", collection.count_documents({}))

# Query the database
myquery = {"SerialNumber": "24"}
mydoc = collection.find(myquery)
print(
    "Printing all data found using query, total items found : ",
    collection.count_documents(myquery),
)
for x in mydoc:
    print(x, "\n")
