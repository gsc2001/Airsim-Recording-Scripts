import airsim
import os
import json

client = airsim.VehicleClient(ip='172.27.208.1')
client.confirmConnection()

if client.ping():
    print("connected to the client...")
else:
    print("can't connect to the client...")
    exit()

print("connection closed...")
