
import airsim
import os
import json
import time

client = airsim.MultirotorClient()
client.reset()
client.confirmConnection()

while(client.ping()):
    real_pos = client.simGetGroundTruthKinematics().position
    print(real_pos)
    time.sleep(1)

print("connection closed...")