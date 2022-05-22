from re import S
from termios import VEOL
from typing import List
from xml.dom import ValidationErr
import airsim
from dataclasses import dataclass
from enum import Enum, auto
import numpy as np
import time
import subprocess
import os
import argparse


class StepTypes(Enum):
    rotate = 'rotate'
    move = 'move'


@dataclass
class Step:
    type_ = StepTypes.move


@dataclass
class RotationStep(Step):
    type_ = StepTypes.rotate
    to_yaw: float


@dataclass
class MoveStep(Step):
    type_ = StepTypes.move
    position: list


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='')
    parser.add_argument('-p', '--path', required=True, help='Path File')
    parser.add_argument('-i', '--init_path', required=True, help='Initialization Path File')
    parser.add_argument('-v', '--velocity', required=True, type=float, help='Velocity of drone')
    parser.add_argument('-o', '--output', default='output', help='Output path')
    parser.add_argument('--auto_rotation', action='store_true')
    args = parser.parse_args()
    return args


def read_path_file(file_name: str):
    global args
    path = []
    with open(file_name, 'r') as f:
        for line in f:
            items = line.strip().split()
            if items[0] == 'm':
                path.append(MoveStep(position=list(map(float, items[1:]))))
            elif items[0] == 'r':
                # rotation
                path.append(RotationStep(to_yaw=float(items[1])))
            else:
                raise ValidationErr(f'Not supported step type: {items[0]}')
    return path


def moveClientOnPath(client: airsim.MultirotorClient, path: List[Step]):
    global args
    if args.auto_rotation:
        assert all(step.type_ == StepTypes.move for step in path), "Only move steps for auto rotation"
        total_path = list(map(lambda step: airsim.Vector3r(step.position), path))
        client.moveOnPathAsync(total_path, args.velocity, drivetrain=airsim.DrivetrainType.ForwardOnly).join()
    else:
        for step in path:
            if step.type_ == StepTypes.move:
                print(step, step.position, args.velocity)
                client.moveToPositionAsync(*step.position, args.velocity).join()
            elif step.type_ == StepTypes.rotate:
                client.rotateToYawAsync(step.to_yaw).join()


def main():
    # Load path
    global args
    args = get_args()
    path = read_path_file(args.path)
    print(path)
    init_path = read_path_file(args.init_path)
    print(init_path)

    # Setup airsim drone
    client = airsim.MultirotorClient(ip=args.ip)
    client.reset()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    # assign ids
    # assignIds(client)
    # time.sleep(2)

    print("Initializing camera ...")
    # initializing camera
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
                                     airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)])
    time.sleep(2)

    print("Initializing Done ...")
    v = float(args.velocity)
    print(v)

    starting = init_path[0]
    # time.sleep(1)
    print('Moving to start point for initialization ...')
    client.moveToPositionAsync(*starting.position, 1).join()
    print('Reached starting point...')

    output = args.output
    captures_dir = os.path.join(output, 'captures')
    # Delete previous recordings
    if os.path.isfile(output) or os.path.isdir(output):
        if os.path.isdir(captures_dir):
            os.system("rm -rf " + captures_dir)
        os.system("rm -rf " + output)
    os.mkdir(output)
    os.mkdir(captures_dir)

    # Wait for 'g' key presss
    print("Press 'g' to start recording")
    ch = input()

    # Start motion and recording
    print("starting recording..")
    camera_process = subprocess.Popen(['python', 'getCamera.py', '--ip', args.ip, '--output', captures_dir])
    imu_process = subprocess.Popen(['python', 'getDroneData.py', '--ip', args.ip, '--output', output])

    print("Initialization path starting ...")

    moveClientOnPath(client, init_path[1:])
    print("Initialization Done!")

    print("flying on main path..")
    # for point in path:
    #     client.moveToPositionAsync(point.x_val, point.y_val, point.z_val, 2).join()
    moveClientOnPath(client, path)

    # End motion and recording
    print("Done ...")
    # client.moveToZAsync(0, 3).join()
    camera_process.terminate()
    imu_process.terminate()
    camera_process.kill()
    imu_process.kill()
    time.sleep(1)
    client.enableApiControl(False)
    time.sleep(1)
    print("connection closed...")


if __name__ == '__main__':
    main()
