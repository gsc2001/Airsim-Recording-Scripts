from ast import arg
from calendar import c
import yaml
from yaml.loader import SafeLoader
import argparse
import subprocess
import time
import airsim

processes = []


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', required=True, help='config yaml file')
    args = parser.parse_args()
    return args


def parse_config_file(file_name):
    with open(file_name, 'rb') as f:
        config = yaml.load(f, Loader=SafeLoader)
    if config['ip'] == 'localhost':
        config['ip'] = ''
    return config


def spawn_for_drone(drone_dict, drone_name, ip):
    global processes
    command = [
        "python",
        "moveDroneSine.py",
        "--ip",
        ip,
        "-p",
        drone_dict['main_path'],
        "-i",
        drone_dict['init_path'],
        "-v",
        str(drone_dict['velocity']),
        "-o",
        drone_dict['output_dir'],
        "--drone_name",
        drone_name
    ]

    if drone_dict['auto_rotation']:
        command.append("--auto_rotation")
    print(command)

    proc = subprocess.Popen(command)
    processes.append(proc)


def main():
    global processes
    args = get_args()
    config = parse_config_file(args.config)
    
    client = airsim.MultirotorClient(ip=config['ip'])
    client.reset()

    for drone_name, drone in config['drones'].items():
        spawn_for_drone(drone, drone_name=drone_name, ip=config['ip'])
        # time.sleep(1)

    exit_codes = [proc.wait() for proc in processes]
    print('Exit codes: ', exit_codes)


if __name__ == '__main__':
    main()
