import airsim
import os
from time import sleep
import argparse


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='')
    parser.add_argument('-o', '--output', default='output', help='Captures path')
    args = parser.parse_args()
    return args


def main():
    args = get_args()
    client = airsim.MultirotorClient(ip=args.ip)
    client.confirmConnection()

    if client.ping():
        print("connected to the client...")
    else:
        print("can't connect to the client...")
        exit()
    output = args.output
    data_file = open(os.path.join(output, "imu_data.txt"), 'w')
    data_file.write("timestamp\t angular_vel_x \t angular_vel_y \t angular_vel_z \t linear_acc_x \t linear_acc_y \t linear_acc_z \t orientation_w \t orientation_x \t orientation_y \t orientation_z \n")
    gt_file = open(os.path.join(output, "gt_data.txt"), 'w')
    gt_file.write("timestamp\t pos_x \t pos_y \t pos_z \t orientation_w \t orientation_x \t orientation_y \t orientation_z \n")

    imu_frequency = 200  # in hertz

    while(client.ping()):
        sleep(1 / imu_frequency)
        real_pos = client.simGetGroundTruthKinematics().position
        imu_data = client.getImuData()

        gt_dict = {}
        imu_data_dict = {}
        imu_data_dict['angular_vel'] = [
            imu_data.angular_velocity.x_val,
            imu_data.angular_velocity.y_val,
            imu_data.angular_velocity.z_val]
        imu_data_dict['linear_acc'] = [
            imu_data.linear_acceleration.x_val,
            imu_data.linear_acceleration.y_val,
            imu_data.linear_acceleration.z_val,
        ]
        imu_data_dict['orientation'] = [
            imu_data.orientation.w_val,
            imu_data.orientation.x_val,
            imu_data.orientation.y_val,
            imu_data.orientation.z_val,
        ]
        imu_data_dict['timestamp'] = imu_data.time_stamp

        real_pos = client.simGetGroundTruthKinematics().position
        real_orient = client.simGetGroundTruthKinematics().orientation
        gt_dict['position'] = [real_pos.x_val, real_pos.y_val, real_pos.z_val]
        gt_dict['orientation'] = [real_orient.w_val, real_orient.x_val, real_orient.y_val, real_orient.z_val]
        gt_dict['timestamp'] = imu_data.time_stamp

        # print(gt_dict)
        data_file.write(str(imu_data_dict['timestamp']) +
                        "\t" +
                        str(imu_data_dict['angular_vel'][0]) +
                        "\t" +
                        str(imu_data_dict['angular_vel'][1]) +
                        "\t" +
                        str(imu_data_dict['angular_vel'][2]) +
                        "\t" +
                        str(imu_data_dict['linear_acc'][0]) +
                        "\t" +
                        str(imu_data_dict['linear_acc'][1]) +
                        "\t" +
                        str(imu_data_dict['linear_acc'][2]) +
                        "\t" +
                        str(imu_data_dict['orientation'][0]) +
                        "\t" +
                        str(imu_data_dict['orientation'][1]) +
                        "\t" +
                        str(imu_data_dict['orientation'][2]) +
                        "\t" +
                        str(imu_data_dict['orientation'][3]) +
                        " \n")
        gt_file.write(str(gt_dict['timestamp']) +
                      "\t" +
                      str(gt_dict['position'][0]) +
                      "\t" +
                      str(gt_dict['position'][1]) +
                      "\t" +
                      str(gt_dict['position'][2]) +
                      "\t" +
                      str(gt_dict['orientation'][0]) +
                      "\t" +
                      str(gt_dict['orientation'][1]) +
                      "\t" +
                      str(gt_dict['orientation'][2]) +
                      "\t" +
                      str(gt_dict['orientation'][3]) +
                      "\n")

    print("connection closed...")


if __name__ == '__main__':
    main()
