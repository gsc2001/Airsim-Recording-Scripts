import airsim
import os
import numpy as np
import cv2
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
    if not os.path.isdir(output):
        exit(1)

    time_file = open(os.path.join(output, 'timestamps.txt'), 'w')

    i = 0
    while(client.ping()):
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False), airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
                                        # airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)
                                         ])
        timestamp = client.getMultirotorState().timestamp
        response_scene = responses[0]
        response_seg = responses[1]
        # response_depth = responses[2]

        if (response_scene.height != 0 and response_scene.width != 0 and response_seg.height !=
                0 and response_seg.width != 0):
            # and response_depth.height != 0 and response_depth.width != 0):
            img1d_scene = np.fromstring(response_scene.image_data_uint8, dtype=np.uint8)
            img_rgb_scene = img1d_scene.reshape(response_scene.height, response_scene.width, 3)

            img1d_seg = np.fromstring(response_seg.image_data_uint8, dtype=np.uint8)
            img_rgb_seg = img1d_seg.reshape(response_seg.height, response_seg.width, 3)

            # depth_img_in_meters  = np.array(response_depth.image_data_float, dtype=np.float32)
            # depth_img_in_meters = depth_img_in_meters.reshape(response_depth.height, response_depth.width, 1)

            # depth_img_in_millimeters = depth_img_in_meters * 1000
            # depth_img_in_centimeters = depth_img_in_meters * 100
            # depth_16bit = np.clip(depth_img_in_millimeters, 0, 65535)
            # depth_cm_16bit = np.clip(depth_img_in_centimeters, 0, 65535)

            cv2.imwrite(os.path.join(output,str(i) + "_scene" + '.png'), img_rgb_scene)
            cv2.imwrite(os.path.join(output,str(i) + "_seg" + '.png'), img_rgb_seg)
            # cv2.imwrite("./captures/" + str(i) + "_depth" + '.png', depth_16bit.astype('uint16'))
            # cv2.imwrite("./captures/" + str(i) + "_depth_cm" + '.png', depth_cm_16bit.astype('uint16'))
            # airsim.write_pfm("./captures/" + str(i) + "_depth_raw" + '.pfm', depth_img_in_meters)
            #print(timestamp)
            time_file.write(str(timestamp) + "\t" + str(i) + "_scene" + '.png ' + "\n")
            i = i + 1

    print("disconnected...")


if __name__ == '__main__':
    main()
