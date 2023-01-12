# Airsim Recording Scripts

This repository contains scripts to fly a drone on a given path and record following for dataset creation

    - Images
    - IMU data
    - Ground truth path

The script can fly multiple drones at the same time and record the data into folders specified in configuration.

The movement is broken into 2 steps, first initialization then main drone movement.

The definition of both the paths are given in different files in following format:

Each line is a step to be taken by the drone which can be move or rotate.

-   **Move Step**: syntax: `m <x> <y> <z>`
-   **Rotate Step**: syntax: `r <angle to rotate by in degrees>`

```
Note: All measurements here are in NED co-ordinate system in SI units. 
```

If you don't want to hard code rotation and let drone automatically rotate so that its always facing the movement direction, just pass `auto_rotate: true` in config file.

To record,

-   Create a new config file using [this](./config/small_square_street/6dof.yaml) as an example
    -   ip in this is the ip of the machine on which unreal engine is running.
-   Start unreal engine and start airsim inside it.
-   Specify paths, and change config variables in your config file to point to appropriate file positions
-   Install dependencies:
    ```
    pip install -r requirements.txt
    ```
-   Run `multi_drone.py`

    -   Help on how to run:

        ```
          ❯ python multi_drone.py --help
          usage: multi_drone.py [-h] -c CONFIG

          optional arguments:
              -h, --help            show this help message and exit
              -c CONFIG, --config CONFIG
                                  config yaml file
        ```

After recording the dataset, it can be converted into a bag file using [data_to_bag_vins_mask.py](./data_to_bag_vins_mask.py)