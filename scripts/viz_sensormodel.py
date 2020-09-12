# Testing Code for SensorModel.py
# Matt Schmittle
# DO NOT EDIT

import matplotlib.pyplot as plt
import numpy as np
import rosbag
from nav_msgs.msg import OccupancyGrid

import mushr_pf.utils as utils
from mushr_pf.sensor_model import SensorModel

if __name__ == "__main__":
    maps = ["hallway", "intersection"]
    print("Running " + str(len(maps)) + " maps")

    for m in range(0, len(maps)):

        # Load laser scan and map from bag
        bag_path = "../bags/" + maps[m] + ".bag"
        bag = rosbag.Bag(bag_path)
        for _, msg, _ in bag.read_messages(topics=["/scan"]):
            laser_msg = msg
            break
        for _, msg, _ in bag.read_messages(topics=["/map"]):
            raw_map_msg = msg
            break
        map_info = raw_map_msg.info

        # Convert to proper type. Fixes error with reading from bag and ros types
        map_msg = OccupancyGrid()
        map_msg.info = map_info
        map_msg.header = raw_map_msg.header
        map_msg.data = raw_map_msg.data

        # Create numpy array representing map for later use
        array_255 = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width)
        )
        permissible_region = np.zeros_like(array_255, dtype=bool)
        permissible_region[
            array_255 == 0
        ] = 1  # Numpy array of dimension (map_msg.info.height, map_msg.info.width),# With values 0: not permissible, 1: permissible
        permissible_x, permissible_y = np.where(permissible_region == 1)

        # Potentially downsample permissible_x and permissible_y here
        print("Creating particles")
        angle_step = 25
        particles = np.zeros((angle_step * permissible_x.shape[0], 3))
        for i in range(angle_step):
            idx_start = i * (particles.shape[0] / angle_step)
            idx_end = (i + 1) * (particles.shape[0] / angle_step)
            particles[idx_start:idx_end, 0] = permissible_y[:]
            particles[idx_start:idx_end, 1] = permissible_x[:]
            particles[idx_start:idx_end, 2] = i * (2 * np.pi / angle_step)

        utils.map_to_world(particles, map_info)
        weights = np.ones(particles.shape[0]) / float(particles.shape[0])

        # Instatiate Your Sensor Model
        sensor_model = SensorModel(
            "/fake_topic1", 18, True, 11.0, map_msg, particles, weights, 0.33
        )

        sensor_model.lidar_cb(laser_msg)
        print("Going to wait for sensor model to finish")
        sensor_model.state_lock.acquire()
        print("Done, preparing to plot")
        weights = weights.reshape((angle_step, -1))

        weights = np.amax(weights, axis=0)
        print(map_msg.info.height)
        print(map_msg.info.width)
        print(weights.shape)
        w_min = np.amin(weights)
        w_max = np.amax(weights)
        print("w_min = %f" % w_min)
        print("w_max = %f" % w_max)
        weights = 0.9 * (weights - w_min) / (w_max - w_min) + 0.1

        img = np.zeros((map_msg.info.height, map_msg.info.width))
        for i in range(len(permissible_x)):
            img[permissible_y[i], permissible_x[i]] = weights[i]

        infile = open("weights.csv", "r")
        correct_weights = infile.read().split(",")
        plt.figure(maps[m])
        if m < 1:
            plt.imshow(img)
            plt.savefig("{}_test.png".format(maps[m]))
        else:
            img = img[1750:2300, 1550:2100]
            plt.imshow(img)
            plt.savefig("{}_test.png".format(maps[m]))
        print("check weight", np.mean(img))
    # plt.show()
