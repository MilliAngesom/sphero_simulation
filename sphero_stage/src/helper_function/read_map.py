#!/usr/bin/env python3

import rospy
import nav_msgs.msg
import numpy as np
import pdb
import cv2
from scipy.ndimage import rotate

from PIL import Image
import numpy as np


def read_map_callback(msg):
    """Callback function that is called when a message is received on the `/map` topic."""

    # Print the necessary information about the map.
    # print("Map information:")
    # print("Type:", msg.header.frame_id)
    # print("Publishers:", msg.info.map_load_time)
    # print("Subscribers:", msg.info.resolution)
    # print("Map size: width x height", msg.info.width, msg.info.height)
    # print("Map origin:", msg.info.origin.position.x, msg.info.origin.position.y)
    map_data = np.array([msg.data])
    # map_data = map_data.reshape(200, 200)
    map_data = np.reshape(map_data, (200, 200), 'F')
    map_data = rotate(map_data, 90)

    # Creates PIL image showing image
    img = Image.fromarray(np.uint8(map_data))
    img.show()

    np.savetxt("map_data_3.txt", map_data)
    # pdb.set_trace()
    # print("Map data size:", map_data.shape)


if __name__ == "__main__":
    rospy.init_node("read_map_topic")
    sub = rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, read_map_callback)
    # read_map_callback()
    rospy.spin()
