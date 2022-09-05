#! /usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap
import numpy as np

from matplotlib import pyplot as plt


if __name__ == "__main__":
    rospy.init_node("task_generator")
    
    rospy.wait_for_service("/static_map")

    service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)

    map = service_client_get_map().map
    map.data = np.array(map.data)

    map.data[map.data < 0] = 100

    imgplot = plt.imshow(np.reshape(
            map.data, 
            (map.info.height, map.info.width)
        ))

    print(imgplot, map.data)

    plt.show()

    # imgplot.show()

    rospy.spin()
