import rospy


class Utils:
    def get_environment():
        return rospy.get_param("environment", "flatland")
