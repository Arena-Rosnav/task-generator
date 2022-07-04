import rospy
import os

class Utils:
    def get_environment():
        return os.getenv("ENVIRONMENT", "flatland").lower()

