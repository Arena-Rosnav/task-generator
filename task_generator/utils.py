import rospy
import os


class Utils:
    def print_divider_with_text(text):
        rospy.loginfo("".join(["="] * 80))
        rospy.loginfo(text)
        rospy.loginfo("".join(["="] * 80))

    def get_environment():
        return os.getenv("ENVIRONMENT", "flatland").lower()
