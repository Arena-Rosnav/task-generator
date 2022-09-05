import rospy
import os

class Utils:
    def get_environment():
        return os.getenv("ENVIRONMENT", "flatland").lower()
    
    def get_arena_type():
        return os.getenv("ARENA_TYPE", "training").lower()