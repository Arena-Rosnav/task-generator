import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from task_generator.constants import Constants


class BaseTask():
    """
        Base Task as parent class for all other tasks.
    """

    def __init__(self, obstacles_manager, robot_manager, map_manager, *args, **kwargs):
        self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self.map_manager = map_manager
        
    def reset(self, callback):
        """
            Calls a passed reset function (usually the tasks own reset)
            inside a loop so when the callback fails once it is tried
            again. After MAX_RESET_FAIL_TIMES the reset is considered
            as fail and the simulation is shut down.
        """
        fails = 0
        return_val = False, None 

        while fails < Constants.MAX_RESET_FAIL_TIMES:
            try:
                return_val = callback()

                break
            except rospy.ServiceException as e:
                rospy.logwarn(repr(e))
                fails += 1

        if fails >= Constants.MAX_RESET_FAIL_TIMES:
            rospy.signal_shutdown("Reset error!")
            raise Exception("reset error!")

        return return_val
