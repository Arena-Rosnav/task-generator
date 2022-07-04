import rospy
from threading import Lock
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
        
        self._service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
        self._map_lock = Lock()

        rospy.Subscriber("/map", OccupancyGrid, self._update_map)

    def reset(self, callback):
        """
            Calls a passed reset function (usually the tasks own reset)
            inside a loop so when the callback fails once it is tried
            again. After MAX_RESET_FAIL_TIMES the reset is considered
            as fail and the simulation is shut down.
        """
        self._map_lock.acquire()

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

        self._map_lock.release()

        return return_val

    def _update_map(self, map):
        with self._map_lock:
            self.map_manager.update_map(map)
