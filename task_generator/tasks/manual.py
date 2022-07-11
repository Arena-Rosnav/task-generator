from threading import Lock
import rospy
from geometry_msgs.msg import PoseStamped
from task_generator.constants import TaskMode
from std_srvs.srv import Empty, EmptyRequest

from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register(TaskMode.MANUAL)
class ManualTask(RandomTask):
    """
        Derives from the random task but has a subscriber set
        up to listen on the "task_generator" topic.
        New Goals defined in rviz are sent there and
        will be set as next goal when received by the
        manual task. 
        Except this, the manual task behaves like a random
        task.   
    """

    def __init__(
        self,
        obstacles_manager,
        robot_manager,
        map_manager,
        namespace = "",
        *args, 
        **kwargs
    ):
        super().__init__(
            obstacles_manager, robot_manager, map_manager, *args, **kwargs
        )

        self.namespace = namespace
        self.namespace_prefix = "" if namespace == "" else "/" + namespace + "/"

        self.prevent_endless_loop_lock = Lock()

        rospy.Subscriber(
            f"{self.namespace_prefix}/task_generator/set_goal", 
            PoseStamped, 
            self._set_goal_callback
        )

        self._trigger_reset_srv = rospy.ServiceProxy("task_generator", Empty)

        self._current_goal = None

    def reset(self):
        return super().reset(goal=self._current_goal)

    def _set_goal_callback(self, goal):
        goal = goal.pose.position
        
        self._current_goal = [goal.x, goal.y, 0]

        rospy.loginfo(f"Set goal position to {self._current_goal}")

        self._trigger_reset_srv(EmptyRequest())
