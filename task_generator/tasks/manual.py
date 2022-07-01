from threading import Lock
import rospy
from geometry_msgs.msg import PoseStamped
from task_generator.constants import TaskMode
from std_srvs.srv import Empty

from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register(TaskMode.MANUAL)
class ManualTask(RandomTask):
    def __init__(
        self,
        obstacles_manager,
        robot_manager,
        map_manager,
        namespace = "",
    ):
        super().__init__(obstacles_manager, robot_manager, map_manager)

        self.namespace = namespace
        self.namespace_prefix = "" if namespace == "" else "/" + namespace + "/"

        self.prevent_endless_loop_lock = Lock()

        rospy.Subscriber(f"{self.namespace_prefix}/task_generator/set_goal", PoseStamped, self._set_goal_callback)

        self._trigger_reset_srv = rospy.ServiceProxy("task_generator", Empty)

        self._current_goal = None

    def reset(self):
        return super().reset(goal=self._current_goal)

    def _set_goal_callback(self, goal):
        goal = goal.pose.position
        
        self._current_goal = [goal.x, goal.y, 0]

        self._trigger_reset_srv()
