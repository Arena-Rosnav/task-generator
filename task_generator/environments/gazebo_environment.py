import rospy
from std_msgs.msg import Empty
from task_generator.environments.environment_factory import EnvironmentFactory

from .base_environment import BaseEnvironment


@EnvironmentFactory.register("gazebo")
class GazeboEnvironment(BaseEnvironment):
    def __init__(self):
        super().__init__()

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

    def before_reset_task(self):
        self.pause()

    def after_reset_task(self):
        self.unpause()