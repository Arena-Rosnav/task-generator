import rospy
from std_srvs.srv import Empty
from task_generator.environments.environment_factory import EnvironmentFactory

from .base_environment import BaseEnvironment


@EnvironmentFactory.register("gazebo")
class GazeboEnvironment(BaseEnvironment):
    def __init__(self, namespace):
        super().__init__(namespace)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        print("INITIALIZED")

    def before_reset_task(self):
        self.pause()

    def after_reset_task(self):
        self.unpause()

    def remove_all_obstacles(self):
        raise NotImplementedError()

    def spawn_random_dynamic_obstacle(self):
        raise NotImplementedError()

    def spawn_random_static_obstacles(self):
        raise NotImplementedError()

    def publish_goal(self, goal):
        raise NotImplementedError()

    def move_robot(self, pos):
        raise NotImplementedError()

    def spawn_robot(self):
        raise NotImplementedError()
