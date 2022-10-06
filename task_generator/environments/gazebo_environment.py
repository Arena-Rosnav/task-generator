import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from pedsim_srvs.srv import SpawnPeds
from std_msgs.msg import Empty
from std_srvs.srv import Empty, Trigger
from task_generator.environments.environment_factory import EnvironmentFactory
from tf.transformations import quaternion_from_euler

from ..constants import Constants
from .base_environment import BaseEnvironment
from .environment_factory import EnvironmentFactory


T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@EnvironmentFactory.register("gazebo")
class GazeboEnvironment(BaseEnvironment):
    def __init__(self, namespace):
        super().__init__(namespace)
        self._namespace = namespace
        self._ns_prefix = "/" if namespace == "" else "/" + namespace + "/"

        self._goal_pub = rospy.Publisher(
            f"{self._ns_prefix}goal", PoseStamped, queue_size=1, latch=True
        )
        self._move_base_goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )

        self._robot_name = rospy.get_param("robot_model")
        self._robot_radius = rospy.get_param("robot_radius")
        self._robot_description = rospy.get_param("robot_description")

        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=T)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=T)
        rospy.wait_for_service(
            f"{self._ns_prefix}pedsim_simulator/spawn_peds", timeout=T
        )
        rospy.wait_for_service(
            f"{self._ns_prefix}pedsim_simulator/reset_all_peds", timeout=T
        )

        self._spawn_model_srv = rospy.ServiceProxy(
            "/gazebo/spawn_urdf_model", SpawnModel
        )
        self._move_model_srv = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState, persistent=True
        )

        self._spawn_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/spawn_peds", SpawnPeds
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/reset_all_peds", Trigger
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

        self._obstacles_amount = 0

    def before_reset_task(self):
        self.pause()

    def after_reset_task(self):
        self.unpause()

    def remove_all_obstacles(self):
        pass

    def spawn_pedsim_agents(self, agents):
        peds = [agent.getPedMsg() for agent in agents]
        self._spawn_peds_srv(peds)

    def reset_pedsim_agents(self):
        self._reset_peds_srv()

    def spawn_obstacle(self, position, yaml_path=""):
        pass

    def spawn_random_dynamic_obstacle(self, **args):
        pass

    def spawn_random_static_obstacle(self, **args):
        pass

    def publish_goal(self, goal):
        goal_msg = PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]

        goal_msg.pose.orientation.w = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 1

        self._goal_pub.publish(goal_msg)
        self._move_base_goal_pub.publish(goal_msg)

    def move_robot(self, pos):
        model_state_request = ModelState()
        model_state_request.model_name = self._robot_name
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, pos[2], axes="sxyz")
        )
        model_state_request.pose = pose
        model_state_request.reference_frame = "world"
        self._move_model_srv(model_state_request)

    def spawn_robot(self):
        request = SpawnModelRequest()
        request.model_name = self._robot_name
        request.model_xml = self._robot_description
        request.robot_namespace = self._ns_prefix
        request.reference_frame = "world"
        self._spawn_model_srv(request)
