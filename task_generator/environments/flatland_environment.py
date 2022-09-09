from abc import abstractmethod
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
from std_srvs.srv import Trigger
import numpy as np
import os
import yaml
import math
import random
from flatland_msgs.srv import (
    DeleteModelRequest,
    MoveModelRequest, 
    MoveModel, 
    SpawnModel, 
    DeleteModel,
    SpawnModelRequest
)
from flatland_msgs.msg import MoveModelMsg
from pedsim_srvs.srv import SpawnPeds

from ..constants import Constants, FlatlandRandomModel
from .base_environment import BaseEnvironment
from .environment_factory import EnvironmentFactory


T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@EnvironmentFactory.register("flatland")
class FlatlandEnvironment(BaseEnvironment):
    """
        This is the flatland encoder for connecting the
        flatland environment with the arena-rosnav task
        generator. The class implements all methods
        defined in `BaseEnvironment`.

        For flatland to work properly, a dedicated .yaml
        file has to be created for each used model. This
        is, because the spawn model request only contains
        the path to this file instead of the file content
        directly. For each reset a new set of obstacles 
        is created and saved in files. The path to these
        files is defined in the `tmp_model_path` param
        and defaults to `/tmp`.
    """

    def __init__(self, namespace):
        super().__init__(namespace)
        self._namespace = namespace
        self._ns_prefix = "" if namespace == "" else "/" + namespace + "/"

        self._goal_pub = rospy.Publisher(
            f"{self._ns_prefix}goal", PoseStamped, queue_size=1, latch=True
        )
        self._move_base_goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )

        self._move_robot_pub = rospy.Publisher(
            self._ns_prefix + "move_model", MoveModelMsg, queue_size=10
        )

        self._robot_name = rospy.get_param("robot_model")
        self._robot_radius = rospy.get_param("robot_radius")
        self._is_training_mode = rospy.get_param("train_mode")
        self._step_size = rospy.get_param("step_size")
        self._robot_yaml_path = rospy.get_param("robot_yaml_path")
        self._tmp_model_path = rospy.get_param("tmp_model_path", "/tmp")

        rospy.wait_for_service(f"{self._ns_prefix}move_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}spawn_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}delete_model", timeout=T)

        self._move_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}move_model", MoveModel, persistent=True
        )
        self._spawn_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}spawn_model", SpawnModel
        )
        self._delete_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}delete_model", DeleteModel
        )

        self._spawn_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/spawn_peds", SpawnPeds
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/reset_all_peds", Trigger
        )

        self._obstacles_amount = 0

    def before_reset_task(self):
        pass

    def after_reset_task(self):
        pass

    def remove_all_obstacles(self):
        for obs in range(self._obstacles_amount):
            obs_name = FlatlandEnvironment.create_obs_name(obs)

            self._delete_model(obs_name)

        self._obstacles_amount = 0

    def _delete_model(self, name):
        delete_model_request = DeleteModelRequest()
        delete_model_request.name = name

        self._delete_model_srv(delete_model_request)

    def spawn_pedsim_agents(self, agents):
        peds = [agent.getPedMsg() for agent in agents]

        self._spawn_peds_srv(peds)

    def reset_pedsim_agents(self):
        self._reset_peds_srv()

    def spawn_obstacle(self, position, yaml_path=""):
        name = FlatlandEnvironment.create_obs_name(self._obstacles_amount)

        self._spawn_model(yaml_path, name, self._namespace, position)

        self._obstacles_amount += 1
        
    def spawn_random_dynamic_obstacle(self, **args):
        self._spawn_random_obstacle(**args, is_dynamic=True)

    def spawn_random_static_obstacle(self, **args):
        self._spawn_random_obstacle(**args, is_dynamic=False)

    def _spawn_random_obstacle(
            self, is_dynamic=False, position=[0, 0, 0], **args
        ):
        model = self._generate_random_obstacle(is_dynamic=is_dynamic, **args)

        obstacle_name = FlatlandEnvironment.create_obs_name(
            self._obstacles_amount
        )

        model_path = self._create_obstacle_yaml(model, obstacle_name)

        self._spawn_model(model_path, obstacle_name, self._namespace, position)

        self._obstacles_amount += 1 

    def spawn_robot(self):
        self._spawn_model(
            self._robot_yaml_path, self._robot_name, self._namespace, [0, 0, 0]
        )

    def _spawn_model(self, yaml_path, name, namespace, position):
        request = SpawnModelRequest()
        request.yaml_path = yaml_path
        request.name = name
        request.ns = namespace
        request.pose.x = position[0]
        request.pose.y = position[1]
        request.pose.theta = position[2]

        self._spawn_model_srv(request)

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
        pose = Pose2D()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.theta = pos[2]

        move_model_request = MoveModelMsg()
        move_model_request.name = self._robot_name
        move_model_request.pose = pose

        # self._move_model_srv(move_model_request)

        self._move_robot_pub.publish(move_model_request)

    ## HELPER FUNCTIONS TO CREATE MODEL.YAML
    def _generate_random_obstacle(
            self, 
            is_dynamic=False, 
            min_radius=FlatlandRandomModel.MIN_RADIUS, 
            max_radius=FlatlandRandomModel.MAX_RADIUS,
            linear_vel=FlatlandRandomModel.LINEAR_VEL,
            angular_vel_max=FlatlandRandomModel.ANGLUAR_VEL_MAX
        ):
        """
            Creates a dict in the flatland model schema.

            Since a lot of the variables are untouched
            the majority of the dict is filled up with
            constants defined in the `Constants` file.
        """
        body = {
            **FlatlandRandomModel.BODY,
            "type": "dynamic" if is_dynamic else "static"
        }

        footprint = {
            **FlatlandRandomModel.FOOTPRINT,
            **self._generate_random_footprint_type(min_radius, max_radius)
        }

        body["footprints"].append(footprint)

        model = {'bodies': [body], "plugins": []}

        if is_dynamic:
            model['plugins'].append({
                **FlatlandRandomModel.RANDOM_MOVE_PLUGIN,
                'linear_velocity': linear_vel,
                'angular_velocity_max': angular_vel_max
            })

        return model

    def _generate_random_footprint_type(self, min_radius, max_radius):
        """
            An object in flatland can either be a circle with a 
            specific radius or a polygon shape.

            This function will choose a shape randomly and
            creates a shape from this. 

            For the circle the radius is chosen randomly and
            lies in a specific range defined in the `constants` file

            For the polygon, the amount of vertexes is determined
            at first. Then the vertexes are distributed around the center
            and for each vertex a distance to the center is calculated.
            At the end, the vertexes form the polygon. The distance
            to the center is chosen randomly and lies in the range
            defined in `constants`. 
        """
        type = random.choice(["circle", "polygon"])

        if type == "circle":
            radius = random.uniform(min_radius, max_radius)

            return {
                "type": type,
                "radius": radius
            }

        points_amount = random.randint(3, 8) # Defined in flatland definition
        angle_interval = 2 * np.pi / points_amount

        points = []

        for p in range(points_amount):
            angle = random.uniform(0, angle_interval)
            radius = random.uniform(min_radius, max_radius)

            real_angle = angle_interval * p + angle

            points.append([
                math.cos(real_angle) * radius, 
                math.sin(real_angle) * radius
            ])

        return {
            "type": type,
            "points": list(points)
        }

    def _create_obstacle_yaml(self, model, obs_name):
        os.makedirs(self._tmp_model_path, exist_ok=True)

        tmp_model_file_name = self._namespace + "_" + obs_name + ".model.yaml"

        model_file_name = self._tmp_model_path + "/" + tmp_model_file_name

        with open(model_file_name, 'w') as fd:
            yaml.dump(model, fd)

        return model_file_name

    @abstractmethod
    def create_obs_name(number):
        return "obs_" + str(number)

    @abstractmethod
    def check_yaml_path(path):
        return os.path.isfile(path)