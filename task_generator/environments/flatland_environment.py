from abc import abstractmethod
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
import tf
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

from ..constants import Constants, FlatlandRandomModel
from .base_environment import BaseEnvironment
from .environment_factory import EnvironmentFactory


@EnvironmentFactory.register("flatland")
class FlatlandEnvironment(BaseEnvironment):
    def __init__(self, namespace):
        super().__init__(namespace)
        self._namespace = namespace
        self._ns_prefix = "" if namespace == "" else "/" + namespace + "/"

        self._goal_pub = rospy.Publisher(f"{self._ns_prefix}goal", PoseStamped, queue_size=1, latch=True)
        self._move_base_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)

        self._robot_name = rospy.get_param("robot_model")
        self._robot_radius = rospy.get_param("robot_radius")
        self._is_training_mode = rospy.get_param("train_mode")
        self._step_size = rospy.get_param("step_size")
        self._robot_yaml_path = rospy.get_param("robot_yaml_path")
        self._tmp_model_path = rospy.get_param("tmp_model_path")

        rospy.wait_for_service(f"{self._ns_prefix}move_model", timeout=Constants.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(f"{self._ns_prefix}spawn_model", timeout=Constants.WAIT_FOR_SERVICE_TIMEOUT)
        rospy.wait_for_service(f"{self._ns_prefix}delete_model", timeout=Constants.WAIT_FOR_SERVICE_TIMEOUT)

        self._move_model_srv = rospy.ServiceProxy(f"{self._ns_prefix}move_model", MoveModel)
        self._spawn_model_srv = rospy.ServiceProxy(f"{self._ns_prefix}spawn_model", SpawnModel)
        self._delete_model_srv = rospy.ServiceProxy(f'{self._ns_prefix}delete_model', DeleteModel)

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

    def spawn_random_dynamic_obstacle(self, **args):
        print("ARGS", args)
        self._spawn_random_obstacle(**args, is_dynamic=True)

    def spawn_random_static_obstacle(self, **args):
        self._spawn_random_obstacle(**args, is_dynamic=False)

    def _spawn_random_obstacle(self, is_dynamic=False, position=[0, 0, 0], **args):
        model = self._generate_random_obstacle(is_dynamic=is_dynamic, **args)

        obstacle_name = FlatlandEnvironment.create_obs_name(self._obstacles_amount)

        model_path = self._create_obstacle_yaml(model, obstacle_name)

        self._spawn_model(model_path, obstacle_name, self._namespace, position)

        self._obstacles_amount += 1 

    def spawn_robot(self):
        self._spawn_model(self._robot_yaml_path, self._robot_name, self._namespace, [0, 0, 0])

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
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        goal_msg.pose.orientation.w = quaternion[0]
        goal_msg.pose.orientation.x = quaternion[1]
        goal_msg.pose.orientation.y = quaternion[2]
        goal_msg.pose.orientation.z = quaternion[3]

        self._goal_pub.publish(goal_msg)
        self._move_base_goal_pub.publish(goal_msg)

    def move_robot(self, pos):
        pose = Pose2D()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.theta = pos[2]

        move_model_request = MoveModelRequest()
        move_model_request.name = self._robot_name
        move_model_request.pose = pose

        self._move_model_srv(move_model_request)

    ## HELPER FUNCTIONS TO CREATE MODEL.YAML
    def _generate_random_obstacle(
            self, 
            is_dynamic=False, 
            min_radius=FlatlandRandomModel.MIN_RADIUS, 
            max_radius=FlatlandRandomModel.MAX_RADIUS,
            linear_vel=FlatlandRandomModel.LINEAR_VEL,
            angular_vel_max=FlatlandRandomModel.ANGLUAR_VEL_MAX
        ):

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
        type = random.choice(["circle", "polygon"])

        if type == "circle":
            radius = random.uniform(min_radius, max_radius)

            return {
                "type": type,
                "radius": radius
            }

        points_amount = random.randint(3, 8) # Defined in flatland model definition
        angle_interval = 2 * np.pi / points_amount

        points = []

        for p in range(points_amount):
            angle = random.uniform(0, angle_interval)
            radius = random.uniform(min_radius, max_radius)

            real_angle = angle_interval * p + angle

            points.append([math.cos(real_angle) * radius, math.sin(real_angle) * radius])

        return {
            "type": type,
            "points": list(points)
        }

    def _create_obstacle_yaml(self, model, obs_name):
        # since flatland  can only config the model by parsing the yaml file, we need to create a file for every random obstacle
        os.makedirs(self._tmp_model_path, exist_ok=True)

        tmp_model_file_name = self._namespace + "_" + obs_name + ".model.yaml"

        model_file_name = self._tmp_model_path + "/" + tmp_model_file_name

        with open(model_file_name, 'w') as fd:
            yaml.dump(model, fd)

        return model_file_name

    @abstractmethod
    def create_obs_name(number):
        return "obs_" + str(number)