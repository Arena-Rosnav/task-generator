from ast import Constant
import traceback
import rospy
import rospkg
import yaml
import os
import roslaunch
from task_generator.constants import Constants

from task_generator.environments.environment_factory import EnvironmentFactory
from task_generator.environments.gazebo_environment import GazeboEnvironment
from task_generator.environments.flatland_environment import FlatlandRandomModel
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.manual import ManualTask
from task_generator.tasks.random import RandomTask
from task_generator.tasks.scenario import ScenarioTask
from task_generator.tasks.staged import StagedRandomTask
from task_generator.utils import Utils

from map_distance_server.srv import GetDistanceMap

from nav_msgs.srv import GetMap


def get_predefined_task(namespace, mode, environment=None, **kwargs):
    """
    Gets the task based on the passed mode
    """
    if environment == None:
        environment = EnvironmentFactory.instantiate(Utils.get_environment())(namespace)

    rospy.wait_for_service("/distance_map")

    service_client_get_map = rospy.ServiceProxy("/distance_map", GetDistanceMap)


    # rospy.wait_for_service("/static_map")

    # service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)

    map_response = service_client_get_map()

    map_manager = MapManager(map_response)
    obstacle_manager = ObstacleManager(namespace, map_manager, environment)

    robot_managers = create_robot_managers(namespace, map_manager, environment)

    # For every robot
    # - Create a unique namespace name
    # - Create a robot manager
    # - Launch the robot.launch file

    # return

    # robot_manager = RobotManager(namespace, map_manager, environment)

    task = TaskFactory.instantiate(
        mode,
        obstacle_manager,
        robot_managers,
        map_manager,
        namespace=namespace,
        **kwargs
    )

    return task

def create_robot_managers(namespace, map_manager, environment):
    # Read robot setup file
    robot_setup_file = rospy.get_param('/robot_setup_file')

    if robot_setup_file == "":
        robots = create_default_robot_list(
            rospy.get_param("/model"),
            rospy.get_param("/local_planner"),
            rospy.get_param("/agent_name", "")
        )
    else:
        robots = read_robot_setup_file(robot_setup_file)

    print(robots)

    if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
        return [RobotManager(namespace, map_manager, environment, robots[0])]

    robot_managers = []

    for robot in robots:
        amount = robot["amount"]

        print(robot, robot['model'])

        for r in range(0, amount):
            name = f"{robot['model']}_{r}_{len(robot_managers)}"  

            robot_managers.append(
                RobotManager(namespace + "/" + name, map_manager, environment, robot)
            )

            # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            #     ["arena_bringup", "robot.launch"]
            # )

            # args = [
            #     f"model:={robot['model']}",
            #     f"local_planner:={robot['planner']}",
            #     f"namespace:={name}",
            #     *([f"agent_name:={robot.get('agent')}"] if robot.get('agent') else [])
            # ]

            # print(roslaunch_file)

            # launch = roslaunch.parent.ROSLaunchParent(
            #     roslaunch.rlutil.get_or_generate_uuid(None, False),
            #     [(*roslaunch_file, args)]
            # )

            # launch.start()

            # robot_managers.append(0)
            
    print("CREATED ALL ROBOTS")

    return robot_managers


def read_robot_setup_file(setup_file):
    try:
        with open(
            os.path.join(rospkg.RosPack().get_path("task-generator"), "robot_setup", setup_file),
            "r"
        ) as file:
            return yaml.safe_load(file)["robots"]
    except:
        traceback.print_exc()
        rospy.signal_shutdown()


def create_default_robot_list(robot_model, planner, agent):
    return [{
        "model": robot_model,
        "planner": planner,
        "agent": agent,
        "amount": 1
    }]