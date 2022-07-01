import rospy
from nav_msgs.srv import GetMap

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

def get_predefined_task(namespace, mode, environment, **args):
    # get the map
    rospy.wait_for_service("/static_map")
    service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
    map_response = service_client_get_map()

    map_manager = MapManager(map_response.map)

    robot_manager = RobotManager(namespace, map_manager, environment)
    obstacle_manager = ObstacleManager(namespace, map_manager, environment)

    task = TaskFactory.instantiate(mode, obstacle_manager, robot_manager, map_manager, namespace=namespace, **args)

    return task

def get_predefined_task_outside(namespace, mode, start_stage, paths):
    environment = EnvironmentFactory.instantiate(Utils.get_environment())(namespace)

    return get_predefined_task(namespace, mode, environment, start_stage=start_stage, paths=paths)