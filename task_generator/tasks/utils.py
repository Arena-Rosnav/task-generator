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

from map_distance_server.srv import GetDistanceMap

def get_predefined_task(namespace, mode, environment=None, **kwargs):
    """
    Gets the task based on the passed mode
    """
    if environment == None:
        environment = EnvironmentFactory.instantiate(Utils.get_environment())(namespace)

    rospy.wait_for_service("/distance_map")

    service_client_get_map = rospy.ServiceProxy("/distance_map", GetDistanceMap)

    map_response = service_client_get_map()

    map_manager = MapManager(map_response)

    robot_manager = RobotManager(namespace, map_manager, environment)
    obstacle_manager = ObstacleManager(namespace, map_manager, environment)

    task = TaskFactory.instantiate(
        mode,
        obstacle_manager,
        robot_manager,
        map_manager,
        namespace=namespace,
        **kwargs
    )

    return task