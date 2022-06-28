import rospy
from task_generator.constants import TaskMode 


def get_predefined_task(ns, mode="random", PATHS=None, *args) -> None:
    # type: (str, str, int, dict) -> None

    get_static_map_srv = rospy.ServiceProxy("/static_map", GetMap)
    static_map = get_static_map_srv()

    robot_manager = RobotManager(ns="", map_=static_map.map)
    obstacle_manager = ObstaclesManager(ns="", map_=static_map.map)
    pedsim_manager = PedsimManager()

    assert mode in [TaskMode.STAGED, TaskMode.SCENARIO, TaskMode.MANUAL, TaskMode.RANDOM], f"Mode '{mode}' not supported"

    # Tasks will be moved to other classes or functions.
    task = None
    if mode == "random":
        forbidden_zones = obstacle_manager.register_random_static_obstacles(
            N_OBS["static"]
        )
        # forbidden_zones = obstacle_manager.register_random_dynamic_obstacles(
        #     N_OBS["dynamic"], forbidden_zones=forbidden_zones
        # )
        task = RandomTask(pedsim_manager, obstacle_manager, robot_manager)
        print("random tasks requested")
    if mode == "staged":
        task = StagedTask(
            ns, pedsim_manager, obstacle_manager, robot_manager, PATHS, *args
        )

    if mode == "scenario":
        # forbidden_zones = obstacle_manager.register_random_static_obstacles(
        #     N_OBS["static"]
        # )
        task = ScenarioTask(
            pedsim_manager, obstacle_manager, robot_manager, PATHS["scenario"]
        )

    if mode == "manual":
        task = ManualTask(pedsim_manager, obstacle_manager, robot_manager)

    return task

