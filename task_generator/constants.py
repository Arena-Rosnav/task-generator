class Constants:
    GOAL_REACHED_TOLERANCE = 1.0
    TIMEOUT = 3.0 * 60 ## 3 min
    WAIT_FOR_SERVICE_TIMEOUT = 5 # 5 secs
    MAX_RESET_FAIL_TIMES = 3

    class ObstacleManager:
        DYNAMIC_OBSTACLES = 3
        STATIC_OBSTACLES = 0 

        OBSTACLE_MAX_RADIUS = 0.6
    
    class RobotManager:
        SPAWN_ROBOT_SAFE_DIST = 0.4


class TaskMode:
    RANDOM = "random"
    STAGED = "staged"
    SCENARIO = "scenario"
    MANUAL = "manual"

    class Random:
        MIN_DYNAMIC_OBS = 1
        MAX_DYNAMIC_OBS = 3
        MIN_STATIC_OBS = 1
        MAX_STATIC_OBS = 3

    class Scenario:
        RESETS_DEFAULT = 5

class FlatlandRandomModel:
    BODY = {
        "name": "base_link",
        "pose": [0, 0, 0],
        "color": [1, 0.2, 0.1, 1.0],
        "footprints": []
    }
    FOOTPRINT = {
        "density": 1,
        "restitution": 1,
        "layers": ["all"],
        "collision": "true",
        "sensor": "false"
    }
    MIN_RADIUS = 0.2
    MAX_RADIUS = 0.6
    RANDOM_MOVE_PLUGIN = {
        "type": "RandomMove",
        "name": "RandomMove_Plugin",
        "body": "base_link"
    }
    LINEAR_VEL = 0.2
    ANGLUAR_VEL_MAX = 0.2

