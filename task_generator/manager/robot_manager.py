import rospy
from task_generator.constants import Constants


class RobotManager:
    """
        The robot manager manages the goal and start 
        position of a robot for all task modes.
    """

    def __init__(self, namespace, map_manager, environment):
        self.namespace = namespace
        self.namespace_prefix = "" if namespace == "" else "/" + namespace + "/"
        self.map_manager = map_manager
        self.environment = environment

        self.start_pos = None
        self.goal_pos = None

        self.environment.spawn_robot()

        self.robot_radius = rospy.get_param("robot_radius")

    def reset(self, forbidden_zones=[], start_pos=None, goal_pos=None):
        """
            The manager creates new start and goal position
            when a task is reset, publishes the goal to
            move base and rviz and moves the robot to
            the start position.
        """
        start, goal = self.generate_new_start_and_goal(forbidden_zones, start_pos, goal_pos)

        self.publish_goal()
        self.move_robot_to_start()

        return start, goal

    def generate_new_start_and_goal(self, forbidden_zones, start_pos, goal_pos):
        self.start_pos = self._default_position(
            start_pos,
            self.map_manager.get_random_pos_on_map(
                self.robot_radius + Constants.RobotManager.SPAWN_ROBOT_SAFE_DIST, 
                forbidden_zones
            )
        )

        self.goal_pos = self._default_position(
            goal_pos,
            self.map_manager.get_random_pos_on_map(
                3 * self.robot_radius + Constants.RobotManager.SPAWN_ROBOT_SAFE_DIST, 
                [
                    *forbidden_zones,
                    (
                        self.start_pos[0], self.start_pos[1], self.robot_radius
                    )
                ]
            )
        )

        return self.start_pos, self.goal_pos

    def publish_goal(self):
        if not self.goal_pos == None:
            self.environment.publish_goal(self.goal_pos)

    def move_robot_to_start(self):
        if not self.start_pos == None:
            self.move_robot_to_pos(self.start_pos)

    def move_robot_to_pos(self, pos):
        self.environment.move_robot(pos)

    def _default_position(self, pos, callback_pos):
        if not pos == None:
            return pos

        return callback_pos 