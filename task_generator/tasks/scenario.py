import rospy
import rospkg
import os

from task_generator.constants import TaskMode
from task_generator.tasks.base_task import BaseTask
from ArenaScenario import *
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register(TaskMode.SCENARIO)
class ScenarioTask(BaseTask):
    def __init__(
        self,
        obstacles_manager,
        robot_manager,
        map_manager,
        **kwargs
    ):
        super().__init__(obstacles_manager, robot_manager, map_manager, **kwargs)

        scenario_file_path = rospy.get_param("~scenario_json_path")

        # load scenario from file
        self.scenario = ArenaScenario()
        self.scenario.loadFromFile(scenario_file_path)

        self._check_map_paths()

        self.reset_count = 0
        
        self.desired_resets = self.scenario.resets

        if self.desired_resets <= 0:
            rospy.loginfo(
                f"Setting resets to default of {TaskMode.Scenario.RESETS_DEFAULT}"
            )
            self.desired_resets = TaskMode.Scenario.RESETS_DEFAULT

        self.obstacles_manager.start_scenario(self.scenario)

    def reset(self):
        if self.reset_count >= self.desired_resets:
            return True, list(self.scenario.robotGoal)

        super().reset(
            lambda: self.reset_scenario()
        )

        self.reset_count += 1

        return False, list(self.scenario.robotGoal)

    def reset_scenario(self):
        self.obstacles_manager.reset_scenario(self.scenario)

        self.robot_manager.reset(
            start_pos=(
                self.scenario.robotPosition[0], 
                self.scenario.robotPosition[1], 
                0
            ), 
            goal_pos=(
                self.scenario.robotGoal[0], 
                self.scenario.robotGoal[1], 
                0
            )
        )

    def _check_map_paths(self):
        static_map = rospy.get_param("map_path")
        scenario_map_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"), 
            "maps", 
            self.scenario.mapPath
        )

        if not static_map == scenario_map_path:
            rospy.logerr("Map path of scenario and static map are not the same. Shutting down.")
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown("Map path of scenario and static map are not the same.")