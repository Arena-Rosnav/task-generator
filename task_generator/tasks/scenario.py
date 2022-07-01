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
        scenario_file_path,
    ):
        super().__init__(obstacles_manager, robot_manager)

        # load scenario from file
        self.scenario = ArenaScenario()
        self.scenario.loadFromFile(scenario_file_path)

        self.reset_count = 0

    def reset(self):
        if self.reset_count >= self.scenario.resets:
            return True, self.scenario.robotGoal

        super().reset(
            lambda: self.reset_scenario()
        )

        self.reset_count += 1

        return False, self.scenario.robotGoal

    def reset_scenario(self):
        self.obstacles_manager.reset_scenario(self.scenario)

        self.robot_manager.reset(
            start_pos=(
                self.scenario.robotPosition[0], 
                self.scenario.robotPosition[1], 
            0), 
            goal_pos=(
                self.scenario.robotGoal[0], 
                self.scenario.robotGoal[1], 
                0
            )
        )