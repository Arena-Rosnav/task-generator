#! /usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Bool
from std_srvs.srv import Empty

from task_generator.utils import Utils
from task_generator.constants import Constants, TaskMode
from task_generator.tasks import get_predefined_task

from task_generator.environments.environment_factory import EnvironmentFactory
from task_generator.environments.gazebo_environment import *


class TaskGenerator:

    def __init__(self) -> None:
        ## Params
        mode = rospy.get_param("/task_mode")
        scenarios_json_path = rospy.get_param("~scenarios_json_path")
        self.auto_reset = (
            rospy.get_param("~auto_reset", False) 
            and (mode in [TaskMode.SCENARIO, TaskMode.RANDOM])
        )

        ## Publishers
        self.pub_scenario_reset = rospy.Publisher("scenario_reset", Int16, queue_size=1)
        self.pub_scenario_finished = rospy.Publisher('scenario_finished', Bool, queue_size=10)
        
        ## Subscribers
        rospy.Subscriber(
            rospy.get_param("robot_odom_topic_name", "odom"), 
            Odometry, 
            self.robot_pos_callback
        )

        ## Services
        rospy.Service(
            "task_generator", Empty, self.reset_task_srv_callback
        )

        ## Vars
        self.env_wrapper = EnvironmentFactory.instantiate(Utils.get_environment())

        paths = {"scenario_json_path": scenarios_json_path}

        self.start_time = rospy.get_time() 
        self.task = get_predefined_task("", mode, PATHS=paths)

        self.first_round = True
        self.curr_goal_pos = None
        self.curr_robot_pos = None
        self.distance_to_goal = math.inf
        self.number_of_resets = 0

        ## Timers
        rospy.Timer(rospy.Duration(0.5), self.check_task_status)

        self.reset_task()

    def check_task_status(self):
        should_reset_task = False

        if self._get_distance_to_goal() <= Constants.GOAL_REACHED_TOLERANCE:
            print("GOAL REACHED")
            should_reset_task = True

        if rospy.get_time() > self.start_time + Constants.TIMEOUT:
            print("TIMEOUT")
            should_reset_task = True

        if not self.auto_reset or not should_reset_task:
            return

        self.reset_task()

    def reset_task(self):
        self.start_time = rospy.get_time()

        self.env_wrapper.before_reset_task()
        
        is_end, goal_position = self.task.reset()

        self.curr_goal_pos = goal_position
        
        self.pub_scenario_reset.publish(self.number_of_resets)
        self._send_end_message_when_is_end(is_end)

        Utils.print_divider_with_text("TASK RESETED!")

        self.env_wrapper.after_reset_task()

        self.number_of_resets += 1

    def robot_pos_callback(self, odom_msg: Odometry) -> None:
        self.curr_robot_pos = odom_msg.pose.pose.position

    def reset_task_srv_callback(self, req):
        rospy.loginfo("Task Generator received task-reset request!")

        self.task.reset()

    def _send_end_message_when_is_end(self, is_end):
        if not is_end:
            return

        self.pub_scenario_finished.publish(Bool(True))
        rospy.signal_shutdown("Finished all episodes of the current scenario")

    def _get_distance_to_goal(self):
        if self.curr_goal_pos == None or self.curr_robot_pos == None:
            return math.inf

        return math.sqrt(
            (self.curr_robot_pos[0] - self.curr_goal_pos[0]) ** 2 
            + (self.curr_robot_pos[1] - self.curr_goal_pos[1]) ** 2
        )


if __name__ == "__main__":
    rospy.init_node("task_generator")
    rospy.wait_for_service("/static_map")
    task_generator = TaskGenerator()
    rospy.spin()
