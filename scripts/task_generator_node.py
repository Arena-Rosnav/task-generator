#! /usr/bin/env python3

import math
import rospy
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Bool
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped


from task_generator.utils import Utils
from task_generator.constants import Constants, TaskMode

from task_generator.tasks.utils import get_predefined_task
from task_generator.environments.environment_factory import EnvironmentFactory
from task_generator.environments.gazebo_environment import GazeboEnvironment
from task_generator.environments.flatland_environment import FlatlandRandomModel



class TaskGenerator:
    """
        Task Generator Node
        Will initialize and reset all tasks. The task to use is read from the `/task_mode` param.
    """

    def __init__(self) -> None:
        print("BOOTING TASK GENERATOR")

        ## Params
        self.task_mode = rospy.get_param("/task_mode")
        self.auto_reset = rospy.get_param("~auto_reset", True)

        ## Publishers
        self.pub_scenario_reset = rospy.Publisher("scenario_reset", Int16, queue_size=1)
        self.pub_scenario_finished = rospy.Publisher('scenario_finished', Bool, queue_size=10)
        
        ## Subscribers
        # rospy.Subscriber(
        #     rospy.get_param("robot_odom_topic_name", "odom"), 
        #     Odometry, 
        #     self.robot_pos_callback
        # )
        # rospy.Subscriber(f"/move_base_simple/goal", PoseStamped, self.set_goal_callback)

        ## Services
        rospy.Service("task_generator", Empty, self.reset_task_srv_callback)

        # rospy.wait_for_service("/move_base/clear_costmaps")
        # self._clear_constmaps_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        ## Vars
        self.env_wrapper = EnvironmentFactory.instantiate(Utils.get_environment())("")

        rospy.loginfo(f"Launching task mode: {self.task_mode}")

        self.start_time = rospy.get_time() 
        self.task = get_predefined_task("", self.task_mode, self.env_wrapper)

        self.first_round = True
        self.curr_goal_pos = None
        self.curr_robot_pos = None
        self.distance_to_goal = math.inf
        self.number_of_resets = 0


        self.reset_task()

        ## Timers
        # while not rospy.is_shutdown():
        #     self.check_task_status("")
        #     time.sleep(10)
        rospy.Timer(rospy.Duration(0.5), self.check_task_status)


    def check_task_status(self, _):
        # print("TIMER")

        # should_reset_task = False

        if self.task.is_done():
            self.reset_task()

        # should_reset_task = True

        # if not self.auto_reset or not should_reset_task:
        #     return

        # self.reset_task()

    def reset_task(self):
        self.start_time = rospy.get_time()

        rospy.loginfo("=============")
        rospy.loginfo("Task Reseted!")
        rospy.loginfo("=============")

        self.env_wrapper.before_reset_task()

        # self._clear_constmaps_srv()

        is_end, goal_position = self.task.reset()

        self.curr_goal_pos = goal_position
        
        self.pub_scenario_reset.publish(self.number_of_resets)
        self._send_end_message_on_end(is_end)

        self.env_wrapper.after_reset_task()

        self.number_of_resets += 1

    def robot_pos_callback(self, odom_msg: Odometry) -> None:
        self.curr_robot_pos = odom_msg.pose.pose.position

    def set_goal_callback(self, goal):
        if not self.task_mode == TaskMode.MANUAL:
            return

        goal = goal.pose.position

        self.curr_goal_pos = [goal.x, goal.y, 0]

    def reset_task_srv_callback(self, req):
        rospy.logdebug("Task Generator received task-reset request!")

        self.reset_task()

        return EmptyResponse()

    def _send_end_message_on_end(self, is_end):
        if not is_end:
            return

        rospy.loginfo("Shutting down. All tasks completed")

        self.pub_scenario_finished.publish(Bool(True))
        rospy.signal_shutdown("Finished all episodes of the current scenario")

    def _get_distance_to_goal(self):
        if self.curr_goal_pos == None or self.curr_robot_pos == None:
            return math.inf

        return math.sqrt(
            (self.curr_robot_pos.x - self.curr_goal_pos[0]) ** 2 
            + (self.curr_robot_pos.y - self.curr_goal_pos[1]) ** 2
        )


if __name__ == "__main__":
    rospy.init_node("task_generator")
    
    rospy.wait_for_service("/static_map")

    task_generator = TaskGenerator()
    
    rospy.spin()
