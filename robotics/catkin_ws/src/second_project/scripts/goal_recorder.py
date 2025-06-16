#!/usr/bin/env python3

import rospy
import csv
import tf
import math
from geometry_msgs.msg import PoseStamped

class GoalRecorder:
    def __init__(self):
        rospy.init_node("goal_recorder", anonymous=True)
        self.csv_file = rospy.get_param("~csv_file", "goals.csv")
        self.file = open(self.csv_file, mode='w', newline='')
        self.writer = csv.writer(self.file)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
        rospy.loginfo("Goal Recorder started. Click goals in RViz.")

    def callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        theta_deg = math.degrees(yaw)
        self.writer.writerow([round(x, 2), round(y, 2), round(theta_deg, 2)])
        rospy.loginfo(f"Goal saved: x={x:.2f}, y={y:.2f}, theta={theta_deg:.2f}°")

    def __del__(self):
        self.file.close()
        rospy.loginfo("CSV file closed.")

if __name__ == "__main__":
    GoalRecorder()
    rospy.spin()
