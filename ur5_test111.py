#!/usr/bin/python
# Gazebo UR5
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from math import *
from random import uniform
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# [0.0, -pi/2, pi/2, pi/3, 0, -pi/10]
# waypoints = [[uniform(-pi, pi) for _ in range(0,6)], [0,0,0,0,0,0]]

def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=100)


    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(10)
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()
    
    while not rospy.is_shutdown():
	joint_angle_mat = load_file("/home/nimblekun/kun_ur5test/src/ur5_test/scripts/kun_5.txt")
	cnt = 0
	for joint_angle in joint_angle_mat:
            pts.positions = joint_angle
	    pts.time_from_start = rospy.Duration(0.1)
	    # Set the points to the trajectory
	    traj.points = []
	    traj.points.append(pts)
	    # Publish the message
	    pub.publish(traj)
	    cnt+=1
	    print(cnt)
	    #tra()
	    rate.sleep()

def load_file(file_path):
    num_mat = []
    with open(file_path) as file:
        for line in file.readlines():
            line = line.rstrip()
            num_list = line.split("\t")
            num_list = [float(i) for i in num_list]
            num_mat.append(num_list)
        file.close()
    return num_mat


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
