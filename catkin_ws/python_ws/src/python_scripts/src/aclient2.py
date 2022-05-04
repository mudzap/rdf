#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def doneCb(state, res):
    rospy.loginfo('Se recibio resultado')
    rospy.loginfo('Codigo de error: ' + str(res.error_code))

def activeCb():
    print('active')
    pass

def feedbackCb(fb):
    print('Current pos:')
    for pos in fb.actual.positions:
        print(' ' + str(pos))
    print('')

def main():
    rospy.loginfo('Iniciando nodo')
    rospy.init_node('ur_5_action') 

    rospy.loginfo('Definiendo cliente')
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory/', FollowJointTrajectoryAction)

    rospy.loginfo('Esperando a servidor')
    client.wait_for_server(rospy.Duration.from_sec(5.0))

    rospy.loginfo('Definiendo objetivo')
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    points = [JointTrajectoryPoint()]
    points[0].positions = [0.0, -0.7, -1.5, 0.0, 0.0, 0.0]
    points[0].time_from_start = rospy.Duration.from_sec(5.0)

    goal.trajectory.points = points

    rospy.loginfo('Enviando objetivo')
    rospy.loginfo(goal)
    client.send_goal(goal, doneCb, activeCb, feedbackCb)

    rospy.loginfo('Spinning')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        pass

