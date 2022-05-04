#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('publicador')
    pub = rospy.Publisher('publicacion', String, queue_size=1000)
    rate = rospy.Rate(1)

    count = 0
    rospy.loginfo('Practica P-S')

    while not rospy.is_shutdown():
        msg = String()
        p_string = 'Cuenta: ' + str(count)
        msg.data = p_string
        rospy.loginfo(msg.data + ' seg')
        pub.publish(msg)
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        main()
    except:
        pass

