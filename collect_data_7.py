

import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import pickle

class collect_data :
    velocity_buffer = []
    
    
    def callback(self, msg):
        # print(msg.twist.twist.linear.x)
        self.velocity_buffer.append([msg.header.stamp.secs + 1e-9* msg.header.stamp.nsecs,msg.twist.twist.linear.x, msg.twist.twist.angular.z])

    def __init__(self):
        rospy.Subscriber('rover_7/vesc/odom', Odometry, self.callback)
        


    


if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    CD = collect_data()
    print("collecting_data ", len(CD.velocity_buffer))
    rospy.spin()
    if rospy.is_shutdown():
        file = open('velocity-risk-ego-rover-7-overtake_7.dat', 'wb')

        # dump information to that file
        pickle.dump(CD.velocity_buffer, file)
        file.close()
