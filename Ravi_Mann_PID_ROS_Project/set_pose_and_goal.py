#!/usr/bin/env python
import rospy # Rospy helps us with initialis=zing nodes, subscribing and publishing  messages
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped  #importing this message types
import tf #helps with quaternion to euler math



def publisher():
    print("Setting robot pose and goal on topic initialpose and move_base_simple/goal")
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1) # initiate publishing on initialpose topic
    pub2 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1) # initiate publishing on move_base_simple/goal topic
    rospy.init_node('goal_pose_publisher', anonymous=True) #Initialize the python node
    rate = rospy.Rate(2) # Hz
    count = 2   #Makes us publish data multiple times to make sure ROS does not miss it
    while not rospy.is_shutdown():
        p = PoseWithCovarianceStamped() #create object variable for publishing on /initialpose
        p.pose.pose.position.x = -1.0    #set the required object variables
        p.pose.pose.position.y = 0.0
        p.pose.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
        p.pose.pose.orientation.x,p.pose.pose.orientation.y,p.pose.pose.orientation.y,p.pose.pose.orientation.z =  q[0],q[1],q[2],q[3]
        p.pose.covariance = [0.0,0.0,0.0,0.0,0.0,0.0,
0.0,0.0,0.0,0.0,0.0,0.0,
0.0,0.0,0.0,0.0,0.0,0.0,
0.0,0.0,0.0,0.0,0.0,0.0,
0.0,0.0,0.0,0.0,0.0,0.0,
0.0,0.0,0.0,0.0,0.0,0.0] # an example covariance matrix required for publishing with the position and quaternion data
        pub.publish(p) # publish the goal on the topic /initialpose
        time.sleep(0.5) 
        p2 = PoseStamped()  #create object variable for publishing on topic /move_base_simple/goal
        p2.header.frame_id = 'map'  #set the required object variables
        p2.pose.position.x = -15.732
        p2.pose.position.y =  4.067
        p2.pose.position.z = 0.0
        # Make sure the quaternion is valid and normalized
        p2.pose.orientation.x,p2.pose.orientation.y,p2.pose.orientation.z,p2.pose.orientation.w =  0.0, 0.0, 0.999860547532, 0.016699864919
        pub2.publish(p2) # publish the goal on the topic /move_base_simple/goal
        rate.sleep() #regulates sleep to make sure rate is maintained 
        count -= 1
        #print count
        if count < 0: # break the loop if count less than 0
           break
    


if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
