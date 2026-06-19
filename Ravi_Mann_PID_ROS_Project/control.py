#!/usr/bin/env python
import rospy  # Importing the Rospy library helps us to listen to ros topics and create our own nodes
from nav_msgs.msg import Path, Odometry  # Import Path and Odometry Message types
from ackermann_msgs.msg import AckermannDriveStamped  # Final control output message
import math  # for math functions
import tf    # conversion between Euler and Quaternion
import sched
import time
import signal
import sys


speed = 0.65         # constant speed
kp = 3.2             # proportional gain
ki = 0.05             # integral gain
kd = 0.2             # derivative gain

# Error clamping & anti-windup limits
max_error = 0.3 # radian (17*) max heading error

max_integral = 0.3 # clam on e of dt

max_steering = 0.43

min_steering = -0.43


prev_error = 0.0
integral_error = 0.0
prev_time = time.time()

path_theta = 0.0
car_theta = 0.0

def signal_handler(sig, frame):  # This function is called when ctrl+c is pressed 
    
    print('Stopping car and exiting')  
    odom_sub.unregister()        # Unregister from /odom topic
    plan_sub.unregister()        # Unregister from /move_base/TrajectoryPlannerROS/local_plan topic

    for i in range(4):

        d_msg = AckermannDriveStamped()   # Create an object Variable of the AckermannDriveStamped Class ( Note: use this messagetype to publish the steering angle ro ROS ) 
        d_msg.drive.speed = 0.0           # Sets speed to zero
        d_msg.drive.steering_angle = 0.0  # Sets Steering Angle
        pub.publish(d_msg)                # Publish this message 

    time.sleep(0.5)                       # Pause program for half a second
    sys.exit(0)                           # Close the program and exit



def constrain(val, min_val, max_val):     # Constrains a value and returns it as output when the three arguments are passed  
    return min(max_val, max(min_val, val))



def odomCallback(msg):                     # This function gets called everytime the car odometery data is published
    global car_theta , car_theta_deg,  prev_error, prev_time, speed, max_steering, min_steering
        
    # Current position
    car_x, car_y =  msg.pose.pose.position.x, msg.pose.pose.position.y
    car_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y , msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    car_theta = tf.transformations.euler_from_quaternion(car_quaternion)[2]    
    """
    Write your PD controller code below this comment section. The Output variable should be the steering angle of the car. Tune kp and kd 		coefficients to get better control. Hint: Input error is difference between car_theta and path_theta. Code need not be complex. It can 		be written in about 5 lines or so. 
    """

    # error between path direction and car heading
    error = path_theta - car_theta
    
    
    # wrap error into [-pi, pi]
    error = math.atan2(math.sin(error), math.cos(error))

    # Clamp extreme look-ahead errors
    error = constrain(error, -max_error, max_error)


    # time since last callback
    now = time.time()
    dt = now - prev_time if (now - prev_time) > 1e-6 else 1e-6
    prev_time = now

    #anti windup integral update
    integral_error = constrain(integral_error + error * dt, -max_integral, max_integral)    


    # derivative term
    derivative = (error - prev_error) / dt
    prev_error = error
    
    

    # PID control law
    steering = kp * error + ki * integral_error + kd * derivative
    # enforce steering limits
    steering = constrain(steering, min_steering, max_steering)

    # debug prints
    #print "error = ", error
    #print "integral = ", integral_error
    
    #print "derivative = ", derivative
    #print "steer = ", steering

	# Checking Distance to Goal
    goal_x, goal_y = -15.7325611115, 4.067029953 # hardcoded value same as the one used when setting car pose and goal in the 		set_goal_andpose.py
    distance_to_goal = math.sqrt( (goal_x - car_x)*(goal_x - car_x) + (goal_y - car_y)*(goal_y - car_y) )
    print "distance_to_goal", distance_to_goal 


    if ( distance_to_goal < 0.1 ):    #i.e if car has reached goal
        print "Reached Goal!! Stopping Car!"
        odom_sub.unregister() #unsubscribe from /odom
        plan_sub.unregister() #unsubsrcibe from /move_base/TrajectoryPlannerROS/local_plan

        for i in range(4):  # repeat publish loop 4 times  
            print "Reached Goal!! Stopping Car!"
            """
            Write code to publish an ackermann message to stop the car. About 4 lines of code should do it.
            """
            d_msg = AckermannDriveStamped()		
            d_msg.drive.steering_angle = 0.0
            d_msg.drive.speed = 0.0
            pub.publish(d_msg)
        

    else: # this part runs for the usual running of the car when car has not yet reached the goal
    
        """
        Write code to publish an ackermann msg to drive the car with the steering value from the PID controller output and the speed 		    	variable as the speed of the car. About 4 lines of code should do it.  
        """
        d_msg = AckermannDriveStamped()		
        d_msg.drive.steering_angle = steering
        d_msg.drive.speed = speed
        pub.publish(d_msg)


def pathCallback(msg):
	global path_theta  # a global variable can be used inside other functions as well

	index =int(len(msg.poses)/2) # Since the path is usually a curved arc we take a point about halway of the arc and find its orientation
	
	path_quaternion = (msg.poses[index].pose.orientation.x, msg.poses[index].pose.orientation.y, msg.poses[index].pose.orientation.z, 		msg.poses[index].pose.orientation.w)  # extract quaternion from the ros message

	path_theta = tf.transformations.euler_from_quaternion(path_quaternion)[2]  # Path theta gives a general sense of direction of the 		angle of the local path



if __name__ == '__main__':
    s = sched.scheduler(time.time, time.sleep)  #helps in dealing with Ctrl+C input to the terminal
    signal.signal(signal.SIGINT, signal_handler) #helps in dealing with Ctrl+C input to the terminal
    signal.signal(signal.SIGTERM, signal_handler) #helps in dealing with Ctrl+C input to the terminal

    rospy.init_node('Driver_Node',disable_signals=True)  #Initialize node to ROS Master	
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)  #initialize publishing on /drive topic to steer and move the car
    odom_sub = rospy.Subscriber('/odom', Odometry, odomCallback)  #subscribe to odom topic and run OdomCallback function 
    plan_sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, pathCallback) #subscribe to odom topic and run pathCallback function
    while not rospy.is_shutdown():  #while the master is running
    	rospy.spin()   #keep allowing the publishing and callbacks

	
