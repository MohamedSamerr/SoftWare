#!/usr/bin/env python3
import rospy
# std_msgs should be in package.xml file
from std_msgs.msg import Float64    # type of msg of imu yaw angle

#intial values 
estimate =  0        # prediction for intial measured value    
estimate_error = 0.5 # difference between estimate and actual value
measure_error  = 1.0# from datasheet or average of errors in multiple trials
q = 0.1
 


def yaw_callback(MEASURED : Float64):
    

    #KALMAN FILTER
    global estimate_error
    global measure_error
    global estimate
    global q
    KG =  estimate_error/(estimate_error+measure_error)   # Kalman Gain

    last_estimate = estimate
    estimate = last_estimate + KG*(MEASURED.data-last_estimate) #new estimate value

    pub.publish(estimate)  # publish new estimate value on "/filtered" topic

    last_estimate_error = estimate_error
    estimate_error = (1-KG)*last_estimate_error
    estimate_error = estimate_error + q



if __name__ == '__main__':

    rospy.init_node("kalman_1D")  

    rospy.loginfo("node has started")
     
                         #("name of new filtered topic" , data type)
    pub = rospy.Publisher ("/filtered" , Float64 , queue_size=10)
                         #("name of imu topic from previous task" , data type )
    sub = rospy.Subscriber("/IMU" , Float64 , callback=yaw_callback)

    rospy.spin()   # doesnt allow the node to shutdown instantly
                   # should be last line in program