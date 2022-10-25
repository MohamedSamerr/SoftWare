#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64,Float32,Int16    

encoder_counts = 0
setpoint = 0

error = 0
last_error = 0

measured_value =0
last_measured_value = 0

#limits
max_lim = 255
min_lim = -255 
max_lim_integral = 0
min_lim_integral = 0

#cutoff frequency
frequency = 5            
tau = 1/frequency

#constants
Kp = 100
Ki = 100
Kd = 1

P = 0
I = 0
D = 0
#sampling time in seconds
T = 1/100    
#calculating rpm
pulses_per_rotation = 540
newcount  = 0
lastcount = 0

##################################################################################
##################################################################################
##################################################################################
pub = rospy.Publisher('/PWM_Values',Int16,queue_size=100)

def start():
    rospy.init_node('CONTROL')
    rospy.loginfo("node has started")
    rospy.Subscriber("/encoder" , Int64 , callback=callback1)
    rospy.Subscriber("/setpoint" , Float32 , callback=callback)
    timer = rospy.Timer(rospy.Duration(T),PID)
    rospy.spin()  
    timer.shutdown()


def callback(data : Int64): 

    global encoder_counts

    encoder_counts = data.data

def callback1(data : Float32) :

    global setpoint

    setpoint=data.data    
##################################################################################
##################################################################################
##################################################################################
def PID(self):
    global setpoint,encoder_counts,error,last_error,measured_value,last_measured_value

    global max_lim ,min_lim ,max_lim_integral ,max_lim_integral

    global pulses_per_rotation ,lastcount ,newcount

    global T , tau , Kp , Ki , Kd , P , I , D , pwm
    #calculating rpm"measured_value"
    newcount = encoder_counts
    countpertime = (newcount-lastcount)/(T)
    revolutions = (countpertime)/pulses_per_rotation
    measured_value = revolutions / 60 
    lastcount = newcount 
  

    error = setpoint - measured_value

    #proprtional
    P = Kp * error

    #Integral
    I = I + 0.5 * Ki * T * (error + last_error)

    #Integrator anti-windup

    if(max_lim > P):

        max_lim_integral = max_lim - P

    else:

        max_lim_integral = 0    


    if(min_lim < P):

        min_lim_integral = min_lim - P

    else:

        min_lim_integral = 0


    if(I > max_lim_integral):

        I = max_lim_integral

    elif(I < min_lim_integral):

        I = min_lim_integral   

    #derivative + low pass filter ("band limited diffrentiator")  
                # change in measurment to solve derivative kick
    D = (2 * Kd * (measured_value - last_measured_value) + (2 * tau - T) * D) / (2 * tau + T)

    #calculate pwm signal then send it to STM32
    pwm = P + I + D

    if (pwm > max_lim):

        pwm = max_lim

    elif (pwm < min_lim):

        pwm = min_lim

    pub.publish(pwm)

    #updating last_error for next iteration   
    last_error = error

    last_measured_value = measured_value

##################################################################################

if __name__ == '__main__':

    try:
        start()
    except rospy.ROSInterruptException:
        pass
    


   