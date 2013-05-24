#!/usr/bin/env python

# mainbrain topic
# arduino topic

import os
import roslib; roslib.load_manifest('odor_plume_calibration')
import rospy
from ros_flydra.msg import *
from std_msgs.msg import *
from odor_plume_calibration.srv import *
import time

import pickle

from optparse import OptionParser

import arduino_stepper.arduino_stepper as arduino_stepper         
import arduino_stepper.stepper_functions as stepper_functions           


# rosrun ros_arduino_daq arduino_daq.py --port='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_74133353537351109202-if00' --streaming-delay=2000 --topic='pid_data'

# rosrun odor_plume_calibration harvest_data.py --path='/media/967455ca-a716-4b83-a3a3-a802f2e16ab9/odor_plume_calibration_data'

if __name__ == '__main__':
    
    tstart = time.time()
    
    vel = 300
    
    port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_64131383231351814122-if00'
    baudrate = 19200
    astep = arduino_stepper.Arduino_Stepper(port=port,timeout=1, baudrate=19200)
    if not astep.is_connected:
        print 'Bad connection, try unplugging and plugging back in!'
    else:
        
        rospy.init_node('run_stepper', anonymous=True)
        print 'starting data collection'
        
        rospy.wait_for_service("pid_data_control")
        pid_data_control = rospy.ServiceProxy("pid_data_control", HarvestDataService)
        turn_on_pid_streaming = pid_data_control(1)

        rospy.wait_for_service("harvester_control")
        harvester_control = rospy.ServiceProxy("harvester_control", HarvestDataService)
        
        turn_on_data_collection = harvester_control(1)
        
        print 'starting stepper motors'
        stepper_functions.bounce_between_two_limit_switches(sign=-1, ncycles=50, vel=vel, astep=astep)
        print 'stopping data collection'
        
        turn_off_data_collection = harvester_control(0)
        turn_off_pid_streaming = pid_data_control(0)
        
        
    print 
    print 'time elapsed: ', (time.time()-tstart)/60., ' min'
