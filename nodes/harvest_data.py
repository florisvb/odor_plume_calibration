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

class Harvester:
    def __init__(self, data_destination=''):
        self.data_destination = data_destination
        self.record_data = 0
        self.pid_data = 0
        self.data = {'pid': [], 'flydra_position': [], 'flydra_velocity': []}
        
        # set up subscriber to pid data via arduino
        rospy.Subscriber("pid_data", Int32, self.pid_data_callback)
        
        # set up subscriber to flydra mainbrain
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.callback)
        
        # set up controller
        self.service = rospy.Service('harvester_control', HarvestDataService, self.controller_callback)
        
        rospy.init_node('harvester', anonymous=True)
        
        
        rospy.spin()
        
    def controller_callback(self, data):
        if data.action == 0:
            print 'data recording off'
            self.record_data = False
            filename = time.strftime("pid_flydra_data_%Y%m%d_%H%M%S",time.localtime())
            filename_with_path = os.path.join(self.data_destination, filename)
            f = open(filename_with_path, 'w')
            pickle.dump(self.data, f)
            f.close()
            return 0
        if data.action == 1:
            print 'data recording on'
            self.data = {'pid': [], 'flydra_position': [], 'flydra_velocity': []}
            self.record_data = True
            return 1
            
    def pid_data_callback(self, data):
        self.pid_data = data.data

    def callback(self, super_packet):
        if self.record_data:
            for packet in super_packet.packets:
                if len(packet.objects) == 1:
                    obj = packet.objects[0]
                    position = [obj.position.x, obj.position.y, obj.position.z]
                    velocity = [obj.velocity.x, obj.velocity.y, obj.velocity.z]
                    self.data['flydra_position'].append(position)
                    self.data['flydra_velocity'].append(velocity)
                    self.data['pid'].append(self.pid_data)
                    

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--path", type="string", dest="path", default='',
                        help="file path where we should store data files")
    (options, args) = parser.parse_args()
    
    harvester = Harvester(options.path)
    
    
    
    
