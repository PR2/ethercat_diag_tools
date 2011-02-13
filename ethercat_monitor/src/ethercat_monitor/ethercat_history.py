#! /usr/bin/env python

#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

##\author Derek King
##\brief Keeps a history of EtherCAT counters.  History can be indexed by time and also pruned

import threading

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from ethercat_device_status import EtherCATDeviceMissing
import rosbag
import sys 
import os
import time
import rospy
from ethercat_monitor.ethercat_device_diag import EtherCATDeviceDiag, EtherCATDeviceAddDiag
from ethercat_monitor.ethercat_master_diag import EtherCATMasterDiag


class EtherCATHistoryTimestepData:
    def __init__(self,timestamp):
        self.timestamp = timestamp
        self.devices = {}
        self.master = None

    def hasData(self):
        if (len(self.devices) > 0) and (self.master is not None):
            return True
        elif (len(self.devices) == 0) and (self.master is None):
            return False
        elif (len(self.devices) > 0) and (self.master is None):
            raise RuntimeError("Device but no master")
        else:
            raise RuntimeError("Master but no devices")

    def numDevices(self):
        return len(self.devices)

    def addDevice(self, name, device_status):
        self.devices[name] = device_status

    def addMaster(self, master):
        if self.master is not None:
            raise RuntimeError("Master data already set")
        self.master = master

    def __str__(self):
        result = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.timestamp.to_sec())) + '\n'
        result += "Master\n" + str(self.master)
        for name,device in self.devices.iteritems():
            result += " " + name + " : \n" + str(device)
        result += '--\n'
        return result

    def getDeviceGrid(self):
        #first make list of tuples (position, name, device) from devices
        ordered_devices = []
        for name,device in self.devices.iteritems():
            ordered_devices.append( (device.ring_position, name, device) )
        ordered_devices.sort()
        data = []
        for position,name,device in ordered_devices:
            data += device.getDataGrid(name)
        return data

    def getMasterGrid(self):
        return self.master.getDataGrid()


    def getDiff(self, timestamp_data_old):
        """ returns new EthercatHistoryTimestampData that represents difference
        between both timesteps"""

        tsd_old = timestamp_data_old
        tsd_diff = EtherCATHistoryTimestepData(self.timestamp)

        tsd_diff.master = self.master.getDiff(tsd_old.master)

        #First match every device in this list to device in old data
        for name,device_now in self.devices.iteritems():
            if name in tsd_old.devices:
                device_old = tsd_old.devices[name]
                device_diff = device_now.getDiff(device_old)
                tsd_diff.addDevice(name, device_diff)
                # use device function to get difference
            else:
                #For any devices that are not in current data set, use special type
                # to indicate the missmatch
                print "Device '%s' not found in old data" % name
                tsd_diff.addDevice(name, EtherCATDeviceMissing())

        # Include any device in old timestamp that is not in new timestamp
        for name in tsd_old.devices.iterkeys():
            if name not in self.devices:
                print "Device '%s' not found in new data" % name
                tsd_diff.addDevice(name, EtherCATDeviceMissing())

        return tsd_diff


class EtherCATHistory:
    def __init__(self):
        self.END = "END"
        self.BEGIN = "BEGIN"
        self.lock = threading.Lock()
        self.subscription = None
        self.reset()

    def reset(self):
        with self.lock:
            self.history = []
            self.other_diag_map = {}
            self.device_diag_map = {}
            self.master_diag     = EtherCATMasterDiag()
            self.device_add_diag = EtherCATDeviceAddDiag(self.device_diag_map)
            if self.subscription is not None:
                self.subscription.unregister()
                self.subscription = None

    def addTimestepData(self, timestep_data):
        with self.lock:
            if (len(self.history) > 0) and (timestep_data.timestamp < self.history[-1].timestamp):
                raise RuntimeError("New data is older than previous data in history")
            #self.history.append(timestep_data)
            # For now just remember most recent data, instead of storing history.
            self.history = [timestep_data]

    def getTimestepData(self, timestamp):
        with self.lock:
            if len(history) == 0:
                return None
            if (timestamp is self.END):
                return self.history[-1]
            elif (timestamp is self.BEGIN):
                return self.history[0]
            else:
                # TODO, history list is ordered by time, use binary search to make things faster
                for data in self.history:
                    if data.timestamp >= timestamp:
                        return data
                return self.history[-1]
            

    def processDiagnosticsMsg(self, msg):
        """ Process message. Message contains array of data that may 
        include devices and master
        """
        t = msg.header.stamp 
        timestep_data = EtherCATHistoryTimestepData(t)
        for status in msg.status:
            if status.name in self.device_diag_map:
                dev_status = self.device_diag_map[status.name].process(status)
                timestep_data.addDevice(status.name, dev_status)
            elif status.name in self.other_diag_map:
                pass # Ingore anything filed in other
            elif self.master_diag.isMatch(status):
                master_status = self.master_diag.process(status)
                timestep_data.addMaster(master_status)
            elif self.device_add_diag.is_match(status):
                dev_status = self.device_add_diag.process(status)
                timestep_data.addDevice(status.name,dev_status)
            else:
                self.other_diag_map[status.name] = None
        if timestep_data.hasData():
            #print timestep_data
            self.addTimestepData(timestep_data)


    def subscribeToDiagnostics(self, diagnostics_topic_name):
        if self.subscription is not None:
            self.reset()
        self.subscription = rospy.Subscriber(str(diagnostics_topic_name), DiagnosticArray, self.processDiagnosticsMsg)

    def getNewestTimestepData(self):
        with self.lock:
            return self.history[-1] if (len(self.history) > 0) else None

    def processBagInternal(self, bag, diag_list, diag_map):
        t = None
        for topic, msg, unused in bag.read_messages(topics=['diagnostics','/diagnostics']):
            t = msg.header.stamp
            self.processDiagnosticsMsg(msg)
            
        if t is not None:
            print "Log ends %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec()))
            print "Log ends %f" % t.to_sec()
        else:
            print "bag file contains no diagnostics data"

        
    def processBag(self, inbag_filename, diag_list, diag_map):
        if not os.path.isfile(inbag_filename): 
            raise RuntimeError("Cannot locate input bag file %s" % inbag_filename)

        bag = rosbag.Bag(inbag_filename)
        print bag

        # TODO : start separate thread to process bag

        self.process_bag_internal(bag,diag_list,diag_map)
        

    def printHistory(self):
        with self.lock:
            for timestep_data in self.history:
                print timestep_data
            print "Length = %d" % len(self.history)
        
