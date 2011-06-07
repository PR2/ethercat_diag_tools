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
from ethercat_device_status import EtherCATDeviceMissing, EtherCATDevicePortMissing
import rosbag
import sys 
import os
import time
import rospy
import copy
from ethercat_monitor.ethercat_device_diag import EtherCATDeviceDiag, EtherCATDeviceAddDiag
from ethercat_monitor.ethercat_master_diag import EtherCATMasterDiag

from ethercat_monitor.util import prettyTimestamp, prettyDuration

import ethercat_monitor.msg


def mergeDevices(devices1, devices2):
    """ creates a dict of device tuple based on the device name """
    dev_map = {}
    for dev in devices1:
        if dev.name in dev_map:
            raise Exception('Device with name %s in list twice' % dev.name)
        else:
            dev_map[dev.name] = [dev,None]
        
    for dev in devices2:
        if dev.name in dev_map:
            dev_pair = dev_map[dev.name]
            if (dev_pair[1] != None):
                raise Exception ('Device with name s% in list twice' % dev.name)
            dev_pair[1] = dev
        else:
            dev_map[dev.name] = [None,dev]

    return dev_map


def getPortDiff(port_new, port_old):
    """ Returns difference in error counters between this and old values"""
    port_diff = ethercat_monitor.msg.EtherCATDevicePortStatus()
    port_diff.rx_errors = port_new.rx_errors - port_old.rx_errors
    port_diff.forwarded_rx_errors = port_new.forwarded_rx_errors - port_old.forwarded_rx_errors
    port_diff.frame_errors = port_new.frame_errors - port_old.frame_errors
    port_diff.lost_links = port_new.lost_links - port_old.lost_links
    port_diff.est_drops = port_new.est_drops - port_old.est_drops

    if port_new.open != port_old.open:
        port_diff.open = None
    else:
        port_diff.open = port_new.open
    return port_diff



def getDeviceDiff(ds_new, ds_old):        
    """ Gets (error) difference between new and old device """     
    num_ports = max(len(ds_new.ports), len(ds_old.ports))
    ds_diff = ethercat_monitor.msg.EtherCATDeviceStatus()
    if ds_new.name != ds_old.name:
        raise Exception("Name of old and new device does not match")
    ds_diff.name = ds_new.name
    ds_diff.epu_errors = ds_new.epu_errors - ds_old.epu_errors
    ds_diff.pdi_errors = ds_new.pdi_errors - ds_old.pdi_errors
    ds_diff.valid = ds_new.valid and ds_old.valid
    if ds_new.hardware_id == ds_old.hardware_id:
        ds_diff.hardware_id = ds_new.hardware_id
    else:
        ds_diff.hardware_id = "Mismatch %s != %s" % (ds_new.hardware_id, ds_old.hardware_id)
    for num in range(num_ports):
        if (num >= len(ds_new.ports)) or (num >= len(ds_old.ports)):
            ds_diff.ports.append(EtherCATDevicePortMissing())
        else:
            ds_diff.ports.append(getPortDiff(ds_new.ports[num], ds_old.ports[num]))
    if ds_new.ring_position != ds_old.ring_position:
        ds_diff.ring_position = None
    else:
        ds_diff.ring_position = ds_new.ring_position
    return ds_diff


def getMasterDiff(new, old):
    """ Returns difference in counters between new and old master structures"""
    diff = ethercat_monitor.msg.EtherCATMasterStatus()
    diff.sent    = new.sent    - old.sent
    diff.dropped = new.dropped - old.dropped
    diff.late    = new.late    - old.late
    diff.unassigned_drops = new.unassigned_drops - old.unassigned_drops
    return diff


class DropEstimator:
    """ Assigns packet drops to one or more EtherCAT devices.
    Usually frame errors counters increase a cycle after a packet drops occurs. 
    To handle this, this keeps some history to properly assign packet 
    drops to the correct device.
    """
    def __init__(self):
        self.last_unassigned_drops = 0.0
        self.timestamp_data_old = None
        self.cycle_count = 0
    
    def process(self, timestamp_data):        
        """ Takes EtherCAT device and Master data and calculates 
            estimate dropped packets for each EtherCAT device port"""

        tsd_new = timestamp_data
        tsd_old = self.timestamp_data_old
        if tsd_old is None:
            self.timestamp_data_old = timestamp_data
            return 
        
        # This is a little sloppy because sometimes packets are counted as
        # late before they are counted as dropped, however this situation is
        # not incredibly common 
        new_master = tsd_new.getMaster()
        old_master = tsd_old.getMaster()
        dropped_new = new_master.dropped - new_master.late
        dropped_old = old_master.dropped - old_master.late
        drops = dropped_new - dropped_old

        # determine which device caused dropped packets, by looking for devices
        # where frame error count increased.  If multiple devices have frame errors
        # at same time, split dropped between devices based on how much frame error
        # increased for each device

        # Create list of port pairs between old and new data
        # port_pairs list is tupple of (device_name, port_number, port_new, port_old)
        dev_map = mergeDevices(tsd_new.getDevices(), tsd_old.getDevices())
        port_pairs = []
        for name, (dev_new, dev_old) in dev_map.iteritems():
            if (dev_new is None) or (dev_old is None):
                print "Warning, no device pair for device %s" % name
            else:
                if len(dev_old.ports) != len(dev_new.ports):
                    print "Number of ports do not match beween old (%d) an new (%d) devices" % (len(dev_old.ports), len(dev_new.ports))
                else:
                    for port_num in range(len(dev_new.ports)):                        
                        port_pairs.append( (name, port_num, dev_new.ports[port_num], dev_old.ports[port_num]) )


        # sum new frame_errors for all devices
        frame_error_sum = 0.0  # should be float
        for dev_name, port_num, port_new, port_old in port_pairs:
            frame_error_sum += port_new.frame_errors - port_old.frame_errors

        # multiple devices might have a frame_error for same packet 
        # However, only one packet is dropped from master's point of view.
        # There is also the possibility that the packet is corrupted (dropped) on the 
        # way back to the computer which would not generate any frame errors

        # First assign each unassigned drops from last cycle to frame errors for this cycle
        # Any remaining drops from last cycle will be given to ethercat master as unassigned drops
        device_drops = min(self.last_unassigned_drops, frame_error_sum)
        master_unassigned_drops = self.last_unassigned_drops - device_drops
        remaining_frame_error_sum  = frame_error_sum - device_drops
        new_master.unassigned_drops = old_master.unassigned_drops + master_unassigned_drops

        # take remaining drop and assign them to frame errors from this cycle
        # any remaining drops get passed to next cycle
        unassigned_drops = max(drops - remaining_frame_error_sum, 0)
        device_drops += drops - unassigned_drops

        self.cycle_count += 1
        if (device_drops > 0.0) or (frame_error_sum > 0.0) or (drops > 0.0) or (unassigned_drops > 0.0) or (self.last_unassigned_drops > 0.0):
            print
            print "cycle", self.cycle_count
            print "frame_error_sum", frame_error_sum
            print "drops", drops
            print "device_drops", device_drops
            print "remaining_frame_error_sum", remaining_frame_error_sum
            print "master_unassigned_drops", master_unassigned_drops
            print "last_unassigned_drops", self.last_unassigned_drops
            print "unassigned_drops", unassigned_drops
            print

        self.last_unassigned_drops = unassigned_drops

        if device_drops > 0:
            # divide drops against all devices with increased frame_error count            
            for dev_name, port_num, port_new, port_old in port_pairs:
                frame_errors = port_new.frame_errors - port_old.frame_errors
                port_new.est_drops = port_old.est_drops + device_drops * (frame_errors / frame_error_sum)    
        else:
            for dev_name, port_num, port_new, port_old in port_pairs:
                port_new.est_drops = port_old.est_drops

        self.timestamp_data_old = tsd_new




class EtherCATHistoryTimestepData:
    def __init__(self,msg_header):
        self.timestamp = msg_header.stamp
        self.msg_header = msg_header
        self.timestamp_old = None
        self.system = ethercat_monitor.msg.EtherCATSystemStatus()
        self.system.header = msg_header
        self.has_master = False

    def hasData(self):        
        if (len(self.system.devices) > 0) and self.has_master:
            return True
        elif (len(self.system.devices) == 0) and not self.has_master:
            return False
        elif (len(self.system.devices) > 0) and not self.has_master:
            raise RuntimeError("Devices but no master")
        else:
            raise RuntimeError("Master but no devices")

    def numDevices(self):
        return len(self.system.devices)

    def addDevice(self, device_status):        
        self.system.devices.append(device_status)        

    def addMaster(self, master):
        if self.has_master:
            raise RuntimeError("Master data already set")        
        self.system.master = master
        self.has_master = True

    def __str__(self):
        result = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.timestamp.to_sec())) + '\n'
        result += "Master\n" + str(self.master)
        for name,device in self.devices.iteritems():
            result += " " + name + " : \n" + str(device)
        result += '--\n'
        return result


    def getDevices(self):
        return self.system.devices

    def getMaster(self):
        return self.system.master


    def getDiff(self, timestamp_data_old):
        """ returns new EthercatHistoryTimestampData that represents difference
        between both timesteps"""

        tsd_old = timestamp_data_old
        tsd_diff = EtherCATHistoryTimestepData(self.msg_header)
        tsd_diff.timestamp_old = tsd_old.timestamp

        tsd_diff.addMaster(getMasterDiff(self.getMaster(), tsd_old.getMaster()))

        dev_map = mergeDevices(self.getDevices(), tsd_old.getDevices())

        #First match every device in this list to device in old data
        for name, (dev_new, dev_old) in dev_map.iteritems():
            if (dev_new is None) or (dev_old is None):
                print "Device '%s' not found" % name
                tsd_diff.addDevice(EtherCATDeviceMissing(name))
            else:
                dev_diff = getDeviceDiff(dev_new, dev_old)
                tsd_diff.addDevice(dev_diff)

        return tsd_diff



    
    def generateYaml(self):        
        master = self.system.master
        master_out = {'sent':master.sent, 'dropped':master.dropped, 'late':master.late}

        devices_out = {}
        for dev in self.system.devies:
            dev_out = {'valid':dev.valid, 'position':dev.ring_position }
            dev_out['epu_errors']=dev.epu_errors
            dev_otu['pdi_errors']=dev.pdi_errors
            for port in dev.ports:
                port_out = {'rx_errors':port.rx_errors}
                port_out['forwarded_rx_errors'] = port.forwarded_rx_errors
                port_out['frame_errors'] = port.frame_errors
                port_out['lost_links'] = port.lost_links
            devices_out[dev.name] = dev_out
        out['devices'] = devices_out

        out['master':master_out, 'devices':devices_out]
        out['devices'] = device_out

        if self.timestamp_old is not None:
            duration = self.timestamp - self.timestamp_old
            out['duration'] = prettyDuration(duration)
        else:
            out['date'] = prettyTimestamp(self.timestamp)
            out['ros_time'] = {'secs':self.timestamp.secs, 'nsecs':self.timestamp.nsecs}

        
        return out



class EtherCATHistoryTimestepDataNote:
    """ Represents save timestamp data with message attached to it """
    def __init__(self, timestep_data, note_msg):
        self.timestep_data = timestep_data
        self.note_msg = note_msg


class EtherCATHistory:
    def __init__(self):
        self.END = "END"
        self.BEGIN = "BEGIN"
        self.lock = threading.Lock()
        self.subscription = None
        self.reset()        
        self.drop_estimator = DropEstimator()
        self.sample_interval = rospy.Duration(60)  # default to sampling data every 60 seconds
        self.last_sample_time = None

    def reset(self):
        with self.lock:
            # list of automatically saved ethercat timestep data.  
            # New data is saved to history every minute.  
            # history allows user to go back in time to view events that might have been missed
            self.history = []
            # Notes are timestamp data that have been explicity saved by user
            # the note part is a message that the user provides for the timestamp data
            self.notes = []
            self.newest_tsd = None
            self.other_diag_map = {}
            self.device_diag_map = {}
            self.master_diag     = EtherCATMasterDiag()
            self.device_add_diag = EtherCATDeviceAddDiag(self.device_diag_map)
            if self.subscription is not None:
                self.subscription.unregister()
                self.subscription = None                

    def addTimestepData(self, timestep_data):
        with self.lock:
            history = self.history
            if (len(history) > 0) and (timestep_data.timestamp < history[-1].timestamp):
                raise RuntimeError("New data is older than previous data in history")            
            self.newest_tsd = timestep_data
            self.drop_estimator.process(timestep_data)
            # save sample timestep data every minute to history            
            if self.last_sample_time is None:
                # first sample
                history.append(timestep_data)
                self.notes.append(EtherCATHistoryTimestepDataNote(timestep_data, "First message"))
                self.last_sample_time = timestep_data.timestamp + self.sample_interval
                print "first sample"
            elif (timestep_data.timestamp - self.last_sample_time) >= self.sample_interval:
                history.append(timestep_data)                
                self.last_sample_time += self.sample_interval
                print "sample %d" % len(history)
            
            if (self.last_sample_time - timestep_data.timestamp) > self.sample_interval:
                print "Warning, last_sample_time < current timestamp"
                self.last_sample_time = timestep_data.timestamp


    def addNote(self, note):
        with self.lock:
            self.notes.append(note)

    def getNotes(self):
        # make shallow copy of notes list before returning them to user
        with self.lock:
            notes = copy.copy(self.notes)
        return notes

    def getTimestepData(self, timestamp):
        with self.lock:            
            if len(history) == 0:
                return None
            if (timestamp is self.END):
                return self.newest_tsd
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
        timestep_data = EtherCATHistoryTimestepData(msg.header)
        for status in msg.status:
            if status.name in self.device_diag_map:
                dev_status = self.device_diag_map[status.name].process(status)
                timestep_data.addDevice(dev_status)
            elif status.name in self.other_diag_map:
                pass # Ingore anything filed in other
            elif self.master_diag.isMatch(status):
                master_status = self.master_diag.process(status)
                timestep_data.addMaster(master_status)
            elif self.device_add_diag.is_match(status):
                dev_status = self.device_add_diag.process(status)
                timestep_data.addDevice(dev_status)
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
            return self.newest_tsd

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
        

    def saveBag(self, outbag_filename):
        # make shallow copy of history, so it can continue to get new data while we saving current data
        with self.lock:
            history = copy.copy(self.history)

        outbag = rosbag.Bag(outbag_filename, 'w', compression=rosbag.Compression.BZ2)
        for tsd in history:
            outbag.write('ethercat_system_status', tsd.system, t=tsd.system.header.stamp)
        print "Saved %d messages to history" % len(history)
        outbag.close()
        

    def printHistory(self):
        with self.lock:
            for timestep_data in self.history:
                print timestep_data
            print "Length = %d" % len(self.history)
        
