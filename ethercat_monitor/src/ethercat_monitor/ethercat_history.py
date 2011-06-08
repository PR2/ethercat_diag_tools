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


from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from ethercat_device_status import EtherCATDeviceMissing, EtherCATDevicePortMissing
import rosbag
import sys 
import os
import os.path
import time
import rospy
import copy
import yaml
import threading
import multiprocessing

from ethercat_monitor.ethercat_device_diag import EtherCATDeviceDiag, EtherCATDeviceAddDiag
from ethercat_monitor.ethercat_master_diag import EtherCATMasterDiag

from ethercat_monitor.util import prettyTimestamp, prettyDuration
from ethercat_monitor.timestep_data import EtherCATHistoryTimestepData, EtherCATHistoryTimestepDataNote
from drop_estimator import DropEstimator

import ethercat_monitor.msg
import ethercat_monitor.timestep_data
import std_msgs.msg





class EtherCATDiagnosticProcessor:
    def __init__(self):
        self.other_diag_map = {}
        self.device_diag_map = {}
        self.master_diag     = EtherCATMasterDiag()
        self.device_add_diag = EtherCATDeviceAddDiag(self.device_diag_map)
        
    def processDiagnosticsMsg(self, msg):
        """ 
        Process DiagnosticArray ROS message. 
        DiagnosticArray contains array of DiagnosticStatus messages.
        Diagnostics are produced by many different components, so not all 
        DiagnosticArray messages will contain EtherCAT information.   
        
        However, pr2_etherCAT will publish single DiagnosticArrray for
        for EtherCAT Master and all EtherCAT devices.   

        If <msg> is contains EtherCAT master and EtherCAT device information,
        this function creates a TimestepData class for it and appends it to 
        history.
        """
        devices = []
        master = None
        for status in msg.status:
            if status.name in self.device_diag_map:
                devices.append(self.device_diag_map[status.name].process(status))
            elif status.name in self.other_diag_map:
                pass # Ingore anything filed in other
            elif self.master_diag.isMatch(status):
                master = self.master_diag.process(status)
            elif self.device_add_diag.is_match(status):
                devices.append(self.device_add_diag.process(status))
            else:
                self.other_diag_map[status.name] = None

        if master is not None:
            hdr = std_msgs.msg.Header(msg.header.seq, msg.header.stamp, msg.header.frame_id)
            sys_status = ethercat_monitor.msg.EtherCATSystemStatus(hdr, master, devices)
            return sys_status
        else:
            return None




class EtherCATHistory:
    def __init__(self, topic_name=None, bag_filename=None, subscriber=None):
        if (topic_name is not None) and (bag_filename is not None):
            raise Exception("Both topic name and Bag filename cannot both be specified")
        self.topic_name = topic_name
        self.bag_filename = bag_filename

        self.END = "END"
        self.BEGIN = "BEGIN"
        self.lock = threading.Lock()        
        self.drop_estimator = DropEstimator()
        self.sample_interval = rospy.Duration(60)  # default to sampling data every 60 seconds
        self.last_sample_time = None
        #self.diag_processor = EtherCATDiagnosticProcessor()

        # list of automatically saved ethercat timestep data.  
        # New data is saved to history every minute.  
        # history allows user to go back in time to view events that might have been missed
        self.history = []
        # Notes are timestamp data that have been explicity saved by user
        # the note part is a message that the user provides for the timestamp data
        self.notes = []
        self.newest_tsd = None

        if self.bag_filename is not None:
            self.processBag(bag_filename)
        elif self.topic_name is not None:
            self.diag_processor = EtherCATDiagnosticProcessor()
            if subscription is not None:
                self.subscription = subscription
            else:
                self.subscription = rospy.Subscriber(str(topic_name), DiagnosticArray, self.diagnosticsCallback)
            
        #self.subscription.unregister()


    def getTitle(self):
        if self.topic_name is not None:
            return self.topic_name
        elif self.bag_filename is not None:
            return os.path.basename(self.bag_filename)
        else:
            raise Exception("Internal error : both bagfile and topic name unspecified")

    def getStatus(self):
        with self.lock:
            if self.topic_name is not None:
                if self.subscriber is not None:
                    return "Listening for messages"
                else:
                    return "Subscriber closed"
            elif self.bag_filename is not None:
                return self.loading_bag_status
            else:
                raise Exception("Internal error : both bagfile and topic name unspecified")


    def updateSubProc(self):
        """Pulls updates from subprocess and adds them to history 
        Updates are in form of EtherCATSystemStatus messages. 
        Wrap them in timestep data class before adding them to queue
        """
        result = self.connection.recv()
        while result is not None:
            (status_msg, system_msg_list) = result
            with self.lock:
                self.loading_bag_status = status_msg
                self.addEtherCATSystemMsgs(system_msg_list)
            result = self.connection.recv()
        print "subprocess is complete"
        self.subproc.join()
        self.subproc = None

           
    def diagnosticsCallback(self, msg):
        system_msg = self.diag_processor.processDiagnosticMsg(msg)
        if system_msg is not None:
            with self.lock:
                self.loading_bag_status = "Recieving messages"
                self.addEtherCATSystemMsgs([system_msg])


    def addEtherCATSystemMsgs(self, system_msgs_list):    
        for system_msg in system_msgs_list:
            # first wrap EtherCATSytemStatus message in timestep_data structure.
            timestep_data = EtherCATHistoryTimestepData(system_msg)
            # now mak
            history = self.history
            if (len(history) > 0) and (timestep_data.getTimestamp() < history[-1].getTimestamp()):
                raise RuntimeError("New data is older than previous data in history")            
            self.drop_estimator.process(timestep_data)
            self.newest_tsd = timestep_data
            # save sample timestep data every minute to history            
            if self.last_sample_time is None:
                # first sample
                history.append(timestep_data)
                self.notes.append(EtherCATHistoryTimestepDataNote(timestep_data, "First message"))
                self.last_sample_time = timestep_data.getTimestamp() + self.sample_interval
            elif (timestep_data.getTimestamp() - self.last_sample_time) >= self.sample_interval:
                history.append(timestep_data)                
                self.last_sample_time += self.sample_interval
            if (self.last_sample_time - timestep_data.getTimestamp()) > self.sample_interval:
                print "Warning, last_sample_time < current timestamp"
                self.last_sample_time = timestep_data.getTimestamp()


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

    def getNewestTimestepData(self):
        with self.lock:
            return self.newest_tsd
        

    def processBag(self, bag_filename):
        """ 
        Opens and starts processing of <bag_filename>.  
        To avoid blocking for a long time or blogging down GUI (because of GIL) 
        the major part of bagfile processing is done in another process.
        """
        self.loading_bag_status = "Starting loader thread..."
        self.connection, subproc_connection = multiprocessing.Pipe()
        args=(bag_filename,subproc_connection)
        self.subproc = multiprocessing.Process(target=processBagSubProc, name='process_bag', args=args)
        self.subproc.start()

        # use thread to get data from bag file and put it in history structure
        self.thread = threading.Thread(target=self.updateSubProc, name='process_thread')
        self.thread.start()


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
        




def processBagSubProc(bag_filename, connection):
    """ Processes diagnostic message in bag file, puts results in connection as tuple of (status_msg, result_list)"""
    try:
        if not os.path.isfile(bag_filename): 
            raise RuntimeError("Cannot locate input bag file %s" % bag_filename)

        bag = rosbag.Bag(bag_filename)

        # first look at bag file to determine what topic has diagnostic_msgs/DiagnosticArray type
        y = yaml.load(bag._get_yaml_info())
        matching_topics = [ ]
        for topic_info in y['topics']:
            if topic_info['type'] == 'diagnostic_msgs/DiagnosticArray':
                topic_name = topic_info['topic']
                num_msgs = int(topic_info['messages'])
                matching_topics.append( (topic_name, num_msgs) )

        if len(matching_topics) == 0:            
            raise RuntimeError("No diagnostic data in bag file")
        elif len(matching_topics) > 1:
            # todo, fix to support multiple topics from bag file
            raise RuntimeError("Multiple matching topics in bag file")

        topic_name, total_msgs = matching_topics[0]

        diag_processor = EtherCATDiagnosticProcessor()
        last_topic = None
        msg_count = 0
        data_queue = []
        for topic, msg, unused in bag.read_messages(topics=[topic_name]):
            if (last_topic is not None) and (topic != last_topic):
                raise RuntimeError("Error multiple topics with diagnostics %s and %s" % (topic, last_topic))
            data = diag_processor.processDiagnosticsMsg(msg)
            if data is not None:
                data_queue.append(data)
            msg_count += 1
            if len(data_queue) >= 10:  # put messages in queue 10 at a time
                status_msg = "Processed %d of %d messages" % (msg_count, total_msgs)
                connection.send( (status_msg, data_queue) ) 
                data_queue = []

        connection.send( ("Loading complete", data_queue) )

    except Exception, e:
        status_msg = 'ERROR : ' + str(e)
        print "subproc : ", status_msg
        connection.send( (status_msg, []) )

    finally:
        connection.send( None )
