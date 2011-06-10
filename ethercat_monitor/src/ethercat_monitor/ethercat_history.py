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



def isNewEtherCATRun(new_system_msg, old_system_msg):
    """ return True if new and old message seems to have come from restart of pr2_etherCAT"""
    # It is possible to restart pr2_etherCAT while ethercat_monitor continues running. 
    # When pr2_etherCAT is restarted, certain values (send_packets, dropped_packet, ...)
    # will once again start at zero.  
    if (old_system_msg is None) or (new_system_msg is None):
        return False
    return (new_system_msg.master.sent < old_system_msg.master.sent)



class EtherCATSubscriber:
    """ 
    Class contains subscriber and callback function for disagnostics.
    The callback will append new diagnostic pr2_etherCAT diagnostics data to a given EteherCATHistory.
   This allows subscriber data can be redirected to a new EtherCATHistory when needed.
    """    
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.lock = threading.Lock()
        self.history_list = []
        self.diag_processor = EtherCATDiagnosticProcessor()
        self.subscriber = rospy.Subscriber(str(topic_name), DiagnosticArray, self.diagnosticsCallback)
        self.last_system_msg = None
        self.dot_count = 0
        self.spawnNewHistory()

    def spawnNewHistory(self):
        # When pr2_etherCAT is restarted, we want to have the ethercat_monitor create a 
        # new tab for the received data, instead of appending values to current data.
        # 
        # To implement this feature, we start a new EtherCATHistory, when we detect the
        # pr2_etherCAT has been restarted.  The new EtherCATHistory is saved in a list.
        # Every so often the GUI will check for new histories objects and add a new 
        # panel for each of them.
        with self.lock:
            if len(self.history_list) > 0:
                old_history = self.history_list[-1]
                old_history.updateStatus("Connection closed")
            print "Spawning a new subscriber EtherCAT History"
            title = "%s - %d" % (self.topic_name, len(self.history_list))
            new_history = EtherCATHistory(title)
            new_history.updateStatus("Waiting for first messsage...")
            self.history_list.append(new_history)

    def diagnosticsCallback(self, msg):            
        system_msg = self.diag_processor.processDiagnosticsMsg(msg)
        if system_msg is not None:
            if isNewEtherCATRun(system_msg, self.last_system_msg):
                self.spawnNewHistory()                
            history = self.history_list[-1]  # new data goes to most recent item on list
            history.updateStatus("Recieving messages" + ('.'*self.dot_count))
            self.dot_count = 0 if (self.dot_count >= 5) else (self.dot_count+1)
            history.addEtherCATSystemMsg(system_msg)
            self.last_system_msg = system_msg

    def getHistoryList(self):
        with self.lock:
            history_list = copy.copy(self.history_list)
        return history_list

    def getSourceDesc(self):
        """ Returns string describing source of reader data"""
        return "ROS Topic '%s'" % self.topic_name

    def getTitle(self):
        """ Returns short string describing reader"""
        return self.topic_name

    def getTopic(self):
        return self.topic_name



class EtherCATBagReader:
    """ Class contain reader for diagnostic bag files """
    def __init__(self, bag_filename):
        self.bag_filename = bag_filename
        if not os.path.isfile(bag_filename): 
            raise RuntimeError("Cannot locate bag file %s" % bag_filename)
        self.lock = threading.Lock()        
        self.history_list = []
        self.last_system_msg = None

        # Parseing bag file is more CPU intesive than I/O intesive.
        # Because of the global interpreter lock, using a thread to perform 
        # processing would bog down application.  Instead, use a separate 
        # process to do most of work parsing bag file

        #self.connection, subproc_connection = multiprocessing.Pipe()

        # use thread to get data from bag file and put it in history structure
        self.thread = threading.Thread(target=self.processorMonitorThread, name='process_thread')
        self.thread.start()

    def spawnNewHistory(self, topic_name):
        with self.lock:
            if len(self.history_list) > 0:
                old_history = self.history_list[-1]
                old_history.updateStatus("Complete")
            print "Spawning a new bag reader EtherCAT History"
            title = "%s - %d" % (topic_name, len(self.history_list))
            new_history = EtherCATHistory(title)
            self.history_list.append(new_history)

    def processorMonitorThread(self):
        """Looks at bag summary and starts subprocess to parse bag data
        Then, pulls updates from subprocess and adds them to history 
        Updates are in form of EtherCATSystemStatus messages. 
        Wrap them in timestep data class before adding them to history
        """
        bag = rosbag.Bag(self.bag_filename)

        # first look at bag file to determine what topic has diagnostic_msgs/DiagnosticArray type
        y = yaml.load(bag._get_yaml_info())
        matching_topics = [ ]
        for topic_info in y['topics']:
            if topic_info['type'] == 'diagnostic_msgs/DiagnosticArray':
                topic_name = topic_info['topic']
                num_msgs = int(topic_info['messages'])
                matching_topics.append( (topic_name, num_msgs) )

        bag.close()

        if len(matching_topics) == 0:            
            raise RuntimeError("No diagnostic data in bag file")
        elif len(matching_topics) > 1:
            # todo, fix to support multiple topics from bag file
            raise RuntimeError("Multiple matching topics in bag file")

        topic_name, total_msgs = matching_topics[0]
        self.spawnNewHistory(topic_name)
        self.history_list[-1].updateStatus("Starting bag parser...")

        self.queue = multiprocessing.Queue(100)
        args=(self.bag_filename, topic_name, total_msgs, self.queue)
        self.subproc = multiprocessing.Process(target=processBagSubProc, name='process_bag', args=args)
        self.subproc.start()

        result = self.queue.get()
        while result is not None:
            (status_msg, system_msg_list) = result
            self.history_list[-1].updateStatus(status_msg)
            for system_msg in system_msg_list:
                if isNewEtherCATRun(system_msg, self.last_system_msg):
                    self.spawnNewHistory()                
                history = self.history_list[-1]  # new data goes to most recent item on list
                history.addEtherCATSystemMsg(system_msg)
                self.last_system_msg = system_msg
            result = self.queue.get()
        print "Subprocess is complete"
        self.subproc.join()
        self.subproc = None


    def getSourceDesc(self):
        """ Returns string describing source of reader data"""
        return "Bag file '%s'" % self.bag_filename

    def getTitle(self):
        """ Returns short string describing reader"""
        return os.path.basename(self.bag_filename)

    def getHistoryList(self):
        with self.lock:
            history_list = copy.copy(self.history_list)
        return history_list

    def getTopic(self):
        return None



class EtherCATHistory:
    def __init__(self, title):
        self.title = title
        self.status_msg = "Starting..."
        self.END = "END"
        self.BEGIN = "BEGIN"
        self.lock = threading.Lock()        
        self.drop_estimator = DropEstimator()
        self.sample_interval = rospy.Duration(60)  # default to sampling data every 60 seconds
        self.last_sample_time = None
        # list of automatically saved ethercat timestep data.  
        # New data is saved to history every minute.  
        # history allows user to go back in time to view events that might have been missed
        self.history = []
        # Notes are timestamp data that have been explicity saved by user
        # the note part is a message that the user provides for the timestamp data
        self.notes = []
        self.newest_tsd = None

    def getTitle(self):
        return self.title

    def updateStatus(self, msg):
        with self.lock:
            self.status_msg = msg

    def getStatus(self):
        with self.lock:
            return self.status_msg

    def addEtherCATSystemMsg(self, system_msg):
        with self.lock:
            timestep_data = EtherCATHistoryTimestepData(system_msg)
            history = self.history
            if self.newest_tsd is not None:
                if (timestep_data.getTimestamp() < self.newest_tsd.getTimestamp()):
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

    def getAllTimestepData(self):
        with self.lock:
            history = copy.copy(self.history)
            if self.newest_tsd is not None:
                history.append(self.newest_tsd)
            return history

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

    def saveBag(self, outbag_filename):
        # make shallow copy of history, so it can continue to get new data while we saving current data
        with self.lock:
            history = copy.copy(self.history)
            notes = copy.copy(self.notes)
            
        outbag = rosbag.Bag(outbag_filename, 'w', compression=rosbag.Compression.BZ2)
        for tsd in history:
            outbag.write('ethercat_system_status', tsd.system, t=tsd.system.header.stamp)
        print "Saved %d messages to history" % len(history)
        outbag.close()
        



def processBagSubProc(bag_filename, topic_name, total_msgs, queue):
    """ Processes diagnostic message in bag file, puts results in connection as tuple of (status_msg, result_list)"""
    timeout = 30.0  #
    try:
        if not os.path.isfile(bag_filename): 
            raise RuntimeError("Cannot locate input bag file %s" % bag_filename)

        bag = rosbag.Bag(bag_filename)

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
                queue.put( (status_msg, data_queue) , timeout=timeout) 
                data_queue = []

        queue.put( ("Loading complete", data_queue), timeout=timeout)    

    except multiprocessing.Queue.Full, e:
        print "Bag subprocessor : queue full"

    except Exception, e:
        status_msg = 'ERROR : ' + str(e)
        print "Bag subprocessor : ", status_msg
        queue.put( (status_msg, []), timeout=timeout)

    finally:
        bag.close()
        queue.put( None, timeout=timeout)
