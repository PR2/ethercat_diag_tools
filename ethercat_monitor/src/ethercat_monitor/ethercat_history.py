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
import socket
import traceback
#import roslib.msg

from ethercat_monitor.ethercat_device_diag import EtherCATDeviceDiag, EtherCATDeviceAddDiag
from ethercat_monitor.ethercat_master_diag import EtherCATMasterDiag

from ethercat_monitor.util import prettyTimestamp, prettyDuration
from ethercat_monitor.timestep_data import EtherCATHistoryTimestepData, EtherCATHistoryTimestepDataNote
from drop_estimator import DropEstimator

import ethercat_monitor.msg
import ethercat_monitor.timestep_data
#import std_msgs.msg




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
            #hdr = std_msgs.msg.Header(msg.header.seq, msg.header.stamp, msg.header.frame_id)
            #hdr = roslib.msg.Header(msg.header.seq, msg.header.stamp, msg.header.frame_id)
            sys_status = ethercat_monitor.msg.EtherCATSystemStatus(msg.header.stamp, master, devices)
            #sys_status.master = master
            #sys_status.devices = devices
            #hdr = sys_status.hdr
            #hdr.stamp = msg.header.stamp

            #sys_status.master = master
            #sys_status.devices = devices
            #hdr = sys_status.header
            
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
        self.error_msg = None
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
            new_history = EtherCATHistory(title, self.topic_name)
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
            history.processAndAddEtherCATSystemMsg(system_msg)
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

    def getAndClearErrorMsg(self):
        with self.lock:
            msg = self.error_msg    
            self.error_msg = None
        return msg


class EtherCATBagReader:
    """ Class contain reader for diagnostic bag files """
    def __init__(self, bag_filename):
        self.bag_filename = bag_filename
        if not os.path.isfile(bag_filename): 
            raise RuntimeError("Cannot locate bag file %s" % bag_filename)
        self.lock = threading.Lock()        
        self.history_list = []
        self.last_system_msg = None
        self.error_msg = None

        # Parseing bag file is more CPU intesive than I/O intesive.
        # Because of the global interpreter lock, using a thread to perform 
        # processing would bog down application.  Instead, use a separate 
        # process to do most of work parsing bag file

        #self.connection, subproc_connection = multiprocessing.Pipe()

        # use thread to get data from bag file and put it in history structure
        self.thread = threading.Thread(target=self.processorMonitorThread, name='process_thread')
        self.thread.start()

    def spawnNewHistory(self, description, topic_name):
        with self.lock:
            if len(self.history_list) > 0:
                old_history = self.history_list[-1]
                old_history.updateStatus("Complete")
            print "Spawning a new bag reader EtherCAT History"
            title = "%s - %d" % (description, len(self.history_list))
            new_history = EtherCATHistory(title, topic_name)
            self.history_list.append(new_history)
        return new_history

    def processorMonitorThread(self):
        """Looks at bag summary to determine type of information saved in bag.
        Possible types: 
          * Diagnostics data. Message types:  
             * diagnostics_msgs/DiagnosticsArray
          * ethercat_monitor saved data. Message types:
             * ethercat_monitor/SavedBagHeader
             * ethercat_monitor/EtherCATSystemStatus
             * ethercat_monitor/StatusNote
          * Converted diagnostics data, produced by running convert.py  :
             * ethercat_monitor/EtherCATSystemStatus
        For each source of data, the reader creates history class to store data.
        """
        try:
            bag = rosbag.Bag(self.bag_filename)

            # first look at bag file to determine what topic has diagnostic_msgs/DiagnosticArray type
            y = yaml.load(bag._get_yaml_info())
            diagnostic_topics = [ ]
            header_topics = [ ]
            note_topics = [ ]
            system_topics = [ ]
            for topic_info in y['topics']:
                topic_name = topic_info['topic']
                num_msgs = int(topic_info['messages'])
                topic_type = topic_info['type']
                if topic_type == 'diagnostic_msgs/DiagnosticArray':
                    diagnostic_topics.append( (topic_name, num_msgs) )
                elif topic_type == 'ethercat_monitor/SavedBagHeader':
                    header_topics.append( (topic_name, num_msgs) )
                elif topic_type == 'ethercat_monitor/StatusNote':
                    note_topics.append( (topic_name, num_msgs) )
                elif topic_type == 'ethercat_monitor/EtherCATSystemStatus':
                    system_topics.append( (topic_name, num_msgs) )
                else:
                    print "Bag contains topic '%s' with unknown message type : " % (topic_name, topic_type)

            bag.close()

            # if there is a note topic, then there should be just one threading topic
            if (len(diagnostic_topics) == 0) and (len(system_topics) == 0) and (len(header_topics) == 0):
                raise RuntimeError("No diagnostic data in bag file")

            if len(note_topics) > 1:
                raise RuntimeError("Bag contains multiple note topics")

            if len(header_topics) > 1:
                raise RuntimeError("Bag contains multiple header topics")

            # start a thread for each diagnostics topic
            thread_list = []
            for topic_name,num_msgs in diagnostic_topics:
                args = (self.bag_filename, topic_name, num_msgs)
                thread = threading.Thread(target=self.processDiagnosticsBagThread, args=args)
                thread_list.append(thread)

            if len(header_topics) == 1: 
                if len(note_topics) > 1:
                    raise RuntimeError("Bag contains header and multiple note topics")
                if len(system_topics) > 1:
                    raise RuntimeError("Bag contains header and multiple system topics")
                if len(system_topics) > 0:
                    (system_topic_name, num_system_msgs) = system_topics[0]
                else:
                    (system_topic_name, num_system_msgs) = (None, 0)
                if len(note_topics) > 0:
                    (note_topic_name, num_note_msgs  ) = note_topics[0]
                else:
                    (note_topic_name, num_note_msgs)   = (None, 0)
                (header_topic_name, num_header_msgs ) = header_topics[0]
                if num_header_msgs != 1:
                    raise RuntimeError("Bag contains multiple (%d) header messages" % num_header_msgs)
                num_msgs = num_note_msgs + num_system_msgs
                args = (self.bag_filename, system_topic_name, header_topic_name, note_topic_name, num_msgs)
                thread = threading.Thread(target=self.processSavedBagThread, args=args)
                thread_list.append(thread)
            else:
                if len(note_topics) > 0:
                    raise RuntimeError("Bag file without header contains note topics")
                # bag file produced by convert (just has system messages but no notes and no header
                for topic_name,num_msgs in system_topics:
                    args = (self.bag_filename, topic_name, num_msgs)
                    thread = threading.Thread(target=self.processConvertedBagThread, args=args)                
                    thread_list.append(thread)

            for thread in thread_list:
                thread.start()

            # collect all running threads
            for thread in thread_list:            
                thread.join()

        except Exception, e:
            trace = traceback.format_exc()            
            with self.lock:
                self.error_msg = "Error : " + str(e) + '\n' + trace
            print trace


    def processConvertedBagThread(self, bag_filename, topic_name, num_msgs):
        bag = rosbag.Bag(self.bag_filename)
        history = self.spawnNewHistory("converted bagfile", '<unknown-topic>')
        # pull systems messages (history) from bag file
        msg_count = 0
        for topic, msg, unused in bag.read_messages(topics=[topic_name]):
            history.processAndAddEtherCATSystemMsg(msg)
            msg_count += 1
            history.updateStatus("Processed %d of %d messages" % (msg_count, num_msgs))

        history.updateStatus("Processing complete")            
        bag.close()
        

    def processSavedBagThread(self, bag_filename, system_topic_name, header_topic_name, note_topic_name, num_msgs):
        # if available pull header from bag file
        bag = rosbag.Bag(self.bag_filename)
        for topic, msg, unused in bag.read_messages(topics=[header_topic_name]):
            print msg
            history = self.spawnNewHistory("saved bag : " + msg.topic_name, msg.topic_name)

        msg_count = 0
        # if available, pull notes from bag file
        if note_topic_name is not None:
            for topic, msg, unused in bag.read_messages(topics=[note_topic_name]):
                tsd = EtherCATHistoryTimestepData(msg.system_status)
                note = EtherCATHistoryTimestepDataNote(tsd, msg.note)
                history.addNote(note) 
                msg_count += 1
                history.updateStatus("Processed %d of %d messages" % (msg_count, num_msgs))
                
        # pull systems messages (history) from bag file
        if system_topic_name is not None:
            for topic, msg, unused in bag.read_messages(topics=[system_topic_name]):
                history.addEtherCATSystemMsg(msg)
                msg_count += 1
                history.updateStatus("Processed %d of %d messages" % (msg_count, num_msgs))
            
        history.updateStatus("Processing complete")            
        bag.close()

            
    def processDiagnosticsBagThread(self, bag_filename, topic_name, num_msgs):
        history = self.spawnNewHistory('diagnostics bag : ' + topic_name, topic_name)
        history.updateStatus("Starting bag parser...")

        self.queue = multiprocessing.Queue(100)
        args=(self.bag_filename, topic_name, num_msgs, self.queue)
        self.subproc = multiprocessing.Process(target=processBagSubProc, name='process_bag', args=args)
        self.subproc.start()

        result = self.queue.get()
        while result is not None:
            (status_msg, system_msg_list) = result
            history.updateStatus(status_msg)
            for system_msg in system_msg_list:
                if isNewEtherCATRun(system_msg, self.last_system_msg):
                    history = self.spawnNewHistory(topic_name)                
                history.processAndAddEtherCATSystemMsg(system_msg)
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

    def getAndClearErrorMsg(self):
        with self.lock:
            msg = self.error_msg    
            self.error_msg = None
        return msg



class EtherCATHistory:
    def __init__(self, title, topic_name):
        self.title = title
        self.topic_name = topic_name
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

    def getTopicName(self):
        return self.topic_name

    def getTitle(self):
        return self.title

    def updateStatus(self, msg):
        with self.lock:
            self.status_msg = msg

    def getStatus(self):
        with self.lock:
            return self.status_msg

    def processAndAddEtherCATSystemMsg(self, system_msg):
        """ Adds system message to history while doing some processing such as drop estimation""" 
        timestep_data = EtherCATHistoryTimestepData(system_msg)
        with self.lock:
            if self.newest_tsd is not None:
                if (timestep_data.getTimestamp() < self.newest_tsd.getTimestamp()):
                    raise RuntimeError("New data is older than previous data in history")
            self.drop_estimator.process(timestep_data)
            if self.last_sample_time is None:
                self.notes.append(EtherCATHistoryTimestepDataNote(timestep_data, "First message"))
            self._addTimestepData(timestep_data)

    def addEtherCATSystemMsg(self, system_msg):
        """ Adds system message to history without doing any extra processing 
        (such as running drop estimator).  This is useful when loading data from 
        saved file where notes are present and drop estimator has already been run. """ 
        timestep_data = EtherCATHistoryTimestepData(system_msg)
        with self.lock:
            if self.newest_tsd is not None:
                if (timestep_data.getTimestamp() < self.newest_tsd.getTimestamp()):
                    raise RuntimeError("New data is older than previous data in history")
            self._addTimestepData(timestep_data)

    def _addTimestepData(self, timestep_data):
        """ Adds Timestamp data to history Assumes lock is held. """
        history = self.history
        self.newest_tsd = timestep_data
        # save sample timestep data every minute to history            
        if self.last_sample_time is None:
            # first sample
            history.append(timestep_data)
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
        """ Saves history data and notes to bagfile.  The saved bag file should have 3-topics. 
        1. bag_header
        2. notes
        3. system_status
        """
        # make shallow copy of history, so it can continue to get new data while we saving current data
        with self.lock:
            history = copy.copy(self.history)
            notes = copy.copy(self.notes)
            
        outbag = rosbag.Bag(outbag_filename, 'w', compression=rosbag.Compression.BZ2)
        hdr_msg = ethercat_monitor.msg.SavedBagHeader()
        hdr_msg.save_time = rospy.Time.from_sec(time.time())
        hdr_msg.topic_name = self.getTopicName()
        hdr_msg.machine_name = socket.gethostname()
        hdr_msg.major_version = 1
        hdr_msg.minor_version = 0
        if 'USERNAME' in os.environ:
            hdr_msg.user_name = os.environ['USERNAME']
        outbag.write('bag_header', hdr_msg, t=hdr_msg.save_time)
        for note in notes:
            msg = ethercat_monitor.msg.StatusNote() 
            msg.note = note.note_msg
            msg.system_status = note.timestep_data.system
            outbag.write('notes', msg, t=note.timestep_data.getTimestamp() )
        for tsd in history:
            outbag.write('system_status', tsd.system, t=tsd.getTimestamp())

        print "Saved %d notes and %d messages to history" % (len(notes), len(history))
        outbag.close()
        



def processBagSubProc(bag_filename, topic_name, num_msgs, queue):
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
                status_msg = "Processed %d of %d messages" % (msg_count, num_msgs)
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
