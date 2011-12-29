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
##\brief Annotates errors in diagnostic bag files.

"""
Usage: %(progname)s [-h] <input bagfile> <output bagfile>
  Goes through diagnostics in <input bagfile> and annotates error events 
  and outputs results to <output_bagfile>

Options:
  -h : show this help
"""

PKG = 'diagnostic_annotate'
REV = 1.01
# rev 1.00 : start
# rev 1.01 : 
#   Added processing support for MotorTraces and Mtrace event types
#   Added event types for ingore and mutiple match errors 
import roslib
roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

import rospy
import rosbag
import time
import sys 
import os
import unittest
import getopt
import itertools
import yaml
import os.path
import traceback
import re

from diagnostic_annotate.diag_event import DiagEvent, no_event
from diagnostic_annotate.event_delay import EventDelay
from diagnostic_annotate.delay_queue import DelayQueue
from diagnostic_annotate.bagutil import get_yaml_bag_info

from diagnostic_annotate.ethercat_device_diag import EtherCATDeviceDiag, EtherCATDeviceAddDiag
from diagnostic_annotate.power_board_diag import PowerBoardDiag, PowerBoardAddDiag
from diagnostic_annotate.realtime_loop_diag import RealtimeControlLoopDiag
from diagnostic_annotate.ethercat_master_diag import EtherCATMasterDiag


def usage(progname):
    print __doc__ % vars()


def ignoreDevice(name, t):
    """ Represents an ignored device diagnostic type"""
    return DiagEvent('Ignored', name, t, "Ignored device diagnostics")

def multipleMatchError(name, t, matches):
    """ Represents error where multiple things match a single device """
    return DiagEvent('MultipleMatches', name, t, "Multiple (%d) matches for %s" % (matches, name) )


class PrintEvents:
    def __init__(self, duration):
        self.duration = duration
        self.last_time = rospy.Time(0)
        self.last_name = "NO_ERROR"

    def process(self, event_list):
        if len(event_list) == 0:
            return

        for event in event_list:
            if event.type == 'NoEvent':
                continue
            if event.hide:
                continue
            if (event.t - self.last_time).to_sec() > self.duration:
                print "On %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(event.t.to_sec())) 
                #t1 = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.last_time.to_sec()))
                #t2 = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(event.t.to_sec())) 
                #print "Between %s and %s" % (t1,t2)
                self.last_time = event.t
                print ' ', event.name
                self.last_name = event.name

            if event.name != self.last_name:
                print ' ', event.name
                self.last_name = event.name
            print '   ', event.short_desc()


class DiagBagProcessor(object):
    def __init__(self, inbag_filename):
        self.diag_list = []
        self.diag_map = {}
        self.ignore_set = set()

        EtherCATMasterDiag(self.diag_map)
        RealtimeControlLoopDiag(self.diag_map)
        EtherCATDeviceAddDiag(self.diag_list, self.diag_map)
        PowerBoardAddDiag(self.diag_list, self.diag_map)

        if not os.path.isfile(inbag_filename): 
            raise RuntimeError("Cannot locate input bag file %s" % inbag_filename)
        self.bag = rosbag.Bag(inbag_filename)
        print self.bag

        # get all diagnostics or MotorTrace messsages from bag file
        self.yaml_bag_info = get_yaml_bag_info(self.bag)

        # make a mapping from topic name to type, 
        # also make a list of topics that have specific types
        topics = []
        topic_types = {}
        if 'topics' in self.yaml_bag_info:
            for topic_yaml in self.yaml_bag_info['topics']:
                print topic_yaml
                typ  = topic_yaml['type']
                topic = topic_yaml['topic']
                topic_types[topic] = typ
                if typ in ('ethercat_hardware/MotorTrace', 'diagnostic_msgs/DiagnosticArray'):
                    topics.append(topic)
        self.topics = topics
        self.topic_types = topic_types


    def _processMotorTrace(self, msg, t):
        """ Processes a MotorTrace message and returns a list of DiagEvents """
        if msg.reason == "Safety Lockout":
            typ = "MtraceSafetyLockout"
        elif msg.reason in ("New max voltage error", "New max current error"):
            typ = "MtraceMotorModelWarning"
        elif re.match("Problem with the MCB, motor, encoder, or actuator model", msg.reason):
            typ = "MtraceMotorModelError"
        else:
            typ = "Mtrace"
        d = DiagEvent(typ, msg.actuator_info.name, t, msg.reason)
        d.data = {'reason':msg.reason}
        return [d]

    def _processDiagnosticStatus(self, msg, t):
        """ Internal function that finds processor for DiagnosticStatus message """
        name = msg.name
        event_list = []
        if name in self.diag_map:
            event_list += self.diag_map[name].process(msg, t)
        elif name in self.ignore_set:
            pass
        else:
            event_list = []
            matches = 0
            for diag in self.diag_list:
                if diag.is_match(msg):
                    if matches == 0:
                        event_list += diag.process(msg, t)
                    matches+=1
            if matches == 0:
                # no matches for this name, for efficiency reason add name to ignore set
                self.ignore_set.add(name)
                event_list.append(ignoreDevice(name, t))
                #print "Ignore diagnostics from device", name
            elif matches > 1:
                print "There are multitple (%d) matches for %s" % (matches,str(name))
                event_list.append(multipleMatchError(name, t, matches))
        return event_list
    

    def process(self, output_filename):
        #for topic, msg, tbag in rosrecord.logplayer(inbag_filename):
        all_event_list = []
        t = None
        for topic, msg, t in self.bag.read_messages(topics=self.topics):
            t = msg.header.stamp # use timestamp from diagnostic msg head instead of bag timestamp
            typ = self.topic_types[topic]
            if typ == 'diagnostic_msgs/DiagnosticArray':
                # DiagnosticArray contains multiple submessages with DiagnosticStatus type.
                # for purposes of this processor, treat these messages separately
                event_list = []
                for status in msg.status:
                    event_list += self._processDiagnosticStatus(status, t)
            elif typ == 'ethercat_hardware/MotorTrace':
                # there is only one processor for all MotorTrace messages
                event_list = self._processMotorTrace(msg, t)
            else:
                raise RuntimeError("Invalid message type %s" % typ)

            all_event_list += event_list

            if len(event_list) > 0:
                last_name = event_list[0].name
                print "On %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec())) 
                for error in event_list:
                    if error.name != last_name:
                        print ' ', error.name
                        last_name = error.name
                    print '  ', error.desc

        yaml_events = []
        yaml_events = [event.to_yaml() for event in all_event_list]
        yaml_output = {'rev':REV, 'bag':self.yaml_bag_info, 'events':yaml_events}
        output_file = open(output_filename, 'w')
        yaml.dump(yaml_output,stream=output_file)
        output_file.close()

        if t is not None:
            print "Log ends %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec()))
        else:
            print "Log contains no diagnostic messages"
            



def main(argv):
    progname = argv[0]
    optlist, argv = getopt.getopt(argv[1:], "ht", ["help", "test"])
    for (opt, val) in optlist:
        if opt == "--help" or opt == '-h':
            usage(progname)
            return 0
        elif opt == "--test" or opt == '-t':
            return 0
        else:
            print "Internal error : unhandled option '%s'"%opt
            return 1

    if len(argv) != 2:
      usage(progname)
      return 1

    inbag_filename = argv[0]
    output_filename = argv[1]  
    bp = DiagBagProcessor(inbag_filename)
    bp.process(output_filename)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
