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
Usage: %(progname)s [-h] <bagfile1> [<bagfile2> ...]
  Goes through diagnostics in <bagfile>s and annotates error events.
  Save events in yaml file in current diretory.
  If the bagfile is named 'name.bag' the yaml output will be named 'name.bag.yaml'

Options:
  -h : show this help
"""

PKG = 'diagnostic_annotate'
REV = 1.0
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

class RunStopEventMerge:
    """ Merges Undervoltage errors and MotorsHalted events into Runstop event """
    def __init__(self):
        def get_t(obj):
            return obj.t
        past = rospy.Duration.from_sec(2)
        future = rospy.Duration.from_sec(5)
        self.dq = DelayQueue(self.merge, get_t, future, past)

    def process(self,event_list):
        return self.dq.process(event_list)

    def merge(self, event, future, past):
        if event.type != 'RunStopEvent':
            return
        for event2 in itertools.chain(future,past):
            if event.type == 'UndervoltageLockoutEvent':
                event.children.append(event2)
                event2.hide = True
            elif event.type == 'OnlyMotorHaltedEvent' and event.motors_halted == None:
                event.children.append(event2)
                event2.hide = True


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


def process_bag(inbag_filename, diag_list, diag_map):
    print
    if not os.path.isfile(inbag_filename): 
        raise RuntimeError("Cannot locate input bag file %s" % inbag_filename)

    output_filename = os.path.basename(inbag_filename) + '.yaml'
    if os.path.isfile(output_filename):
        print >>sys.stderr, "Skipping", inbag_filename
        return

    bag = rosbag.Bag(inbag_filename)
    print bag

    #for topic, msg, tbag in rosrecord.logplayer(inbag_filename):
    all_event_list = []    
    for topic, msg, t in bag.read_messages(topics=['/diagnostics']):
        t = msg.header.stamp # use timestamp from diagnostic msg head instead of bag timestamp
        header = False
        event_list = []
        for status in msg.status:
            if status.name in diag_map:
                event_list += diag_map[status.name].process(status, t)
            else:
                for diag in diag_list:
                    if diag.is_match(status):
                        event_list += diag.process(status, t)                    
                        break  # stop searching when first match is found

        all_event_list += event_list
        
        if len(event_list) > 0:
            last_name = event_list[0].name
            print "On %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec())) 
            for error in event_list:
                if error.name != last_name:
                    print ' ', error.name
                    last_name = error.name
                print '  ', error.desc

    #print >>sys.stderr 'Will save yaml output to :', output_filename
    yaml_bag_info = get_yaml_bag_info(bag)
    yaml_events = []
    yaml_events = [event.to_yaml() for event in all_event_list]
    yaml_output = {'rev':REV, 'bag':yaml_bag_info, 'events':yaml_events}
    output_file = open(output_filename, 'w')
    yaml.dump(yaml_output,stream=output_file)
    output_file.close()

    print "Log ends %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec()))


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

    if len(argv) == 0:
      usage(progname)
      return 1

    diag_list = []
    diag_map = {}

    EtherCATMasterDiag(diag_map)
    RealtimeControlLoopDiag(diag_map)
    EtherCATDeviceAddDiag(diag_list, diag_map)
    PowerBoardAddDiag(diag_list, diag_map)

    for inbag_filename in argv:
        try:
            process_bag(inbag_filename, diag_list, diag_map)
        except KeyboardInterrupt:
            print "Keyboard Interrupt, quiting"
            break
        except Exception,e:
            #print traceback.print_exc()
            print "Error annotating bag %s : %s" % (inbag_filename, str(e))

    # Use 30 second reordering buffer.
    #reorder = EventDelay(30.0, True)    
    #merge_list = []
    #merge_list.append(RunStopEventMerge())
    #print_errors = PrintEvents(1.0)
    #last_error_time = rospy.Time(0)
        
        # Generate no-errors every 10seconds to keep merge queues moving
        #if len(event_list) > 0:
        #    last_error_time = t
        #elif (t-last_error_time).to_sec() > 10.0:
        #    #print "FLUSH ------------------------"
        #    event_list.append(no_event("flush", t, "10second flush"))
        #    last_error_time = t

        # Put messages into reordering queue
        #event_list = reorder.process(event_list)

        #len(event_list)

        # Process error list
        #for merge in merge_list:
        #    event_list = merge.process(event_list)            

        # Process list
        #print_errors.process(event_list)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
