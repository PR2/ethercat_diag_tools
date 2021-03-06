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
##\brief Provide GUI to view network related EtherCAT errors.

"""
Usage: %(progname)s [-m min] [-h] <in_bagfile> <out_bagfile>
  Converts diagnostics data in <in_bagfile> and produces 
  <output_bag> with EtherCATSystemStatus messages

Arguments:
  -m min  : save new system status sample every <min> minutes

Options:
  -h : show this help
"""

PKG = 'ethercat_monitor'
import roslib
roslib.load_manifest(PKG)

import rosbag
import rospy
import time
import sys 
import os
import getopt
import yaml

from ethercat_monitor.ethercat_device_diag import EtherCATDeviceDiag, EtherCATDeviceAddDiag
from ethercat_monitor.ethercat_master_diag import EtherCATMasterDiag
from ethercat_monitor.msg import EtherCATSystemStatus

def usage(progname):
    print __doc__ % vars()    


def convert(inbag_filename, outbag_filename, interval):        
    target_time = None

    inbag = rosbag.Bag(inbag_filename)


    y = yaml.load(inbag._get_yaml_info())
    if 'topics' not in y:
        print "Bag file is empty or not indexed"
        inbag.close()
        return

    # In some cases the 'diagnostics' data can be recorded as some topic other than '/diagnostics' 
    # One example would the the burn-in test stations where the diagnostics topic 
    # is prepended with the teststation name (for example /testc/diagnostics )
    # For these cases its better to look for the topic with the right message type (DiagnosticsArray)
    diagnostic_topics = [ ]
    for topic_info in y['topics']:
        topic_name = topic_info['topic']
        topic_type = topic_info['type']
        if topic_type == 'diagnostic_msgs/DiagnosticArray':
            diagnostic_topics.append( ( topic_name, topic_type ) )

    if len(diagnostic_topics) == 0:
        print "Bag file does not contain any diagnostics"
        inbag.close()
        return 

    if len(diagnostic_topics) > 1:
        print "Can not handle than 1 diagnostics topic is a single bag file."
        print "Bag file contains the following diagnostic topcis:"
        for topic_name,topic_type in diagnostic_topics:
            print "  ", topic_name
        inbag.close()
        return


    outbag = rosbag.Bag(outbag_filename, 'w', compression=rosbag.Compression.BZ2)

    diagnostic_topic_name = diagnostic_topics[0][0]

    device_diag_map = {}
    master_diag     = EtherCATMasterDiag()
    other_diag_map = {}
    device_add_diag = EtherCATDeviceAddDiag(device_diag_map)

    total = 0
    used = 0

    for topic, msg, t in inbag.read_messages(topics=[diagnostic_topic_name]):
        system = None
        for status in msg.status:
            if status.name in device_diag_map:
                dev_status = device_diag_map[status.name].process(status)
                if system is None:
                    system = EtherCATSystemStatus()
                system.devices.append(dev_status)
            elif status.name in other_diag_map:
                pass # Ingore anything filed in other
            elif master_diag.isMatch(status):
                master_status = master_diag.process(status)
                if system is None:
                    system = EtherCATSystemStatus()
                system.master = master_status
            elif device_add_diag.is_match(status):
                dev_status = device_add_diag.process(status)
                if system is None:
                    system = EtherCATSystemStatus()
                system.devices.append(dev_status)
            else:
                other_diag_map[status.name] = None

        if system is not None:
            total+=1
            if target_time is None:
                target_time = msg.header.stamp
            if msg.header.stamp >= target_time:
                system.stamp = msg.header.stamp
                outbag.write('ethercat_system_status', system, t=t)
                target_time += interval
                used+=1

        
    print "used %d of %d messages" % (used, total)
    inbag.close()
    outbag.close()


def main(argv):
    progname = argv[0]
    optlist, argv = getopt.gnu_getopt(argv[1:], "hm:", ["help"])
    interval = rospy.Duration(0)
    for (opt, val) in optlist:
        if opt == "--help" or opt == '-h':
            usage(progname)
            return 0
        if opt == '-m':
            interval = rospy.Duration(60.*float(val))
        else:
            print "Internal error : unhandled option '%s'"%opt
            return 1
    
    if len(argv) != 2:
        usage(progname)
        return 1

    inbag_filename = argv[0]
    outbag_filename = argv[1]
    if not os.path.isfile(inbag_filename):
        print "Cannot locate input bag file %s" % inbag_filename
        return 1

    convert(inbag_filename, outbag_filename, interval)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
