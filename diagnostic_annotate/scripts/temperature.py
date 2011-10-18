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
Usage: %(progname)s [-h] <input bagfile> <output csv file>
  Goes through diagnostics in <input bagfile> and grab temperature data for 
  MCBs, CPUs, and motors.  It save data to output file in CSV format.  

Options:
  -h : show this help
  -p : sample period (in seconds).  Defaults to every 300 seconds (5 minutes)"
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
import os.path
import traceback
import re
import csv

from diagnostic_annotate.kv_convert import ConvertVar, ConvertList, KeyValueConvertList, VarStorage

def usage(progname):
    print __doc__ % vars()



def _average(L):
    """ Returns average of list of values"""
    if len(L) == 0:
        raise RuntimeError("No average of empty list")
    return float(sum(L)) / float(len(L))
    

class EtherCATDeviceDiag:
    """ Parses out temperature data from EtherCAT Device diagnostics """
    def __init__(self, name, hardware_id):
        self.name = name
        kvl = KeyValueConvertList()
        kvl.add('Motor winding temp (C)', ConvertVar('winding_temp', float, None))
        kvl.add('Motor housing temp (C)', ConvertVar('housing_temp', float, None))
        kvl.add('Heating power (Watts)', ConvertVar('heating_power', float, None))
        kvl.add('Board temperature', ConvertVar('bridge_temp', float, None))
        kvl.add('Bridge temperature', ConvertVar('board_temp', float, None))
        self.kvl = kvl

    def process(self, msg): 
        vals = VarStorage()
        self.kvl.set_defaults(vals)
        self.kvl.convert(msg, vals)
            
        result = []
        if vals.winding_temp is not None:
            # only certian motors will have heating model enaled.
            # only those models will have winding and housing temperatures
            result.append( (self.name+" winding temp (max)", vals.winding_temp,  max ) )
            result.append( (self.name+" housing temp (max)", vals.housing_temp,  max ) )
            result.append( (self.name+" heating power (avg)", vals.heating_power, _average ) )
        result.append( (self.name+" bridge temp (max)", vals.bridge_temp, max) )
        result.append( (self.name+" board temp (max)",  vals.board_temp,  max) )

        return result



class EtherCATDeviceAddDiag:
    """ Looks for EtherCAT devices that are not already present and adds a new EtherCAT Device Diag for them """

    def __init__(self, diag_map):
        self.diag_map = diag_map
        self.ecat_re = re.compile("EtherCAT Device (#\d\d)?\((\w+)\)")

    def is_match(self, msg):
        # Only WG005, WG006, and WG021 have temperature sensors
        is_ecat = self.ecat_re.match(msg.name)
        is_wg05 = bool(re.match("68-05005-[0-9]{5}$", msg.hardware_id))
        is_wg06 = bool(re.match("68-05006-[0-9]{5}$", msg.hardware_id))
        is_wg21 = bool(re.match("68-05021-[0-9]{5}$", msg.hardware_id))
        return is_ecat and (is_wg05 or is_wg06 or is_wg21)
            
    def process(self, msg):
        ecat_name = self.ecat_re.match(msg.name).group(2)
        dev = EtherCATDeviceDiag(ecat_name, msg.hardware_id)
        self.diag_map[msg.name] = dev
        return dev.process(msg)



def process_bag(inbag_filename, output_filename, sample_period):
    diag_list = []
    diag_map = {}
    ignore_set = set()

    diag_list.append(EtherCATDeviceAddDiag(diag_map))
    #PowerBoardAddDiag(diag_list, diag_map)
    #CPU

    if not os.path.isfile(inbag_filename): 
        raise RuntimeError("Cannot locate input bag file %s" % inbag_filename)

    bag = rosbag.Bag(inbag_filename)
    print bag

    # Create a mapping between device names and arrays of temperature value
    times = []
    current_data = {}
    data = {}

    empty_value = None
    start_time = None
    prev_sample_time = None
    t = None
    for topic, msg, t in bag.read_messages(topics=['/diagnostics', 'diagnostics']):
        new_data = []
        t = msg.header.stamp # use timestamp from diagnostic msg head instead of bag timestamp
        if start_time is None: start_time = t
        if prev_sample_time is None: prev_sample_time = t        
        for status in msg.status:
            if status.name in diag_map:
                new_data += diag_map[status.name].process(status)
            elif status.name in ignore_set:
                pass
            else:
                matches = 0
                for diag in diag_list:
                    if diag.is_match(status):
                        new_data += diag.process(status)
                        matches+=1
                if matches == 0:
                    # no matches for this name, for efficiency reason add name to ignore set
                    ignore_set.add(status.name)
                    print "Ignore diagnostics from device", status.name
                elif matches > 1:
                    print "There are multitple (%d) matches form device %s" % (matches,status.name)


        # append new values to list of current values, also keep track of specified function for later
        for k,v,func in new_data: 
            if k not in current_data:
                current_data[k] = ([v],func)
            else:
                current_data[k][0].append(v)

        # if enough time has passed, sample most recent temperature value to list
        if (t-prev_sample_time) > sample_period:
            prev_sample_time = t
            for k,(v,func) in current_data.iteritems():
                if k not in data:
                    print "Adding column for", k
                    # Fill data array with empty values
                    data[k] = [empty_value for i in xrange(len(times))]
                # use func() to summarize data for results                    
                data[k].append(func(v))
            times.append(t)
            for k,v in data.iteritems():
                if len(v) < len(times):
                    print "Missing new data for", k
                    v += [empty_value for i in xrange(len(times) - len(v))]
            current_data = {}  # clear out current data after taking a sample

    # output collected data to file
    output_file = open(output_filename, 'w')
    csv_out = csv.writer(output_file, delimiter=',', quotechar='"')
    col_names = [k for k in data.iterkeys()]
    col_names.sort()
    csv_out.writerow(["time","reltime"] + col_names)
    for index,t in enumerate(times):
        row = [t.secs, (t-start_time).secs]
        row += [data[col_name][index] for col_name in col_names]
        csv_out.writerow(row)
                    
    output_file.close()

    if t is not None:
        print "Log ends %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec()))
    else:
        print "Log contains no diagnostic messages"


def main(argv):
    sample_period = rospy.Duration(5*60)  #sample every 5 minutes

    progname = argv[0]
    optlist, argv = getopt.gnu_getopt(argv[1:], "htp:", ["help", "test", "period"])
    for (opt, val) in optlist:
        if opt == "--help" or opt == '-h':
            usage(progname)
            return 0
        elif opt == "--test" or opt == '-t':
            return 0
        elif opt == '--period' or opt == '-p':
            sample_period = rospy.Duration(float(val))
        else:
            print "Internal error : unhandled option '%s'"%opt
            return 1

    if len(argv) != 2:
      usage(progname)
      return 1

    inbag_filename = argv[0]
    output_filename = argv[1]    
    process_bag(inbag_filename, output_filename, sample_period)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
