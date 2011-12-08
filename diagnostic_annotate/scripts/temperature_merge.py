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
Usage: %(progname)s [-h] <output csv file> <input csv files  ...>
  Merges temperature data in put input CSV files into output file  

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


def usage(progname):
    print __doc__ % vars()


from diagnostic_annotate.temperature_data import TemperatureData


def main(argv):

    progname = argv[0]
    optlist, argv = getopt.gnu_getopt(argv[1:], "ht", ["help", "test"])
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

    if len(argv) < 2:
      usage(progname)
      return 1

    output_filename = argv[0] 
    input_filenames = argv[1:]

    td = TemperatureData()
    for input_filename in input_filenames:
        td.loadInputCSV(input_filename)
    td.sortData()  
    td.saveOutputCSV(output_filename)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
