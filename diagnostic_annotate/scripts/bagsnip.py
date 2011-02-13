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
Usage: %(progname)s [-h] <bagfile> <centertime> <+/-duration>
 
   Grabs snippet of bagfile starting at centertime-duration and ending at centertime+duration
   If input file is named 'file.bag' output file will be named 'file.snip.bag'
   If duration is not given, defaults to 1 second.

  Options:
  -h : show this help
"""

PKG = 'diagnostic_annotate'
REV = 1.0
import roslib
roslib.load_manifest(PKG)

import rospy
import rosbag
import time
import sys 
import os
import unittest
import getopt


def usage(progname):
    print __doc__ % vars()


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

    if len(argv) < 2  or len(argv) > 3:
      usage(progname)
      return 1

    inbag_filename = argv[0] 
    if not os.path.isfile(inbag_filename): 
        print >> sys.stderr, "Cannot locate input bag file [%s]" % inbag_filename 
        return 2
    
    centertime = rospy.Time.from_sec(float(argv[1]))
    if len(argv) > 2:
        duration = rospy.Duration.from_sec(float(argv[2])) 
    else:
        duration = rospy.Duration.from_sec(1.0)

    name,ext = os.path.splitext(os.path.basename(inbag_filename))
    outbag_filename = name+'.snip'+ext
    print 'will save output to :', outbag_filename

    output_file = open(outbag_filename, 'w')

    inbag  = rosbag.Bag(inbag_filename)
    outbag = rosbag.Bag(outbag_filename, mode='w')

    starttime = centertime - duration
    endtime   = centertime + duration

    for topic, msg, t in inbag.read_messages(start_time=starttime, end_time=endtime, raw=True):
        outbag.write(topic,msg,t=t,raw=True)

    outbag.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
