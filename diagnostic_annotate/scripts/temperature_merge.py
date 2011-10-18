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
import csv

from diagnostic_annotate.kv_convert import ConvertVar, ConvertList, KeyValueConvertList, VarStorage


def usage(progname):
    print __doc__ % vars()



class TemperatureMerge:
    def __init__(self):
        # data is a dictionary that maps names to array of temperature values
        # each array of temperature value should be exactly same length as teimes
        self.times = []
        self.data = {}

    def verify(self):
        for name,values in self.data.iteritems():
            if len(values) != len(self.times):
                raise RuntimeError("Length of data for %s (%d) doesn't match length of times (%d)" % (name, len(values), len(self.times)))
        

    def loadInputCSV(self, incsv_filename):
        print
        print "Input %s" % incsv_filename

        if not os.path.isfile(incsv_filename): 
            raise RuntimeError("Cannot locate input csv file %s" % incsv_filename)
        
        fd = open(incsv_filename, 'r')
        reader = csv.reader(fd)
        
        # First two columns of *csv should be time, and reltime, the rest is temperature data
        hdr = reader.next()
        col_names = hdr[2:]

        # Put in empty values for all columns that do not exists yet
        for name in col_names:
            if name not in self.data:
                print " creating column for", name
                self.data[name] = [None for i in xrange(len(self.times))]
                
        # Now read data row by row and append it to data structures
        row_count = 0
        for row in reader:
            row_count+=1
            for name, val in zip(col_names, row[2:]):
                self.data[name].append(val)
            self.times.append(row[0])

        # Input file was missing any sort of data, then fill it in
        for name,values in self.data.iteritems():
            if name not in col_names:     
                print " missing column for %s" % (name)            
                self.data[name] += [None for i in xrange(row_count)]


        self.verify()

    def saveOutputCSV(self, outcsv_filename):
        col_names = self.data.keys()
        col_names.sort()    
        sorted_data = []
        for index,t in enumerate(self.times):
            row = [t] + [self.data[name][index] for name in col_names]
            sorted_data.append(row)

        sorted_data.sort() # sorts data by time (first)

        start_time = sorted_data[0][0]

        # output collected data to file
        output_file = open(outcsv_filename, 'w')
        csv_out = csv.writer(output_file, delimiter=',', quotechar='"')
        col_names = self.data.keys()
        col_names.sort()
        csv_out.writerow(["time","reltime(mins)"] + col_names)
        for row in sorted_data:
            csv_out.writerow( [ row[0], (float(row[0])-float(start_time))/60. ] + row[1:] )
        output_file.close()



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

    tm = TemperatureMerge()
    for input_filename in input_filenames:
        tm.loadInputCSV(input_filename)
    tm.saveOutputCSV(output_filename)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
