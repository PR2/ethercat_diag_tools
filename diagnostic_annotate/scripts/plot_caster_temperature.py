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
##\brief Plot caster temperature data from *.csv file 

"""
Usage: %(progname)s [-h] <input csv file>
  Creates temperature data plot for each caster in *.csv file

Options:
  -h : show this help
"""

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

import rospy
import sys 
import unittest
import getopt
import traceback
import re
import pylab
import time

from diagnostic_annotate.temperature_data import TemperatureData

def usage(progname):
    print __doc__ % vars()


def plotData(td, base_name):
    col_names = td.getColumnNames()
    col_names.sort()
    times = td.getTimes()

    start_time = float(times[0])
    rel_times = [ (float(t) - start_time)/60. for t in times ]
    

    temp_names = [ ]
    power_names = []
    for name in col_names:
        if re.search(base_name,name):
            if re.search("heating power",name):
                power_names.append(name)
            else:
                temp_names.append(name)

    pylab.figure()

    pylab.subplot(2,1,1)
    pylab.title(base_name + " temperatures")
    pylab.xlabel('Time (Minutes)')
    pylab.ylabel('Temp (Celcius)')
    for name in temp_names:
        col_data = td.getColumnByName(name)
        pylab.plot(rel_times,col_data,label=name)
    pylab.legend(loc='best')


    pylab.subplot(2,1,2)
    pylab.title(base_name + " power")
    pylab.xlabel('Time (Minutes)')
    pylab.ylabel('Heating Power (Watts)')
    for name in power_names:
        col_data = td.getColumnByName(name)
        pylab.plot(rel_times,col_data,label=name)
    pylab.legend(loc='best')




def main(argv):
    progname = argv[0]
    optlist, argv = getopt.gnu_getopt(argv[1:], "htp:", ["help", "test"])
    for (opt, val) in optlist:
        if opt == "--help" or opt == '-h':
            usage(progname)
            return 0
        elif opt == "--test" or opt == '-t':
            return 0
        else:
            print "Internal error : unhandled option '%s'"%opt
            return 1

    if len(argv) != 1:
      usage(progname)
      return 1

    incsv_filename = argv[0]
    td = TemperatureData()
    td.loadInputCSV(incsv_filename)

    times = td.getTimes()
    start = float(times[0])
    stop  = float(times[-1])
    print "Plots start at ", time.asctime(time.localtime(start))
    print "Plots stop  at ", time.asctime(time.localtime(stop))

    for caster in ('fr','fl','bl','br'):
        plotData(td, caster+'_caster_rotation_motor')

    pylab.show()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
