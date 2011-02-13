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
##\brief Some extra utility functions for bag files

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

import rospy
import rosbag
import sys 
import os
import os.path
import subprocess
import yaml
import re

def get_yaml_bag_info(bag):
    """ Returns yaml-like dict of bag info 
    This is similar to what would be returned by ros command 'rosbag info -y'
    """
    # Fix, on current c-turtle release, the connection field is missing a new line, 
    # which causes an error in the yaml parser.
    # Example:
    # connections: 24      frequency: 31.8697
    # This work-around, puts a new line in when it is not present 
    y1 = bag._get_yaml_info()
    y2 = re.sub("(connections: [0-9]+)([\\t ]*frequency:)", "\\1\n\\2", y1)
    return yaml.safe_load(y2)

def _run_subprocess(args):
    #print args 
    sub = subprocess.Popen(args, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    out,err = sub.communicate()
    if sub.wait() != 0:
        raise RuntimeError(err)

def copy_bag(inbag_filename):
    """ Copy bag to current directory """
    #print "copying", inbag_filename
    new_name = os.path.basename(inbag_filename)
    _run_subprocess(['cp', inbag_filename, new_name])
    return new_name

def is_gzipped(inbag_filename):
    return os.path.splitext(inbag_filename)[1] == '.gz'

def gunzip_bag(inbag_filename):
    """ Uses gzip to decompress file, returns name of new file """
    #print "gunzipping", inbag_filename
    _run_subprocess(['gzip', '-df', inbag_filename])
    return os.path.splitext(inbag_filename)[0]

def is_indexed(inbag_filename):
    try:
        bag = rosbag.Bag(inbag_filename)
    except rosbag.ROSBagException:
        return False
    return True

def remove_original(inbag_filename):
    base,ext = os.path.splitext(inbag_filename)
    filename = base+'.orig'+ext
    #print "removing", filename
    _run_subprocess(['rm', filename])

def reindex_bag(inbag_filename):
    """ Uses rosbag reindex to add index to bag """
    #print "indexing", inbag_filename
    _run_subprocess(['rosbag', 'reindex', inbag_filename])
    remove_original(inbag_filename)
    return inbag_filename

def is_compressed(inbag_filename):
    b = rosbag.Bag(inbag_filename)
    y = get_yaml_bag_info(b)
    b.close()
    return y['compression'] != 'none'
               
def recompress_bag(inbag_filename):
    """ Recompresses bag file """
    #print "compressing", inbag_filename
    _run_subprocess(['rosbag', 'compress', inbag_filename])
    remove_original(inbag_filename)
    return inbag_filename

def is_fixed(inbag_filename):
    """ Check to see input bag has already been converted """
    name,ext = os.path.splitext(os.path.basename(inbag_filename))
    if ext == '.gz':
        inbag_filename = name
    return os.path.isfile(inbag_filename)
        
def bagfix(inbag_filename):
    """ Gunzip/recompress/index Bag file """
    if not os.path.isfile(inbag_filename): 
        raise RuntimeError("Cannot locate input bag file [%s]" % inbag_filename)
    if is_fixed(inbag_filename):
        print "Skipping", inbag_filename
        return
    filename = copy_bag(inbag_filename)
    if is_gzipped(filename):        
        filename = gunzip_bag(filename)
    if not is_indexed(filename):
        reindex_bag(filename)
    if not is_compressed(filename):
        recompress_bag(filename)
    print "Coverted %s to %s" %(inbag_filename, filename)

