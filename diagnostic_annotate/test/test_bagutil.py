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
##\brief Unit test for bagutil

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

import rospy
import rosbag
import sys 
import unittest
import os.path 
from diagnostic_annotate.bagutil import *

from diagnostic_annotate.util import rospack_find

class TestBagUtil(unittest.TestCase):
    def setUp(self):
        self.package_path = rospack_find(PKG)        
        pass

    def test_1_get_yaml_bag_info(self):
        bag_path = os.path.join(self.package_path, 'test', 'bags', 'max_ecat_roundtrip_uncompressed.bag')
        b = rosbag.Bag(bag_path)
        y = get_yaml_bag_info(b)
        print y
        self.assertTrue(y['compression'] == 'none')
        b.close()

    def test_2_is_compressed(self):
        bag_path = os.path.join(self.package_path, 'test', 'bags', 'max_ecat_roundtrip_uncompressed.bag')
        self.assertFalse(is_compressed(bag_path))
        bag_path = os.path.join(self.package_path, 'test', 'bags', 'max_ecat_roundtrip_compressed.bag')
        self.assertTrue(is_compressed(bag_path))

if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG,'test_bagutil', TestBagUtil)
    #unittest.main()
