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
##\brief Holds base class for diagnostic 'Events'

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

from diagnostic_annotate.kv_convert import ConvertVar, KeyValueConvertList, VarStorage

import rospy
from collections import deque
import unittest
import time
import copy

class DiagEventFormatError(Exception):
    def __init__(self, desc):
        self.desc = desc
    def __str__(self):
        return self.desc

class DiagEvent(object):
    def __init__(self, type, name, t, desc, hide=False, children=[]):
        self.type = type     # event type / event class
        self.name = name     # component name
        self.t = t           # start-time of event 
        self.desc = desc     # description of event
        self.hide = hide     # hide event
        self.data = {}       # map for extra data
        self.children = children # events caused by this error
    def short_desc(self):
        """ Return short description of event. """
        return self.desc
    def to_yaml(self):
        """ Returns dict that can easily be converted to yaml """
        child_yaml_list = []
        for child in self.children:
            child_yaml_list.append(child.to_yaml())
        timestr = DiagEvent.time_to_yaml(self.t)
        result = {'type':self.type, 'name':self.name, 'time':timestr, 'desc':self.desc}
        if self.hide:
            result['hide'] = self.hide
        if len(child_yaml_list) > 0:
            result['children'] = child_yaml_list
        if len(self.data) > 0:
            result['data'] = self.data
        return result

    @staticmethod
    def time_to_yaml(t):
        """ Converts ros time to dict that can be used for yaml file """
        return {'secs':t.secs,'nsecs':t.nsecs}

    @staticmethod
    def time_from_yaml(yaml):
        """ Converts dict from yaml file to ros time """
        if type(yaml).__name__ != 'dict':
            raise DiagEventFormatError('expected a dict : got %s' % type(yaml).__name__)
        if 'secs' not in yaml or 'nsecs' not in yaml:
            raise DiagEventFormatError('expected secs and nsecs in dict')
        try :
            secs = int(yaml['secs'])
            nsecs = int(yaml['nsecs'])
            return rospy.Time(secs,nsecs)
        except ValueError,e:
            raise DiagEventFormatError(str(e))

    @staticmethod
    def from_yaml(yaml):
        """ Converts dict (potentially from yaml file) that is converted into DiagEvent Class """
        if type(yaml).__name__ != 'dict':
            raise DiagEventFormatError('expected a dict : got %s' % type(yaml).__name__)
        if 'type' not in yaml or 'name' not in yaml or 'time' not in yaml:
            raise DiagEventFormatError('expected type, name, and time fields in dict')
        desc = yaml.get('desc', '')
        hide = yaml.get('hide', False)
        children_yaml = yaml.get('children',[])
        data_yaml = yaml.get('data', {})
        if type(hide).__name__ != 'bool':
            raise DiagEventFormatError('hide must be bool, got %s' % type(hide).__name__)
        if type(children_yaml).__name__ != 'list':
            raise DiagEventFormatError('chidren must be list, got %s' % type(children_yaml).__name__)
        if type(data_yaml).__name__ != 'dict':
            raise DiagEventFormatError('data must be dict, got %s' % type(data_yaml).__name__)
        children = []
        for child_yaml in children_yaml:
            children.append(from_yaml(child_yaml))
        t = DiagEvent.time_from_yaml(yaml['time'])
        return DiagEvent(yaml['type'],yaml['name'],t,desc,hide,children)


    def __str__(self):
        timestr = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.t.to_sec()))
        return 'AT %s : %s : %s %s'%(timestr, self.type, self.name, self.desc)

    def __eq__(self,other):
        if type(other).__name__ != 'DiagEvent':
            return NotImplemented
        equal = (self.type == other.type) \
            and (self.name == other.name) \
            and (self.t    == other.t) \
            and (self.desc == other.desc) \
            and (self.hide == other.hide) \
            and (self.children == other.children)
        return equal

    def __ne__(self,other):
        return not self.__eq__(other)
        

def no_event(name, t, desc):
    return DiagEvent('NoEvent', name, t, desc)

def generic_event(name, t, desc):
    return DiagEvent('GenericEvent', name, t, desc)


class TestDiagEvent(unittest.TestCase):
    # Test that EventDelay class delays error messages properly
    def setUp(self):
        pass

    def test_1_time_to_yaml(self):
        """ Tests time_to_yaml """
        t = rospy.Time(1,2)
        yaml = DiagEvent.time_to_yaml(t)
        expected_yaml = {'secs':1, 'nsecs':2}
        self.assertTrue(yaml == expected_yaml)

    def test_2_time_from_yaml(self):
        """ Test time_from_yaml """
        t1 = rospy.Time(1,2)
        yaml = DiagEvent.time_to_yaml(t1)
        t2 = DiagEvent.time_from_yaml(yaml)
        self.assertTrue(t1 == t2)

    def test_3_str_time(self):
        """ Test time_from_yaml when time values are strings"""        
        t1 = rospy.Time(1,2)
        yaml = {'secs':'1', 'nsecs':'2'}
        t2 = DiagEvent.time_from_yaml(yaml)
        self.assertTrue(t1 == t2)

    def test_4_bad_time(self):
        """ Test time_from_yaml with different bad inputs """
        self.assertRaises(DiagEventFormatError, DiagEvent.time_from_yaml, {'secs':1} )
        self.assertRaises(DiagEventFormatError, DiagEvent.time_from_yaml, {'nsecs':1} )
        self.assertRaises(DiagEventFormatError, DiagEvent.time_from_yaml, {'secs':3, 'nsecs':'F'} )
        self.assertRaises(DiagEventFormatError, DiagEvent.time_from_yaml, ['secs','nsecs'] )
        self.assertRaises(DiagEventFormatError, DiagEvent.time_from_yaml, 1 )

    def test_5_simple_to_yaml(self):
        """ Tests to_yaml() on simple DiagEvent.  Simple == no children"""
        e = generic_event('component', rospy.Time(100), 'description')
        yaml = e.to_yaml()        
        timestr = DiagEvent.time_to_yaml(e.t)
        expected_yaml = {'type':e.type, 'name':e.name, 'time':timestr, 'desc':e.desc}
        self.assertTrue(yaml == expected_yaml)

        e.hide = True
        yaml = e.to_yaml()        
        timestr = DiagEvent.time_to_yaml(e.t)
        expected_yaml = {'type':e.type, 'name':e.name, 'time':timestr, 'desc':e.desc, 'hide':e.hide}
        self.assertTrue(yaml == expected_yaml)

    def test_6_from_yaml(self):
        """ Test convertion of object back from yaml-ish description """
        e1 = generic_event('component', rospy.Time(100), 'description')
        yaml = e1.to_yaml()
        e2 = DiagEvent.from_yaml(yaml)
        self.assertTrue(e1.type == e2.type)
        self.assertTrue(e1.name == e2.name)
        self.assertTrue(e1.t == e2.t)
        self.assertTrue(e1.hide == e2.hide)
        self.assertTrue(e1.children == e2.children)
        self.assertTrue(e1 == e2, "e1=%s, e2=%s"%(str(e1),str(e2)) )

    def copy_with_missing_var(self, yaml, var):
        yaml2 = copy.deepcopy(yaml)        
        del yaml2[var]
        return yaml2

    def test_7_bad_yaml(self):
        """ Test different cases of bad yaml """
        e = generic_event('component', rospy.Time(100), 'description')
        yaml1 = e.to_yaml()
        yaml2 = self.copy_with_missing_var(yaml1,'time')
        self.assertRaises(DiagEventFormatError, DiagEvent.from_yaml, yaml2)
        yaml2 = self.copy_with_missing_var(yaml1,'type')
        self.assertRaises(DiagEventFormatError, DiagEvent.from_yaml, yaml2)
        yaml2 = self.copy_with_missing_var(yaml1,'name')
        self.assertRaises(DiagEventFormatError, DiagEvent.from_yaml, yaml2)


if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG,'test_diag_event', TestDiagEvent)
                
    #unittest.main()
