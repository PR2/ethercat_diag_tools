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

"""
Converts key-value pairs in DiagnosticStatus messsage into object attributes
@author Derek King

DiagnosticStatus messages store information as an array of key-value pairs.
Writing any code that uses parses diagnostic array becomes very difficuit for 
a variety of reasons:
 * key names are human readable strings (ie 'RX Error Port 1')
 * key naming is not fixed and can change from version to version (ie 'RX error port 1' --> 'RX Error Port 1') 
 * values are always strings (ie '1000' or '1.234')
This module has classes that are provided that a list of keys and convert them to 
instance variable in a given object using a given conversion function.  
The instance variable can have a different name that the key (ie 'Num encoder_errors' --> 'encoder_errors')
which make code that uses the instance variables easier to read and less prone to typos :
   var1.encoder_errors - var2.encoder_errors 
 Instead of:
   var1['Num encoder_errors'] - var2['Num encoder_errors']
Using a different name for the instance variable also gives a simple method for handling key changes:
  'RX Error Port 1' --> rx_error_1
  'RX error port 1' --> rx_error_1
"""

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import unittest

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

class ConvertVar:
    def __init__(self, var_name, convert_func, default):
        if var_name[0] == '_':
            raise Exception("Cannot use var_name begining with '_'")
        self.var_name = var_name
        self.convert_func = convert_func
        self.default = default

    def convert(self, obj, value):
        setattr(obj, self.var_name, self.convert_func(value))

    def set_default(self, obj):
        setattr(obj, self.var_name, self.default)

class ConvertList:
    def __init__(self, var_name, convert_func, index, default):
        if var_name[0] == '_':
            raise Exception("Cannot use var_name begining with '_'")
        self.var_name = var_name
        self.convert_func = convert_func
        self.index = index
        self.default = default
        
    def _set(self, obj, value):
        l = getattr(obj, self.var_name, [])
        if len(l) < (self.index+1):
            for i in range(self.index+1-len(l)):
                l.append(None)
        l[self.index] = value
        setattr(obj, self.var_name, l)

    def convert(self, obj, value):
        self._set(obj, self.convert_func(value))

    def set_default(self, obj):
        self._set(obj, self.default)

class VarStorage:
    """ Used to store variables """

class KeyValueConvertList:
    def __init__(self):
        self._fields = {}

    def add( self, key_name, convert_obj):
        """ Add conversion to list """ 
        self._fields[key_name] = convert_obj

    def convert(self, msg, obj):
        """ Convert values contained in DiagnosticStatus message into variable in obj """
        for kv in msg.values:
            convert_obj = self._fields.get(kv.key)
            if convert_obj != None:
                convert_obj.convert(obj, kv.value)

    def set_defaults(self, obj):
        """ Sets default variable values in object """
        for convert_obj in self._fields.itervalues():
            convert_obj.set_default(obj)

class TestKVConvert(unittest.TestCase):
    def setUp(self):
        #self.package_path = rospack_find(PKG)
        pass

    def test_1_convert_var(self):
        cv = ConvertVar('var', int, -1)
        vs = VarStorage()
        # Make default value gets set correctly, ie var == -1
        cv.set_default(vs)
        self.assertTrue(vs.var == -1)
        # Make sure conversion update array
        cv.convert(vs, '42')
        self.assertTrue(vs.var == 42, 'vs.var == %d'%vs.var)
    
    def test_2_convert_list(self):
        #kvl = KeyValueConvertList()
        cl = ConvertList('array', int, 3, -1)
        vs = VarStorage()        
        # Make default value gets set correctly, ie array[3] == -1
        cl.set_default(vs)
        self.assertTrue(vs.array[3] == -1)
        # Make sure conversion update array
        cl.convert(vs, '42')
        self.assertTrue(vs.array[3] == 42, 'vs.array[3] == %d'%vs.array[3])
        
    def test_3_kv_convert(self):
        kvl = KeyValueConvertList()
        kvl.add("IntVar", ConvertVar('ivar', int, -1))
        kvl.add("FloatVar", ConvertVar('fvar', float, -1.0))
        for i in range(2):
            kvl.add('Array %d'%(i+1), ConvertList('array', int, i, -1))
        # Make sure defaults get set
        vs = VarStorage()
        kvl.set_defaults(vs)
        self.assertTrue(vs.ivar == -1)
        self.assertTrue(vs.fvar == -1.0)
        self.assertTrue(vs.array[0] == -1)
        self.assertTrue(vs.array[1] == -1)
        # Try test conversion, by parsing a diagnostics message
        values = [KeyValue('IntVar','1000'), KeyValue('FloatVar', '4.2'), KeyValue('Array 1', '1'), KeyValue('Array 2', '2')]
        msg = DiagnosticStatus(0, 'NAME', 'MSG', 'HWID', values)
        kvl.convert(msg, vs)
        self.assertTrue(vs.ivar == 1000)
        self.assertTrue(vs.fvar == 4.2)
        self.assertTrue(vs.array[0] == 1)
        self.assertTrue(vs.array[1] == 2)


if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG,'test_kv_convert', TestKVConvert)
    #unittest.main()
