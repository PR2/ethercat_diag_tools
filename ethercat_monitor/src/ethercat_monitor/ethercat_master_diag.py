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
##\brief Finds error events in EtherCAT Master diagnostics

PKG = 'ethercat_monitor'
import roslib
roslib.load_manifest(PKG)

from ethercat_monitor.kv_convert import ConvertVar, KeyValueConvertList, VarStorage
import ethercat_monitor.msg

class EtherCATMasterDiag:
    """ Looks at EtherCAT Master daignostics """
    def __init__(self):
        kvl = KeyValueConvertList()
        kvl.add('Sent Packets'   , ConvertVar('sent'   , int, 0))
        kvl.add('Dropped Packets', ConvertVar('dropped', int, 0))
        kvl.add('RX Late Packet' , ConvertVar('late'   , int, 0))
        self.old = VarStorage()
        kvl.set_defaults(self.old)
        self.kvl = kvl

    def isMatch(self, status):
        return (status.name == 'EtherCAT Master')

    def process(self, msg):
        """ Returns ethercat status """
        new = VarStorage()
        kvl = self.kvl
        kvl.convert(msg, new)
        return ethercat_monitor.msg.EtherCATMasterStatus(new.sent, new.dropped, new.late, 0)
