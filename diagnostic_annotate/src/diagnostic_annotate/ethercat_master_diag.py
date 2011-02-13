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

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

from diagnostic_annotate.kv_convert import ConvertVar, KeyValueConvertList, VarStorage
from diagnostic_annotate.diag_event import DiagEvent, generic_event


def only_motor_halted_event(name, t, desc):
    """ Represents error from master of just 'Motors Halted' """
    return DiagEvent('OnlyMotorHaltedEvent',name, t, desc)

def dropped_packet_event(name, t, drops):
    """ Represents dropped packet from EtherCAT Master """
    evt = DiagEvent('DroppedPacket',name, t, 'dropped %d packets' % drops)
    evt.data = {'drops' : drops}
    return evt

def late_packet_event(name, t, lates):
    """ Represents late packet from EtherCAT Master """
    evt = DiagEvent('LatePacket',name, t, '%d late packets' % lates)
    evt.data = {'lates' : lates}
    return evt
    

class EtherCATMasterDiag:
    """ Looks for errors in EtherCAT Master """
    def __init__(self, diag_map):
        self.name = 'EtherCAT Master'
        diag_map[self.name] = self

        self.level = 0 
        self.message = "OK"

        kvl = KeyValueConvertList()
        kvl.add('Dropped Packets', ConvertVar('dropped_packets', int, 0))
        kvl.add('RX Late Packet', ConvertVar('late_packets', int, 0))
        self.kvl = kvl
                        
        self.old = VarStorage()
        kvl.set_defaults(self.old)
    
    def process(self, msg, t):
        """ Returns array of error descriptions this message my have caused """
        event_list = []

        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        if msg.level > self.level:
            if msg.level==2 and msg.message == 'Motors halted':
                error = only_motor_halted_event(self.name, t, msg.message)
            else:
                error = generic_event(self.name, t, "transitioned into level %d : %s" % (msg.level, msg.message))
            event_list.append(error)
        elif msg.level != 0 and msg.message != self.message:
            event_list.append(generic_event(self.name, t, "message changed to %s" % (msg.message)))

        if new.dropped_packets != old.dropped_packets:
            event_list.append(dropped_packet_event(self.name, t, (new.dropped_packets - old.dropped_packets)))

        if new.late_packets != old.late_packets:                                 
            event_list.append(generic_event(self.name, t, (new.late_packets - old.late_packets)))

        self.level = msg.level
        self.message = msg.message

        self.old = new

        return event_list
