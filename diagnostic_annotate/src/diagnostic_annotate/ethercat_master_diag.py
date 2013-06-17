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

def true_false_to_bool(val):
    if val == 'false':
        return False
    elif val == 'true':
        return True
    raise RuntimeError("%s is not 'true' or 'false'" % val)

def only_motor_halted_event(name, t, desc):
    """ Represents error from master of just 'Motors Halted'."""
    return DiagEvent('OnlyMotorHaltedEvent',name, t, desc)

def motors_halted(name, t, desc, first):
    """ Represents transition of EtherCAT Master motor halted value from false to true"""
    d = DiagEvent('MotorsHalted', name, t, "Motors halted : %s" % desc)
    d.data = {'first':first}
    return d

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
    
def other_eml_event(name, t, new_other_eml, old_other_eml):
    """ Represents seeing packets from different EtherCAT master """
    evt = DiagEvent('OtherEML',name, t, '%d packets from other EtherCAT Master' % (new_other_eml - old_other_eml))
    evt.data = {'new' : new_other_eml, 'old' : old_other_eml}
    return evt

def ecat_time_overrun_event(name, t, cause, _time):
    """ Represents EtherCAT hardware stage that takes too much time """
    evt = DiagEvent('EcatTimeOverrun', name, t, "%s took too much time : %f us" % (cause, _time))
    evt.data = {'time' : _time}
    return evt

class EtherCATMasterDiag:
    """ Looks for errors in EtherCAT Master """
    def __init__(self, diag_map):
        self.name = 'EtherCAT Master'
        diag_map[self.name] = self

        self.level = 0 
        self.message = "OK"
        self.first = True

        kvl = KeyValueConvertList()
        kvl.add('Dropped Packets', ConvertVar('dropped_packets', int, 0))
        kvl.add('RX Late Packet', ConvertVar('late_packets', int, 0))
        kvl.add('RX Other EML', ConvertVar('other_eml', int, 0))
        kvl.add('Motors halted', ConvertVar('motors_halted', true_false_to_bool, False))

        # look at max timing information and complain if something took too long
        kvl.add('Roundtrip time 1 Sec Max (us)',    ConvertVar('max_ecat_rtt', float, None))
        kvl.add('Pack command time 1 Sec Max (us)', ConvertVar('max_pack_time', float, None))
        kvl.add('Unpack state time 1 Sec Max (us)', ConvertVar('max_unpack_time', float, None))
        kvl.add('Publish time 1 Sec Max (us)',      ConvertVar('max_publish_time', float, None))

        self.kvl = kvl                             

        self.old = VarStorage()
        kvl.set_defaults(self.old)
    
    def process(self, msg, t):
        """ Returns array of error descriptions this message my have caused """
        event_list = []

        old = self.old
        new = VarStorage()
        name = self.name
        self.kvl.convert(msg, new)

        if msg.level > self.level:
            if msg.level==2 and msg.message == 'Motors halted':
                error = only_motor_halted_event(name, t, msg.message)
            else:
                error = generic_event(name, t, "transitioned into level %d : %s" % (msg.level, msg.message))
            event_list.append(error)
        elif msg.level != 0 and msg.message != self.message:
            event_list.append(generic_event(name, t, "message changed to %s" % (msg.message)))

        if new.dropped_packets != old.dropped_packets:
            event_list.append(dropped_packet_event(name, t, (new.dropped_packets - old.dropped_packets)))

        if new.late_packets != old.late_packets:                                 
            event_list.append(late_packet_event(name, t, (new.late_packets - old.late_packets)))

        # limit rate of this event, since if another EtherCAT master is running, this value while be constantly increasing 
        if new.other_eml != old.other_eml:
            if (old.other_eml == 0) or ((new.other_eml - old.other_eml) > 10000):
                event_list.append(other_eml_event(name, t, new.other_eml, old.other_eml))
                        
        if new.motors_halted and not old.motors_halted:
            event_list.append(motors_halted(name, t, msg.message, self.first))

        # if any part of timeing loop takes longer than 1-ms then it broke the realtime loop
        # old diagnostics do not contain detailed timing information
        if hasattr(new, 'max_ecat_rtt'):
            if new.max_ecat_rtt > 2000.0: 
                # this happens so much, only pickout stuff that is longer than 2ms
                event_list.append(ecat_time_overrun_event(name, t, 'EtherCAT RTT', new.max_ecat_rtt))
            if new.max_pack_time > 1000.0:
                event_list.append(ecat_time_overrun_event(name, t, 'Pack', new.max_pack_time))
            if new.max_unpack_time > 1000.0:
                event_list.append(ecat_time_overrun_event(name, t, 'Unpack', new.max_unpack_time))
            if new.max_publish_time > 1000.0:
                event_list.append(ecat_time_overrun_event(name, t, 'Publish', new.max_publish_time))

        self.level = msg.level
        self.message = msg.message

        self.old = new
        self.first = False

        return event_list
