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
##\brief Finds error events in Realtime Loop diagnostics

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

from diagnostic_annotate.kv_convert import ConvertVar, KeyValueConvertList, VarStorage
from diagnostic_annotate.diag_event import generic_event

class RealtimeControlLoopDiag:
    """ Looks for issues occurring in 'Realtime Control Loop' """
    def __init__(self, diag_map):
        self.name = "Realtime Control Loop"
        diag_map[self.name] = self

        self.level = 0 
        self.message = "OK"

        kvl = KeyValueConvertList()
        kvl.add('Control Loop Overruns', ConvertVar('control_loop_overruns', int, 0))
        kvl.add('Max EtherCAT roundtrip (us)', ConvertVar('max_ethercat_roundtrip', float, 0.0))
        kvl.add('Max Controller Manager roundtrip (us)', ConvertVar('max_controller_manager_roundtrip', float, 0.0))

        self.kvl = kvl
        self.old = VarStorage()
        kvl.set_defaults(self.old)
        
    def process(self, msg, t):        
        event_list = []

        name = self.name
        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        #There are too many control loop overruns, don't print this out for now
        #if new.control_loop_overruns != old.control_loop_overruns:
        #    event_list.append("%d new control_loop_overruns" % (new.control_loop_overruns - old.control_loop_overruns))
            
        if (new.max_ethercat_roundtrip > old.max_ethercat_roundtrip) and (new.max_ethercat_roundtrip > 1000):
            event_list.append(generic_event(name, t, "Max ethercat roundtrip %f" % (new.max_ethercat_roundtrip)))

        if (new.max_controller_manager_roundtrip > old.max_controller_manager_roundtrip) and (new.max_controller_manager_roundtrip > 1000):
            event_list.append(generic_event(name, t, "Max controller roundtrip %f" % (new.max_controller_manager_roundtrip)))

        self.old = new
        self.level = msg.level
        self.message = msg.message

        return event_list
