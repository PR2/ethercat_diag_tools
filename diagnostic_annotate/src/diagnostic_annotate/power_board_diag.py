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
##\brief Finds error events in PowerBoard diagnostics

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

from diagnostic_annotate.kv_convert import ConvertVar, KeyValueConvertList, VarStorage
from diagnostic_annotate.diag_event import DiagEvent, generic_event

import re
        
def runstop_event(name, t, desc):
    """ Represents Run-Stop being pressed. """
    return DiagEvent('RunStopEvent', name, t, desc)

def str_to_bool(str):
    if (str == "True"):
        return True
    elif (str == "False"):
        return False
    else:
        raise Exception("Not boolean : %s" % str)

class PowerBoardDiag:
    """ Looks for issues occurring in 'Power Board' """
    def __init__(self, diag_map, name):
        self.name = name
        diag_map[self.name] = self

        self.level = 0 
        self.message = "Running"
        self.old_runstop_status = True

        kvl = KeyValueConvertList()
        kvl.add('RunStop Button Status', ConvertVar('runstop_button_status', str_to_bool, True))
        kvl.add('RunStop Wireless Status', ConvertVar('runstop_wireless_status', str_to_bool, True))

        self.kvl = kvl
        self.old = VarStorage()
        kvl.set_defaults(self.old)
        
    def process(self, msg, t):        
        event_list = []

        name = self.name
        old = self.old 
        new = VarStorage()
        self.kvl.convert(msg, new)

        new_runstop_status = new.runstop_button_status and new.runstop_wireless_status
        
        if not new_runstop_status and self.old_runstop_status:
            desc = "Runstop" #Button=%s Wireless=%s" % (str(new.runstop_button_status), str(new.runstop_wireless_status))
            event_list.append(runstop_event(name, t, desc))            

        self.old = new
        self.level = msg.level
        self.message = msg.message

        self.old_runstop_status = new_runstop_status

        return event_list


class PowerBoardAddDiag:
    """ Looks for Power Board Devices and adds a PowerBoard for them """
    def __init__(self, diag_list, diag_map):
        self.name = 'PowerBoardAddDiag'
        diag_list.append(self)
        self.diag_list = diag_list
        self.diag_map = diag_map
        self.is_power_board = re.compile("Power board [0-9]{4}$")

    def is_match(self, msg):
        m = self.is_power_board.match(msg.name)
        return m != None
            
    def process(self, msg, t):
        name = msg.name
        #print "New Power Board : %s" % name
        dev = PowerBoardDiag(self.diag_map, name)
        return dev.process(msg,t)
