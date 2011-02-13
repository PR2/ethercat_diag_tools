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
##\brief Finds error events in EtherCAT Device diagnostics


PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)

from diagnostic_annotate.kv_convert import ConvertVar, ConvertList, KeyValueConvertList, VarStorage
from diagnostic_annotate.diag_event import DiagEvent, generic_event

import re

def safety_disable_event(name, t, desc):
    """ Represents any type of EtherCAT device safety disable """
    return DiagEvent('SafetyDisableEvent', name, t, desc)

def undervoltage_lockout_event(name, t, desc):
    """ Represents undervoltage lockout of EtherCAT device """
    return DiagEvent('UndervoltageLockoutEvent', name, t, desc)

def rx_error_event(name, t, port, rx_errors):
    """ Represents RX errors on specific port of an EtherCAT device"""
    evt =  DiagEvent('RxError', name, t, '%d RX errors on port %d' % (rx_errors, port))
    evt.data = {'port' : port, 'rx_errors' : rx_errors}
    return evt

def lost_link_event(name, t, port, lost_links):
    """ Represents lost link on specific port of an EtherCAT device"""
    evt =  DiagEvent('LostLink', name, t, "%d lost links on port %d" % (lost_links, port))
    evt.data = {'port' : port, 'rx_errors' : lost_links}
    return evt

def motor_model_error(name, t):
    """ Represents motor model error"""
    return DiagEvent('MotorModelError', name, t, "Motor model error")

def motor_model_warning(name, t):
    """ Represents motor model warning"""
    return DiagEvent('MotorModelWarning', name, t, "Motor model warning")


class SafetyDisableStatus:
    def __init__(self, str):
        self.undervoltage = str.find("UNDERVOLTAGE") != -1
        self.over_current = str.find("OVER_CURRENT") != -1
        self.board_overtemp = str.find("BOARD_OVER_TEMP") != -1
        self.bridge_overtemp = str.find("HBRIDGE_OVER_TEMP") != -1
        self.operational = str.find("OPERATIONAL") != -1
        self.watchdog = str.find("WATCHDOG") != -1
        self.disabled = str.find("DISABLED") != -1
        self.enabled = str.find("ENABLED") != -1
        self.str = str

        if (self.disabled == self.enabled):
            raise Exception("disabled and enabled both set in %s", str)

    def compare(self, old):
        """ Compares this safety disable status against old status.
        returns true if safety disable is set that was not in old.  
        """
        return (self.undervoltage and not old.undervoltage) or \
            (self.watchdog and not old.watchdog) or \
            (self.bridge_overtemp and not old.bridge_overtemp) or \
            (self.board_overtemp and not old.board_overtemp) or \
            (self.over_current and not old.over_current) or \
            (self.operational and not old.operational) or \
            (self.disabled and not old.disabled)

    def just_undervoltage(self):
        """ Returns true if safety disable status is just undervoltage """
        return self.undervoltage and not (self.over_current or self.board_overtemp or self.bridge_overtemp or self.operational or self.watchdog)
        
    def to_str(self):
        return self.str


def decode_safety_disable(value):
    """ Return safety disable status object """
    return SafetyDisableStatus(value)


class EtherCATDeviceDiag:
    """ Looks for errors in a specific EtherCAT Device """
    def __init__(self, diag_map, name, num_ports, has_encoder):
        self.name = name
        diag_map[self.name] = self
        self.num_ports = num_ports
        self.has_encoder = has_encoder

        kvl = KeyValueConvertList()
        kvl.add('Safety Disable Status Hold', ConvertVar('safety_disable_status_hold', decode_safety_disable, SafetyDisableStatus("ENABLED (00)")))
        kvl.add('Num encoder_errors', ConvertVar('encoder_errors', int, 0))
        for i in range(4):
            kvl.add('RX Error Port %d'%i, ConvertList('rx_error', int, i, 0))
            kvl.add('Lost Link Port %d'%i, ConvertList('lost_link', int, i, 0))
        self.kvl = kvl

        self.motor_model_error_re   = re.compile("Problem with the MCB, motor, encoder, or actuator model", re.IGNORECASE)
        self.motor_model_warning_re = re.compile("Potential problem with the MCB, motor, encoder, or actuator model",  re.IGNORECASE)
        self.has_motor_model_error   = False
        self.has_motor_model_warning = False
        
        self.old = VarStorage()
        kvl.set_defaults(self.old)

        
    def process(self, msg, t):        
        event_list = []

        name = self.name
        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        # Look for motor model warning or motor model errors
        if msg.level == 2:
            has_motor_model_error = bool(self.motor_model_error_re.search(msg.message))
            if not self.has_motor_model_error and has_motor_model_error:
                event_list.append(motor_model_error(self.name, t))
            self.has_motor_model_error   = has_motor_model_error
            self.has_motor_model_warning = False
        elif msg.level == 1:
            has_motor_model_warning = bool(self.motor_model_warning_re.search(msg.message))
            if not self.has_motor_model_warning and has_motor_model_warning:
                event_list.append(motor_model_warning(self.name, t))            
            self.has_motor_model_error   = False
            self.has_motor_model_warning = has_motor_model_warning
        else:
            self.has_motor_model_error   = False
            self.has_motor_model_warning = False

        if self.has_encoder and new.encoder_errors != old.encoder_errors:
            event_list.append(generic_event(name, t, "%d new encoder errors" % (new.encoder_errors - old.encoder_errors)))

        if (new.safety_disable_status_hold.compare(old.safety_disable_status_hold)):
            if (new.safety_disable_status_hold.just_undervoltage()):
                event_list.append(undervoltage_lockout_event(name, t, "undervoltage lockout"))
            else:
                event_list.append(generic_event(name, t, "safety disable status changed to %s" % (new.safety_disable_status_hold.to_str())))
            
        num_ports = len(new.rx_error)
        if num_ports != self.num_ports:
            event_list.append(generic_event(name, t, "changing number of ports from %d to %d" % (self.num_ports, num_ports)))
            self.num_ports = num_ports

        for port in range(self.num_ports):
            if new.rx_error[port] != old.rx_error[port]:
                event = rx_error_event(name,t, port, (new.rx_error[port] - old.rx_error[port]))
                event_list.append(event)
            if new.lost_link[port] != old.lost_link[port]:
                event = lost_link_event(name,t,port,(new.lost_link[port] - old.lost_link[port]))
                event_list.append(event)

        self.old = new

        return event_list


class EtherCATDeviceAddDiag:
    """ Looks for EtherCAT devices that are not already present and adds a new EtherCAT Device Diag for them """

    def __init__(self, diag_list, diag_map):
        self.name = 'EthercatDeviceAddDiag'
        diag_list.append(self)

        self.diag_list = diag_list
        self.diag_map = diag_map

        self.is_ethercat_device = re.compile("EtherCAT Device (#\d\d )?\(\w+\)")

    def is_match(self, msg):
        m = self.is_ethercat_device.match(msg.name)
        return m != None
            
    def process(self, msg, t):
        name = msg.name
        #print "Found EtherCAT Device %s" % name

        # Use the HW id to figure number of ports and whether device has encoder
        num_ports = 1
        has_encoder = False
        if (re.match("68-05005-[0-9]{5}$", msg.hardware_id)):
            num_ports = 2
            has_encoder = True
        elif (re.match("68-05006-[0-9]{5}$", msg.hardware_id)):
            num_ports = 1
            has_encoder = True
        elif (re.match("68-05014-[0-9]{5}$", msg.hardware_id)):
            num_ports = 4
        elif (re.match("68-05021-[0-9]{5}$", msg.hardware_id)):
            num_ports = 2
        else:
            print "Don't understand hardware_id = ", msg.hardware_id

        dev = EtherCATDeviceDiag(self.diag_map, name, num_ports, has_encoder)

        return dev.process(msg,t)
