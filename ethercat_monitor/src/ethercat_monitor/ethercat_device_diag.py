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
##\brief Parses EtherCAT network counters from EtherCAT device diagnostics


PKG = 'ethercat_monitor'
import roslib
roslib.load_manifest(PKG)

from ethercat_monitor.kv_convert import ConvertVar, ConvertList, KeyValueConvertList, VarStorage

from ethercat_monitor.ethercat_device_status import EtherCATDeviceStatus, EtherCATDevicePortStatus

import re


class EtherCATDeviceDiag:
    """ Looks for network errors in a specific EtherCAT Device """
    def __init__(self, name, num_ports):
        self.name = name
        self.num_ports = num_ports
        position_in_name = re.match("EtherCAT Device #(\d\d)",name)
        if position_in_name is None:
            self.ring_position = None
        else:
            self.ring_position = int(position_in_name.group(1))

        kvl = KeyValueConvertList()

        kvl.add('PDI Errors', ConvertVar('pdi_errors', int, 0))
        kvl.add('EPU Errors', ConvertVar('epu_errors', int, 0))
        kvl.add('Position', ConvertVar('ring_position', int, None))
        for i in range(4):
            kvl.add('RX Error Port %d'%i, ConvertList('rx_errors', int, i, 0))
            kvl.add('Forwarded RX Error Port %d'%i, ConvertList('forwarded_rx_errors', int, i, 0))
            kvl.add('Lost Link Port %d'%i, ConvertList('lost_links', int, i, 0))
        self.kvl = kvl

        
    def process(self, msg): 
        new = VarStorage()
        self.kvl.convert(msg, new)
            
        num_ports = len(new.rx_errors)
        if num_ports != self.num_ports:
            print "WARNING : changing number of ports from %d to %d" % (self.num_ports, num_ports)
            self.num_ports = num_ports
        
        if getattr(new,'ring_position', None) is not None:
            if (self.ring_position is not None) and (self.ring_position != new.ring_position):
                print "WARNING : changing ring position of device from %d to %d" % (self.ring_position, new.ring_position)
            self.ring_position = new.ring_position

        device_status = EtherCATDeviceStatus(self.num_ports)
        device_status.epu_errors = new.epu_errors
        device_status.pdi_errors = new.pdi_errors
        device_status.ring_position = self.ring_position
        for port_num in range(self.num_ports):            
            port = device_status.ports[port_num]
            port.rx_errors = new.rx_errors[port_num]
            port.forwarded_rx_errors = new.forwarded_rx_errors[port_num]
            port.lost_links = new.lost_links[port_num]

        return device_status



class EtherCATDeviceAddDiag:
    """ Looks for EtherCAT devices that are not already present and adds a new EtherCAT Device Diag for them """

    def __init__(self, diag_map):
        #self.name = 'EthercatDeviceAddDiag'
        self.diag_map = diag_map
        self.is_ethercat_device = re.compile("EtherCAT Device (#\d\d )?\(\w+\)")


    def is_match(self, msg):
        m = self.is_ethercat_device.match(msg.name)
        return m != None
            
    def process(self, msg):
        name = msg.name
        #print "Found EtherCAT Device %s" % name

        # Use the HW id to figure number of ports and whether device has encoder
        num_ports = 1
        has_encoder = False
        if (re.match("68-05005-[0-9]{5}$", msg.hardware_id)):
            num_ports = 2
        elif (re.match("68-05006-[0-9]{5}$", msg.hardware_id)):
            num_ports = 1
        elif (re.match("68-05014-[0-9]{5}$", msg.hardware_id)):
            num_ports = 4
        elif (re.match("68-05021-[0-9]{5}$", msg.hardware_id)):
            num_ports = 2
        else:
            print "Don't understand hardware_id = ", msg.hardware_id

        dev = EtherCATDeviceDiag(name, num_ports)
        self.diag_map[name] = dev

        return dev.process(msg)
