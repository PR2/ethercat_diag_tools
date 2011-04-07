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
##\brief Keeps track of (RX errors, lost links, etc...) of ethercat device

PKG = 'ethercat_monitor'
import roslib
roslib.load_manifest(PKG)
from ethercat_monitor.cell_data import CellData, cell_data_empty

class EtherCATDevicePortStatus:
    def __init__(self):
        self.rx_errors = 0
        self.forwarded_rx_errors = 0
        self.frame_errors = 0
        self.lost_links = 0
        self.est_drops = 0.0

    def __str__(self):
        result  = "   RX errors     : %d\n" % self.rx_errors
        result += "   FWD RX errors : %d\n" % self.forwarded_rx_errors
        result += "   Invalid Frame : %d\n" % self.frame_errors
        result += "   Lost Links    : %d\n" % self.lost_links
        result += "   Est drops     : %f\n" % self.est_drops
        return result

    def getDiff(self, port_old):
        """ Returns difference in error counters between this and old values"""
        port_diff = EtherCATDevicePortStatus()
        port_diff.rx_errors = self.rx_errors - port_old.rx_errors
        port_diff.forwarded_rx_errors = self.forwarded_rx_errors - port_old.forwarded_rx_errors
        port_diff.frame_errors = self.frame_errors - port_old.frame_errors
        port_diff.lost_links = self.lost_links - port_old.lost_links
        port_diff.est_drops = self.est_drops - port_old.est_drops

        if self.open != port_old.open:
            port_diff.open = None
        else:
            port_diff.open = self.open

        return port_diff

    def generateYaml(self):
        out = {}
        out['rx_errors'] = self.rx_errors
        out['forwarded_rx_errors'] = self.forwarded_rx_errors
        out['frame_errors'] = self.frame_errors
        out['lost_links'] = self.lost_links
        out['est_drops'] = self.est_drops
        return out


class EtherCATDevicePortMissing:
    """ Represets missing EtherCAT port data"""
    def __str__(self):
        return "   PORT MISSING"

    def getDataGrid(self,port_num):
        empty = cell_data_empty #CellData()
        return [empty,empty,empty, CellData('Port%d'%port_num,CellData.ERROR),empty,empty,empty]


class EtherCATDeviceStatus:
    def __init__(self,num_ports):
        self.ports = [EtherCATDevicePortStatus() for i in range(num_ports)]
        self.ring_position = None
        self.epu_errors = 0
        self.pdi_errors = 0
        self.valid = False

    def __str__(self):
        result  = "  EPU errors : %d\n" % self.epu_errors
        result += "  PDI errors : %d\n" % self.pdi_errors
        for port_num,port in enumerate(self.ports):
            result += "  Port " + str(port_num) + " : \n" + str(port)
        return result

    def getDiff(self,device_status_old):        
        ds_old = device_status_old
        num_ports = max(len(self.ports), len(ds_old.ports))
        ds_diff = EtherCATDeviceStatus(0)
        ds_diff.epu_errors = self.epu_errors - ds_old.epu_errors
        ds_diff.pdi_errors = self.pdi_errors - ds_old.pdi_errors
        ds_diff.valid = self.valid and ds_old.valid
        if self.hardware_id == ds_old.hardware_id:
            ds_diff.hardware_id = self.hardware_id
        else:
            ds_diff.hardware_id = "Mismatch %s != %s" % (self.hardware_id, ds_old.hardware_id)
        for num in range(num_ports):
            if (num >= len(self.ports)) or (num >= len(ds_old.ports)):
                ds_diff.ports.append(EtherCATDevicePortMissing())
            else:
                ds_diff.ports.append(self.ports[num].getDiff(ds_old.ports[num]))
        if self.ring_position != ds_old.ring_position:
            ds_diff.ring_position = None
        else:
            ds_diff.ring_position = self.ring_position

        return ds_diff



    def generateYaml(self):
        out = {}
        out['valid'] = self.valid
        out['position'] = self.ring_position
        out['epu_errors'] = self.epu_errors
        out['pdi_errors'] = self.epu_errors

        port_out = {}
        for num,port in enumerate(self.ports):
            port_out[num] = port.generateYaml()
        out['ports'] = port_out

        return out


class EtherCATDeviceMissing:
    """ Represents that absense of EtherCAT device that should be present. 
    implements most of the same functions as EtherCATDeviceStatus
    """
    def __str__(self):
        return "MISSING DEVICE"

    def getDataGrid(self,name):
        empty = cell_data_empty #CellData()
        return [CellData(name, CellData.ERROR), empty, empty, empty, empty, empty, empty]
