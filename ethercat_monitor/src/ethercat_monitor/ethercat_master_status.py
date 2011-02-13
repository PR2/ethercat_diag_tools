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
##\brief Keeps track of ethercat master status (sent packets, dropped packets, etc)

PKG = 'ethercat_monitor'
import roslib
roslib.load_manifest(PKG)
from ethercat_monitor.cell_data import CellData, cell_data_empty


class EtherCATMasterStatus:
    def __init__(self, sent=0, dropped=0, late=0):
        self.sent    = sent
        self.dropped = dropped
        self.late    = late

    def __str__(self):
        result  = "   Sent     : %d\n" % self.sent
        result += "   Dropped  : %d\n" % self.dropped
        result += "   Late     : %d\n" % self.late
        return result

    def getDataGrid(self):
        ERROR = CellData.ERROR
        WARN = CellData.WARN
        DATA = CellData.DATA
        empty = cell_data_empty #CellData()
        data = [CellData(str(self.sent))]
        data.append(CellData(self.dropped, ERROR if (self.dropped > 0) else DATA))
        data.append(CellData(self.late   , WARN  if (self.late    > 0) else DATA))
        return data

    def getDiff(self, old):
        """ Returns difference in counters between this and old values"""
        diff = EtherCATMasterStatus()
        diff.sent    = self.sent    - old.sent
        diff.dropped = self.dropped - old.dropped
        diff.late    = self.late    - old.late
        return diff

