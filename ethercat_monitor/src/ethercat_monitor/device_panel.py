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
##\brief Provide GUI panel to view EtherCAT device related errors.

import wx
import wx.grid
import wx.lib.scrolledpanel

from ethercat_monitor.cell_data import CellData, cell_data_empty

from ethercat_monitor.wx_util import levelToBackgroundColor


class DevicePanel(wx.lib.scrolledpanel.ScrolledPanel):
    def __init__(self, parent):
        wx.lib.scrolledpanel.ScrolledPanel.__init__(self,parent,-1)

        # Device grid
        device_grid = wx.grid.Grid(self) 
        device_grid.CreateGrid(0,12)
        device_grid.SetColLabelValue(0, "Device")
        device_grid.SetColLabelValue(1, "HWID");
        device_grid.SetColLabelValue(2, "Position")
        device_grid.SetColLabelValue(3, "Valid")
        device_grid.SetColLabelValue(4, "Port")
        device_grid.SetColLabelValue(5, "LostLink")
        device_grid.SetColLabelValue(6, "Rx")
        device_grid.SetColLabelValue(7, "Est Drops")

        device_grid.SetColLabelValue(8, "Empty")
        device_grid.SetColLabelValue(9, "FwdRx")
        device_grid.SetColLabelValue(10, "EPU")
        device_grid.SetColLabelValue(11, "PDI")

        device_grid.SetRowLabelSize(0) # hide the row labels
        device_grid.AutoSize()
        self.device_grid = device_grid

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.device_grid, wx.EXPAND)

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        self.SetupScrolling()


    def updateDeviceGrid(self, tsd):
        """ Update grid with timestamp data """
        # Order list of devices by productID
        ordered_devices = []
        for name,device in tsd.devices.iteritems():
            ordered_devices.append( (device.ring_position, name, device) )
        ordered_devices.sort()

        # Generate list of lists with device data. 
        data = []
        ERROR = CellData.ERROR
        WARN = CellData.WARN
        DATA = CellData.DATA
        for position,name,device in ordered_devices:
            new_data = [ [ cell_data_empty for unused in range(12) ] for port in device.ports ]
            for num,port in enumerate(device.ports):
                row_data = new_data[num]
                row_data[4] = CellData('Port%d'%num)
                row_data[5] = CellData(port.lost_links, ERROR if (port.lost_links != 0) else DATA)
                row_data[6] = CellData(port.rx_errors, ERROR if (port.rx_errors != 0) else DATA)
                if port.est_drops is None:                
                    row_data[7] = CellData("?", WARN)
                else:
                    row_data[7] = CellData(port.est_drops, ERROR if (port.est_drops != 0) else DATA)
                row_data[9] = CellData(port.forwarded_rx_errors, WARN if (port.forwarded_rx_errors != 0) else DATA)
            row_data = new_data[0] 
            row_data[0]  = CellData(name, DATA if (device.valid) else ERROR)
            row_data[1]  = CellData(device.hardware_id, DATA if (device.valid) else ERROR)
            row_data[2]  = CellData(device.ring_position, ERROR if (device.ring_position is None) else DATA)
            row_data[3]  = CellData(device.valid, DATA if (device.valid) else ERROR)            
            row_data[10]  = CellData(device.epu_errors, ERROR if (device.epu_errors != 0) else DATA)
            row_data[11] = CellData(device.pdi_errors, ERROR if (device.pdi_errors != 0) else DATA)
            data += new_data

        # Re-adjust grid to have appropriate number of rows
        grid = self.device_grid
        diff = len(data) - grid.GetNumberRows()
        if diff > 0:
            grid.AppendRows(diff)
        elif diff < 0:
            grid.DeleteRows(0,diff)

        for row,row_data in enumerate(data):
            for col,cell_data in enumerate(row_data):
                grid.SetCellValue(row,col,str(cell_data))
                bg_color = levelToBackgroundColor(cell_data.level)
                grid.SetCellBackgroundColour(row,col,bg_color)
                grid.SetReadOnly( row, col );

        grid.AutoSize()

