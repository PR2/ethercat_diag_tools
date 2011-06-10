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
#import wx.lib.scrolledpanel

from ethercat_monitor.cell_data import CellData, cell_data_empty

from ethercat_monitor.wx_util import levelToBackgroundColor

from  ethercat_monitor.ethercat_port_order import genPortOrder

#wx.lib.scrolledpanel.ScrolledPanel
class DevicePanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, -1)
        #wx.lib.scrolledpanel.ScrolledPanel.__init__(self,parent,-1)


        self.num_rows = 14
        # Header of device grid.  Hack to keep header at top of grid while other grid scrolls
        #header_grid = wx.grid.Grid(self) 
        #self.header_grid = header_grid
        # Device grid
        device_grid = wx.grid.Grid(self)
        self.device_grid = device_grid

        device_grid.CreateGrid(0,self.num_rows)
        #header_grid.CreateGrid(0,self.num_rows)

        self.SetColLabelValue(0, "Device")
        self.SetColLabelValue(1, "HWID");
        self.SetColLabelValue(2, "Position")
        self.SetColLabelValue(3, "Valid")
        self.SetColLabelValue(4, "Port")
        self.SetColLabelValue(5, "Lost\nLinks")
        self.SetColLabelValue(6, "Frame\nErrors")
        self.SetColLabelValue(7, "Est\nDrops")
        self.SetColLabelValue(8, "Empty")
        self.SetColLabelValue(9, "Rx")
        self.SetColLabelValue(10, "Forward\nRx")
        self.SetColLabelValue(11, "EPU")
        self.SetColLabelValue(12, "PDI")
        self.SetColLabelValue(13, "Open")

        #header_grid.SetRowLabelSize(0) # hide the row labels
        device_grid.SetRowLabelSize(0) # hide the row labels

        device_grid.SetColLabelSize(device_grid.GetDefaultColLabelSize() * 2)
        device_grid.AutoSize()

        device_grid.EnableDragGridSize(False)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        #vsizer.Add(self.header_grid, 0, wx.EXPAND)
        vsizer.Add(self.device_grid, 1, wx.EXPAND)

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        #self.SetupScrolling()

        dir (self.device_grid)


    def SetColLabelValue(self, col_num, label_value):    
        #self.header_grid.SetColLabelValue(col_num, label_value)
        self.device_grid.SetColLabelValue(col_num, label_value)

    def genPacketOrderedList(self, tsd):
        """ Return list of (dev_name, device, port_num, port) tuples in that packet would go through devices"""
        return genPortOrder(tsd.getDevices())
        
    def genPositionOrderedList(self, tsd):
        """ Return list of (dev_name, device, port_num, port) tuples ordered by device position """
        # Order list of devices by ring_position
        ordered_devices = []
        for device in tsd.getDevices():
            ordered_devices.append( (device.ring_position, device) )
        ordered_devices.sort()

        result = []
        for position,device in ordered_devices:
            for num,port in enumerate(device.ports):                
                result.append( (device, num, port) )

        return result

        
    def updateDeviceGrid(self, tsd, mode):
        """ Update grid with timestamp data """

        if mode == "position_order":
            port_list = self.genPositionOrderedList(tsd)
        else:
            port_list = self.genPacketOrderedList(tsd)

        # Generate list of lists with device data. 
        data = []
        ERROR = CellData.ERROR
        WARN = CellData.WARN
        DATA = CellData.DATA


        last_dev_name = None
        for device,num,port in port_list:

            row_data = [ cell_data_empty for unused in range(self.num_rows) ]            
            row_data[4] = CellData('Port%d'%num)
            row_data[5] = CellData(port.lost_links, ERROR if (port.lost_links != 0) else DATA)
            row_data[6] = CellData(port.frame_errors, ERROR if (port.frame_errors != 0) else DATA)
            if port.est_drops is None:                
                row_data[7] = CellData("?", WARN)
            else:
                row_data[7] = CellData(port.est_drops, ERROR if (port.est_drops != 0) else DATA)
            row_data[9] = CellData(port.rx_errors, ERROR if (port.rx_errors != 0) else DATA)
            row_data[10] = CellData(port.forwarded_rx_errors, WARN if (port.forwarded_rx_errors != 0) else DATA)
            if port.open is None:
                row_data[13] = CellData("Changed", ERROR)
            else:
                row_data[13] = CellData(port.open, ERROR if ((num==0) and (not port.open)) else DATA)
            if (last_dev_name is None) or (device.name != last_dev_name):
                row_data[0]  = CellData(device.name, DATA if (device.valid) else ERROR)
                row_data[1]  = CellData(device.hardware_id, DATA if (device.valid) else ERROR)
                row_data[2]  = CellData(device.ring_position, ERROR if (device.ring_position is None) else DATA)
                row_data[3]  = CellData(device.valid, DATA if (device.valid) else ERROR)            
                row_data[11]  = CellData(device.epu_errors, ERROR if (device.epu_errors != 0) else DATA)
                row_data[12] = CellData(device.pdi_errors, ERROR if (device.pdi_errors != 0) else DATA)
            data.append(row_data)
            last_dev_name = device.name

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

        grid.AutoSizeColumns()
