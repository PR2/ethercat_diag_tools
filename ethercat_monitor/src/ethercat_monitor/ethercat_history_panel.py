import os
import wx
import time
import rospy

from ethercat_monitor.wx_util import displayErrorDialog
from ethercat_monitor.util import prettyTimestamp, prettyDuration


class EtherCATHistoryPanel(wx.Panel):
    def __init__(self, parent, history):
        wx.Panel.__init__(self, parent, -1, name=history.getTitle())
        # Provides way to view or select items from history
        self.tsd_list = history.getAllTimestepData()

        self.tsd_grid = wx.grid.Grid(self)
        self.tsd_grid.CreateGrid(len(self.tsd_list), 5)
        self.tsd_grid.SetColLabelValue(0, "Date")
        self.tsd_grid.SetColLabelValue(1, "Time")
        self.tsd_grid.SetColLabelValue(2, "Dropped")
        self.tsd_grid.SetColLabelValue(3, "Frame\nError")
        self.tsd_grid.SetColLabelValue(4, "Lost\nLinks")

        self.tsd_grid.SetRowLabelSize(0) # hide the row labels
        self.tsd_grid.SetColLabelSize(self.tsd_grid.GetDefaultColLabelSize() * 2)

        #current_time = rospy.Time.from_sec(time.time())
        for row,tsd in enumerate(self.tsd_list):
            for col in range(4):
                self.tsd_grid.SetReadOnly( row, col );

            localtime = time.localtime(tsd.getTimestamp().to_sec())
            time_str = time.strftime("%I:%M %p", localtime)
            date_str = time.strftime("%a, %b %d %Y", localtime)
            
            frame_errors = 0
            lost_links = 0
            for dev in tsd.getDevices():
                for port in dev.ports:
                    frame_errors += port.frame_errors
                    frame_errors += port.lost_links

            self.tsd_grid.SetCellValue(row,0, date_str)
            self.tsd_grid.SetCellValue(row,1, time_str)
            self.tsd_grid.SetCellValue(row,2, str(tsd.getMaster().dropped))
            self.tsd_grid.SetCellValue(row,3, str(frame_errors))
            self.tsd_grid.SetCellValue(row,4, str(lost_links))

        self.tsd_grid.AutoSize()
        #self.tsd_grid.AutoSizeRowLabelSize(2)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.tsd_grid,1,wx.EXPAND)

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.SetMinSize((800,600))
        vsizer.Fit(self)


class EtherCATHistoryDialog(wx.Dialog):
    def __init__(self, parent, history):
        wx.Dialog.__init__(self,parent,-1,title='View History',size=(800,600))

        panel = EtherCATHistoryPanel(self, history)

        self.Centre()
        self.Layout()
        self.Show(True) 
