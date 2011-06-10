import os
import wx
import time
import rospy

from ethercat_monitor.wx_util import displayErrorDialog
from ethercat_monitor.util import prettyTimestamp, prettyDuration
from ethercat_monitor.timestep_data import EtherCATHistoryTimestepDataNote
from ethercat_monitor.wx_util import displayErrorDialog
from ethercat_monitor.note_edit_dialog import NoteEditDialog

class EtherCATHistoryPanel(wx.Panel):
    def __init__(self, parent, history):
        wx.Panel.__init__(self, parent, -1, name=history.getTitle())

        self.history = history
        self.note_button = wx.Button(self, -1, "Add Note")
        self.Bind(wx.EVT_BUTTON, self.onAddNote, self.note_button)
        self.update_button = wx.Button(self, -1, "Update")
        self.Bind(wx.EVT_BUTTON, self.onUpdate, self.update_button)

        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.Add(self.note_button,0)
        hsizer.Add(self.update_button,0)

        self.tsd_list = history.getAllTimestepData()

        self.tsd_grid = wx.grid.Grid(self)
        grid = self.tsd_grid
        grid.CreateGrid(len(self.tsd_list), 5)
        self.Bind(wx.grid.EVT_GRID_SELECT_CELL, self.onSelectGridCell, self.tsd_grid)
        grid.SetColLabelValue(0, "Date")
        grid.SetColLabelValue(1, "Time")
        grid.SetColLabelValue(2, "Dropped")
        grid.SetColLabelValue(3, "Frame\nError")
        grid.SetColLabelValue(4, "Lost\nLinks")

        grid.SetRowLabelSize(0) # hide the row labels
        grid.SetColLabelSize(grid.GetDefaultColLabelSize() * 2)

        self.updateGrid()
        
        self.tsd_grid.AutoSize()
        #self.tsd_grid.AutoSizeRowLabelSize(2)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(hsizer,0)
        vsizer.Add(self.tsd_grid,1,wx.EXPAND)

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.SetMinSize((800,600))
        vsizer.Fit(self)

        self.current_selection = -1

    def onSelectGridCell(self, event):
        self.current_selection = event.GetRow()
        print "current selection = ", self.current_selection
        event.Skip()

    def onAddNote(self, event):
        self.current_selection
        if self.current_selection > 0:
            tsd = self.tsd_list[self.current_selection]
            note = EtherCATHistoryTimestepDataNote(tsd, "Type note here")
            dlg = NoteEditDialog(self, note)
            dlg.ShowModal()
            dlg.Destroy()
            self.history.addNote(note)
        else:
            displayErrorDialog(self, "Please select a cell from a given row")

    def updateGrid(self):
        # Provides way to view or select items from history
        self.tsd_list = self.history.getAllTimestepData()

        # Re-adjust grid to have appropriate number of rows
        grid = self.tsd_grid
        diff = len(self.tsd_list) - grid.GetNumberRows()
        if diff > 0:
            grid.AppendRows(diff)
        elif diff < 0:
            grid.DeleteRows(0,diff)

        #current_time = rospy.Time.from_sec(time.time())
        for row,tsd in enumerate(self.tsd_list):
            #grid.SetRowLabelValue(row,str(row))
            for col in range(4):
                grid.SetReadOnly( row, col );

            localtime = time.localtime(tsd.getTimestamp().to_sec())
            time_str = time.strftime("%I:%M %p", localtime)
            date_str = time.strftime("%a, %b %d %Y", localtime)
            
            frame_errors = 0
            lost_links = 0
            for dev in tsd.getDevices():
                for port in dev.ports:
                    frame_errors += port.frame_errors
                    frame_errors += port.lost_links

            grid.SetCellValue(row,0, date_str)
            grid.SetCellValue(row,1, time_str)
            grid.SetCellValue(row,2, str(tsd.getMaster().dropped))
            grid.SetCellValue(row,3, str(frame_errors))
            grid.SetCellValue(row,4, str(lost_links))

        grid.AutoSizeColumns()


    def onUpdate(self, event):
        self.updateGrid()


class EtherCATHistoryDialog(wx.Dialog):
    def __init__(self, parent, history):
        wx.Dialog.__init__(self,parent,-1,title='View History',size=(800,600))

        panel = EtherCATHistoryPanel(self, history)

        self.Centre()
        self.Layout()
        self.Show(True) 
