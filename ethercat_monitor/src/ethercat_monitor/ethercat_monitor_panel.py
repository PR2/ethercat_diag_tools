import os
import wx
import wx.grid

from ethercat_monitor.note_edit_dialog import NoteEditDialog
from ethercat_monitor.wx_util import levelToBackgroundColor
from ethercat_monitor.util import prettyTimestamp, prettyDuration
from ethercat_monitor.device_panel import DevicePanel
from ethercat_monitor.cell_data import CellData, cell_data_empty
from ethercat_monitor.yaml_dialog import YamlDialog
from ethercat_monitor.wx_util import displayErrorDialog
from ethercat_monitor.timestep_data import EtherCATHistoryTimestepDataNote
from ethercat_history_panel import EtherCATHistoryDialog

_yaml_output_directory = "/u/wgtest/ethercat_monitor_yaml/"

# EtherCAT Device Grid
#                     
#  Device    Port,  Valid,  Lost Links, RX, FwdD RX,  EPU  PDI,  
#                
#  fl_caster 0    0     Port0     0    0        0
#                       Port1     0    0        0 
#
# EtherCAT Master Grid
#  Sent Dropped  Late 


class EtherCATMonitorReaderPanel(wx.Panel):
    def __init__(self, parent, reader):
        wx.Panel.__init__(self, parent, -1, name=reader.getTitle())
        # Name of diagnostics topic
        self.source_text = wx.TextCtrl(self, -1, reader.getSourceDesc() , style = wx.TE_READONLY)
        self.panels = []
        self.reader = reader
        self.history_list = []
        self.notebook = wx.Notebook(self)
        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.source_text,0,wx.EXPAND)
        vsizer.Add(self.notebook,1,wx.EXPAND)
        self.updateHistoryPanels()            
        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.Fit(self)

    def getReader(self):
        return self.reader

    def updateHistoryPanels(self):
        # if there are new history elements available, add a new panel
        for history in self.reader.getHistoryList():
            if history not in self.history_list:
                self.addPanel(history)

    def update(self):
        self.updateHistoryPanels()
        # update currently viewer panel
        panel = self.getCurrentPanel()
        if panel is not None:
            panel.update()

    def saveBag(self):
        panel = getCurrentPanel(self)
        if panel is not None:
            panel.saveBag()
        else:
            displayErrorDialog(self, "No history panel selected")

    def addPanel(self, history):
        panel = EtherCATMonitorHistoryPanel(self.notebook, history)
        self.history_list.append(history)
        self.panels.append(panel)        
        self.notebook.AddPage(panel, history.getTitle())

    def getCurrentPanel(self):
        index = self.notebook.GetSelection()
        if index >= 0:
            return self.panels[index]
        else:
            return None



class EtherCATMonitorHistoryPanel(wx.Panel):
    def __init__(self, parent, history):
        wx.Panel.__init__(self, parent, -1, name=history.getTitle())

        self.history = history
        self.notes = []
        self.tsd_old = None
        self.tsd_new = None
 
        # zero & note button
        self.zero_button = wx.Button(self, -1, "Zero")
        self.Bind(wx.EVT_BUTTON, self.onZero, self.zero_button)
        self.note_button = wx.Button(self, -1, "Note")
        self.Bind(wx.EVT_BUTTON, self.onNote, self.note_button)
        button_hsizer2 = wx.BoxSizer(wx.HORIZONTAL)
        button_hsizer2.Add(self.zero_button,0)
        button_hsizer2.Add(self.note_button,0)

        self.view_history_button = wx.Button(self, -1, "View History")
        self.Bind(wx.EVT_BUTTON, self.onViewHistory, self.view_history_button)

        # combo box allow choice between absolute and relative values
        self.display_combo = wx.ComboBox(self, -1, choices=['Absolute','Relative'], style=wx.CB_READONLY)
        self.display_combo.SetSelection(0)  # set selection to absolute on startup

        # combo box allow choice between different ordering of devices
        self.order_combo = wx.ComboBox(self, -1, choices=['Position Order', 'Packet Order'], style=wx.CB_READONLY)
        self.order_combo.SetSelection(0)  # set selection to absolute on startup

        # Name of diagnostics topic
        self.status_text = wx.TextCtrl(self, -1, "", style = wx.TE_READONLY)

        # Scroll window with device information arranged in grid
        self.device_panel = DevicePanel(self)

        # Scolling text window with list of saved notes
        self.note_listbox = wx.ListBox(self, style=wx.LB_SINGLE)
        self.edit_note_button = wx.Button(self, -1, "Edit Note")
        self.Bind(wx.EVT_BUTTON, self.onEditNote, self.edit_note_button)
        self.set_old_button = wx.Button(self, -1, "Set Old")
        self.set_old_button.SetToolTip(wx.ToolTip("Click to use current note selection as old timestep data"))
        self.Bind(wx.EVT_BUTTON, self.onSetOld, self.set_old_button)
        button_hsizer1 = wx.BoxSizer(wx.HORIZONTAL)
        button_hsizer1.Add(self.edit_note_button,0)
        button_hsizer1.Add(self.set_old_button,0)

        # Master grid
        master_grid = wx.grid.Grid(self)
        master_grid.CreateGrid(1,6)
        master_grid.SetColLabelValue(0, "Sent")
        master_grid.SetColLabelValue(1, "Drops")
        master_grid.SetColLabelValue(2, "Late")
        master_grid.SetColLabelValue(3, "Drops per Billion Sends")
        master_grid.SetColLabelValue(4, "Drops per Hour")
        master_grid.SetColLabelValue(5, "Unassigned Drops")
        master_grid.SetRowLabelSize(0) # hide the row labels
        master_grid.AutoSize() 
        self.master_grid = master_grid

        # control panel
        vsizer0 = wx.BoxSizer(wx.VERTICAL)
        vsizer0.Add(self.status_text,0,wx.EXPAND)
        vsizer0.Add(self.display_combo, 0)
        vsizer0.Add(self.order_combo, 0)
        vsizer0.Add(self.view_history_button, 0)
        vsizer0.Add(button_hsizer2, 0)
        vsizer0.Add(button_hsizer1, 0)
        vsizer0.Add(self.note_listbox, 1)

        # Grid for displaying timestamps and duration of data
        # Name of diagnostics topic
        timestamp_grid = wx.grid.Grid(self)
        timestamp_grid.CreateGrid(1,3)
        timestamp_grid.SetColLabelValue(0, "New Time")
        timestamp_grid.SetColLabelValue(1, "Old Time")
        timestamp_grid.SetColLabelValue(2, "Duration")
        timestamp_grid.SetRowLabelSize(0) # hide the row labels
        timestamp_grid.AutoSize() 
        self.timestamp_grid = timestamp_grid

        # device panel
        vsizer1 = wx.BoxSizer(wx.VERTICAL)
        vsizer1.Add(self.timestamp_grid, 0, wx.EXPAND)
        vsizer1.Add(self.master_grid, 0, wx.EXPAND)
        vsizer1.Add(self.device_panel, 1, wx.EXPAND)

        # Entire panel
        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.Add(vsizer0, 0, wx.EXPAND)
        hsizer.Add(vsizer1, 4, wx.EXPAND)

        self.SetSizer(hsizer)
        self.SetAutoLayout(1)
        hsizer.SetMinSize((1000,600))
        hsizer.Fit(self)

        self.Show(True)        


    def update(self):
        self.updateNoteList()

        self.status_text.SetValue(self.history.getStatus())

        self.tsd_new = self.history.getNewestTimestepData()
        if self.tsd_new is None:
            return 
        
        if self.tsd_old is None:
            self.tsd_old = self.tsd_new

        display_item = self.display_combo.GetSelection()
        if display_item == 0:
            tsd = self.tsd_new
        else:
            tsd = self.tsd_new.getDiff(self.tsd_old)

        order_item = self.order_combo.GetSelection()
        if order_item == 0:
            device_grid_mode = "position_order"
        else:
            device_grid_mode = "packet_order"

        self.device_panel.updateDeviceGrid(tsd, device_grid_mode)
        self.updateMasterGrid(tsd)
        self.updateTimestampGrid(tsd)
        self.Layout()

    
    def updateNoteList(self):
        notes = self.history.getNotes()
        # todo, don't update noteslist box if notes list has not changed
        if True:
            # keep track of selections so they can be re-selected after updating listbox
            selections = self.note_listbox.GetSelections()
            self.notes = notes
            note_msgs = []
            for note in notes:
                time_str = prettyTimestamp(note.timestep_data.getTimestamp())
                note_msgs.append("%s : %s" % (time_str, note.note_msg))
            self.note_listbox.SetItems(note_msgs)
            for index in selections:
                if index < len(notes):
                    self.note_listbox.Select(index)

    def onViewHistory(self, event):
        dlg = EtherCATHistoryDialog(self, self.history)
        dlg.ShowModal()
        dlg.Destroy()

    def getSelectedNote(self):
        index = self.note_listbox.GetSelection()
        if index < 0:
            displayError(self, "Error", "No note selected to edit")
            return None
        return self.notes[index]        

    def onEditNote(self, event):
        note = self.getSelectedNote()
        if note is None:
            return
        dlg = NoteEditDialog(self, note)
        dlg.ShowModal()
        dlg.Destroy()

    def onSetOld(self, event):
        note = self.getSelectedNote()
        if note is None:
            return
        self.tsd_old = note.timestep_data
        self.selectRelativeView()        

    def selectRelativeView(self):
        self.display_combo.SetSelection(1)  

    def updateMasterGrid(self, tsd):
        ERROR = CellData.ERROR
        WARN = CellData.WARN
        DATA = CellData.DATA
        empty = cell_data_empty #CellData()
        master = tsd.getMaster()
        data = [empty for i in range(6)]
        sent = master.sent
        dropped = master.dropped
        late = master.late

        data[0] = CellData(sent)
        data[1] = CellData(dropped, ERROR if (dropped > 0) else DATA)
        data[2] = CellData(late   , WARN  if (late    > 0) else DATA)
        if sent != 0:
            drops_per_billion_sends = 1e9 * float(dropped) / float(sent)
            if drops_per_billion_sends > 3000 : level = ERROR
            elif drops_per_billion_sends > 1000: level = WARN
            else: level = DATA
            data[3] = CellData("%.2f"%drops_per_billion_sends, level)
            if tsd.timestamp_old is not None:
                secs_per_hour = 3600.
                hours = (tsd.getTimestamp() - tsd.timestamp_old).to_sec() / secs_per_hour
                drops_per_hour = dropped / hours
                if (drops_per_hour > 10): level = ERROR
                elif (drops_per_hour > 1): level = WARN
                else : level = DATA
                data[4] = CellData("%.2f"%drops_per_hour, level)
            unassigned_drops = master.unassigned_drops
            if unassigned_drops is None:
                data[5] = CellData("?", WARN)
            else:
                data[5] = CellData(unassigned_drops, ERROR if (unassigned_drops > 0) else DATA)
        grid = self.master_grid
        row = 0
        for col,cell_data in enumerate(data):
            grid.SetCellValue(row,col,str(cell_data))
            bg_color = levelToBackgroundColor(cell_data.level)
            grid.SetCellBackgroundColour(row,col,bg_color)
            grid.SetReadOnly( 0, col );
        grid.AutoSize()


    def saveBag(self):
        dlg = wx.FileDialog(self, "Select bag file to open", style=wx.FD_OPEN)
        if dlg.ShowModal() == wx.ID_OK:        
            bag_filename = os.path.join(dlg.GetDirectory(), dlg.GetFilename())
            self.history.saveBag(bag_filename)
            print "Saved bag file to ", bag_filename

    def updateTimestampGrid(self, tsd):
        grid = self.timestamp_grid
        grid.SetCellValue(0,0,prettyTimestamp(tsd.getTimestamp()))
        if tsd.timestamp_old is not None:
            grid.SetCellValue(0,1,prettyTimestamp(tsd.timestamp_old))
            duration = tsd.getTimestamp() - tsd.timestamp_old
            grid.SetCellValue(0,2,prettyDuration(duration))
        else:
            grid.SetCellValue(0,1,"")
            grid.SetCellValue(0,2,"")
        grid.AutoSize()

    def onZero(self, event):
        # set selection to relative when zero is pushed
        self.selectRelativeView()
        tsd = self.history.getNewestTimestepData()
        if tsd is not None:
            self.tsd_old = tsd
            note = EtherCATHistoryTimestepDataNote(tsd, "Zero pressed")
            self.history.addNote(note)            

    def onNote(self, event):
        # set selection to relative when note is created
        self.selectRelativeView()
        tsd = self.history.getNewestTimestepData()
        if tsd is not None:
            self.tsd_old = tsd
            note = EtherCATHistoryTimestepDataNote(tsd, "Type note here")
            dlg = NoteEditDialog(self, note)
            dlg.ShowModal()
            dlg.Destroy()
            self.history.addNote(note)            


