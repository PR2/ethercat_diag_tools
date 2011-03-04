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
##\brief Provide GUI to view network related EtherCAT errors.

"""
Usage: %(progname)s [-h] <bagfile>
  Parses diagnostics in <bagfile> and provides GUI to view 
  network related EtherCAT erros.

Options:
  -h : show this help
"""


PKG = 'ethercat_monitor'
import roslib
roslib.load_manifest(PKG)

import rospy
import time
import math
import sys 
import os
import unittest
import getopt
import itertools
import os.path

import wx
import wx.grid

import yaml

from ethercat_monitor.ethercat_history import EtherCATHistory
from ethercat_monitor.wx_util import levelToBackgroundColor
from ethercat_monitor.util import prettyTimestamp, prettyDuration
from ethercat_monitor.device_panel import DevicePanel

from ethercat_monitor.cell_data import CellData, cell_data_empty

def usage(progname):
    print __doc__ % vars()    


# TODO :
#  Widget to select between absolute a interval error counts
#  Dialog box to select different diagnostics topic
#  Put box for duration since zeroing/starting
#  Keep track of port opened/closed status
#  Keep track of diagnostics valid state
#  Unit tests


# TODO (later):
#  Button to take timestamps
#  List of timestamps
#  Way to prune history,
#  Faster way to search history

# EtherCAT Device Grid
#                     
#  Device    Port,  Valid,  Lost Links, RX, FwdD RX,  EPU  PDI,  
#                
#  fl_caster 0    0     Port0     0    0        0
#                       Port1     0    0        0 
#
# EtherCAT Master Grid
#  Sent Dropped  Late 


class TimestampSelect(wx.Panel):
    def __init__(self,parent,name):
        wxPanel.__init__(parent,name=name)
        
        hsizer1 = wx.BoxSizer(wx.HORIZONTAL)
        self.time_pretty_text_box = wx.TextCtrl(self)
        hsizer1.Add(wx.StaticText(self, -1, "Input:"))
        hsizer1.Add(self.time_pretty_text_box, wx.EXPAND)

        hsizer2 = wx.BoxSizer(wx.HORIZONTAL)
        self.time_raw_text_box = wx.TextCtrl(self)
        hsizer2.Add(wx.StaticText(self, -1, "Raw:"))
        hsizer2.Add(self.time_raw_test_box, wx.EXPAND)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(hzizer1, wx.EXPAND)
        vsizer.Add(hzizer2, wx.EXPAND)


    def update(self,time):
        self.raw_raw_text_box.SetValue(str(time))
        self.raw_pretty_text_box.SetValue(str(time))


def getDiagnosticTopics():
    """ Returns list of ROS topic names with type 'diagnostic_msgs/DiagnosticArray' """
    print 'Getting list of topics'
    master = roslib.scriptutil.get_master()
    code, msg, pub_topics = master.getPublishedTopics('/rostopic', '/')
    if code != 1:
        print "Error getting topic list : %s" % msg
        return []
    diag_topics = []
    for topic,topic_type in pub_topics:
        if topic_type == 'diagnostic_msgs/DiagnosticArray':
            diag_topics.append(topic)
    return diag_topics


class TopicSelectDialog(wx.Dialog):
    def __init__(self, parent):
        wx.Dialog.__init__(self,parent,-1,title='Select Diagnostics Topic')

        #self.display_combo = wx.ComboBox(self, -1, choices=['Absolute','Relative'], style=wx.CB_READONLY)
        self.combo = wx.ComboBox(self, -1, choices=[], style=wx.CB_READONLY)
        self.UpdateTopicComboList(None)
        self.selected_topic = None
        self.Bind(wx.EVT_COMBOBOX, self.OnSelectTopic, self.combo)
        self.Bind(wx.EVT_SET_FOCUS, self.UpdateTopicComboList, self.combo)

        self.cancel_button = wx.Button(self, -1, "Cancel")
        self.Bind(wx.EVT_BUTTON, self.OnCancel, self.cancel_button)        

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.combo)
        vsizer.Add(self.cancel_button)
        
        self.SetSizer(vsizer)
        self.Centre()
        self.Show(True) 

    def OnCancel(self, event):
        self.Close()

    def UpdateTopicComboList(self, event):
        print "Updating combo list"
        self.topics = getDiagnosticTopics() 
        if len(self.topics) == 0: 
            self.topics = ['/diagnostics-foo']
        self.combo.Clear()
        #for topic in self.topics:
        self.combo.AppendItems(self.topics)

    def OnSelectTopic(self, event):
        self.selected_topic = self.combo.GetValue()
        self.Close()




class MainWindow(wx.Frame):
    def __init__(self, parent, id, title, history, topic_name):
        wx.Frame.__init__(self, parent, id, title)   

        self.history = history
        self.tsd_old = None
        self.tsd_new = None

        #for s in subassem_list:
        ID_TOPIC=100
        ID_EXIT=101

        # Setting up the menu.
        filemenu= wx.Menu()
        filemenu.Append(ID_TOPIC, "Change &Topic"," Change diagnositic topic")
        filemenu.AppendSeparator()
        filemenu.Append(ID_EXIT,"E&xit"," Terminate the program")
        # Creating the menubar.
        menuBar = wx.MenuBar()
        menuBar.Append(filemenu,"&File") 
        self.SetMenuBar(menuBar)  
                
        wx.EVT_MENU(self, ID_TOPIC, self.OnChangeTopic)
        wx.EVT_MENU(self, ID_EXIT, self.OnQuit)
 
        DEVICE_TABLE_ID   = 10
        ZERO_BUTTON_ID    = 11

        # zero button
        self.zero_button = wx.Button(self, ZERO_BUTTON_ID, "Zero")
        self.Bind(wx.EVT_BUTTON, self.OnZero, self.zero_button)
        #wx.EVT_BUTTON(self, ZERO_BUTTON_ID, self.OnZero)

        # zero button
        self.yaml_button = wx.Button(self, ZERO_BUTTON_ID, "Generate Yaml")
        self.Bind(wx.EVT_BUTTON, self.OnGenerateYaml, self.yaml_button)

        # combo box allow choice between absolute and relative values
        self.display_combo = wx.ComboBox(self, -1, choices=['Absolute','Relative'], style=wx.CB_READONLY)
        self.display_combo.SetSelection(0)  # set selection to absolute on startup

        # Name of diagnostics topic
        self.topic_text = wx.TextCtrl(self, -1, "", style = wx.TE_READONLY)

        # topic button
        self.topic_button = wx.Button(self, -1, "Select Topic")
        self.Bind(wx.EVT_BUTTON, self.OnChangeTopic, self.topic_button)

        # Scroll window with device information arranged in grid
        self.device_panel = DevicePanel(self)

        # Master grid
        master_grid = wx.grid.Grid(self)
        master_grid.CreateGrid(1,5)
        master_grid.SetColLabelValue(0, "Sent")
        master_grid.SetColLabelValue(1, "Drops")
        master_grid.SetColLabelValue(2, "Late")
        master_grid.SetColLabelValue(3, "Drops per Billion Sends")
        master_grid.SetColLabelValue(4, "Drops per Hour")
        master_grid.SetRowLabelSize(0) # hide the row labels
        master_grid.AutoSize() 
        self.master_grid = master_grid

        # control panel
        vsizer0 = wx.BoxSizer(wx.VERTICAL)
        vsizer0.Add(self.zero_button, 0)
        vsizer0.Add(self.display_combo, 0)
        vsizer0.Add(self.topic_text,0,wx.EXPAND)
        vsizer0.Add(self.topic_button, 0)
        vsizer0.Add(self.yaml_button, 0)
        

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

        # Entire app
        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.Add(vsizer0, 0, wx.EXPAND)
        hsizer.Add(vsizer1, 4, wx.EXPAND)

        self.SetSizer(hsizer)
        self.SetAutoLayout(1)
        hsizer.SetMinSize((1000,600))
        hsizer.Fit(self)


        # setup timer to update screen periodically
        self.timer = wx.Timer(self,-1)
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)
        self.timer.Start(500)

        # use difference between begining and end for timestamp data
        #tsd_end = history.getTimestepData(history.END)
        #tsd_begin = history.getTimestepData(history.BEGIN)
        #tsd = tsd_end.getDiff(tsd_begin)
        #self.updateDeviceGrid(tsd)
        self.current_topic = topic_name
        if topic_name is not None:
            self.changeTopic(topic_name)

        self.Show(True)        

    def update(self):
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

        self.device_panel.updateDeviceGrid(tsd)
        self.updateMasterGrid(tsd)
        self.updateTimestampGrid(tsd)


    def changeTopic(self, topic_name):
        self.history.subscribeToDiagnostics(topic_name)
        self.current_topic = topic_name
        self.topic_text.SetValue(self.current_topic)
        self.tsd_new = None
        self.tsd_old = None
        self.update()

    def updateMasterGrid(self, tsd):
        ERROR = CellData.ERROR
        WARN = CellData.WARN
        DATA = CellData.DATA
        empty = cell_data_empty #CellData()
        master = tsd.master
        data = [empty for i in range(5)]
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
                hours = (tsd.timestamp - tsd.timestamp_old).to_sec() / secs_per_hour
                drops_per_hour = dropped / hours
                if (drops_per_hour > 10): level = ERROR
                elif (drops_per_hour > 1): level = WARN
                else : level = DATA
                data[4] = CellData("%.2f"%drops_per_hour, level)
        grid = self.master_grid
        row = 0
        for col,cell_data in enumerate(data):
            grid.SetCellValue(row,col,str(cell_data))
            bg_color = levelToBackgroundColor(cell_data.level)
            grid.SetCellBackgroundColour(row,col,bg_color)
            grid.SetReadOnly( 0, col );
        grid.AutoSize()

    def OnGenerateYaml(self, event):
        self.tsd_new = self.history.getNewestTimestepData()
        if self.tsd_new is None:
            return 
        if self.tsd_old is None:
            self.tsd_old = self.tsd_new

        tsd_new  = self.tsd_new
        tsd_old  = self.tsd_old
        tsd_diff = self.tsd_new.getDiff(self.tsd_old)
        
        out = {}
        out['new'] = tsd_new.generateYaml()
        out['old'] = tsd_old.generateYaml()
        out['diff'] = tsd_diff.generateYaml()


        print yaml.dump(out)
        

    def updateTimestampGrid(self, tsd):
        grid = self.timestamp_grid
        grid.SetCellValue(0,0,prettyTimestamp(tsd.timestamp))
        if tsd.timestamp_old is not None:
            grid.SetCellValue(0,1,prettyTimestamp(tsd.timestamp_old))
            duration = tsd.timestamp - tsd.timestamp_old
            grid.SetCellValue(0,2,prettyDuration(duration))
        else:
            grid.SetCellValue(0,1,"")
            grid.SetCellValue(0,2,"")
        grid.AutoSize()

    def OnTimer(self, event):
        self.update()

    def OnZero(self, event):
        self.display_combo.SetSelection(1)  # set selection to relative when zero is pushed
        self.tsd_old = self.history.getNewestTimestepData()


    def OnChangeTopic(self, event):
        dlg = TopicSelectDialog(None)
        dlg.ShowModal()
        if dlg.selected_topic is not None:
            print "Select topic '%s'" % dlg.selected_topic
            if self.current_topic == dlg.selected_topic:        
                print "New topic name is same as previous topic"
            else:
                self.changeTopic(dlg.selected_topic)
        else:
            print "Cancel"
        dlg.Destroy()


    def OnQuit(self, event):
        self.Close(True)



def main(argv):
    progname = argv[0]
    optlist, argv = getopt.getopt(argv[1:], "ht", ["help", "test"])
    for (opt, val) in optlist:
        if opt == "--help" or opt == '-h':
            usage(progname)
            return 0
        elif opt == "--test" or opt == '-t':
            return 0
        else:
            print "Internal error : unhandled option '%s'"%opt
            return 1
    
    if len(argv) == 1:
        inbag_filename = argv[0]
        history.processBag(inbag_filename, diag_list, diag_map)
    else:
        history = EtherCATHistory()
        rospy.init_node('ethercat_monitor', anonymous=True)

    try:
        pass
    except KeyboardInterrupt:
        print "Keyboard Interrupt, quiting"
    except Exception,e:
        print "Error annotating bag %s : %s" % (inbag_filename, str(e))

    print len(history.history)
    history.printHistory()
    #tsd = history.getTimestepData(history.END)
    #print tsd.getDataGrid()

    app = wx.PySimpleApp()
    MainWindow(None, -1, "EtherCAT Monitor", history, '/diagnostics')
    app.MainLoop()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
