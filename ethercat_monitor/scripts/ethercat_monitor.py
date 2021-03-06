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
import traceback

import wx
import wx.grid

import yaml

from ethercat_monitor.timestep_data import EtherCATHistoryTimestepDataNote
from ethercat_monitor.ethercat_history import EtherCATSubscriber, EtherCATBagReader
from ethercat_monitor.ethercat_monitor_panel import EtherCATMonitorReaderPanel

from ethercat_monitor.wx_util import displayErrorDialog

def usage(progname):
    print __doc__ % vars()    

# TODO :
#  Better way of displaying old/new timestamp data in history
#  Highlighting color for status (Error Warn)
#  Highlighting color for tab base on drops or lost links
#  Unit tests
#  Way to create differential plot of dropped/late packets
#  Have bag processor die when ethercat monitor quits
#  Auto-generate note when link loss or multiple drops occurs within certain period 

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
        vsizer.Add(hzizer1, 0, wx.EXPAND)
        vsizer.Add(hzizer2, 0, wx.EXPAND)


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
    def __init__(self, readers):
        wx.Frame.__init__(self, None, -1, "EtherCAT Monitor")

        self.notebook = wx.Notebook(self)
        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.notebook,1,wx.EXPAND)

        self.panels = []

        for reader in readers:
            self.addPanel(reader)

        # Setting up the menu.
        filemenu= wx.Menu()
        change_topic_menu_item = wx.MenuItem(filemenu, -1, text='New &Topic', help='Subscribe to new diagnostic topic')
        self.Bind(wx.EVT_MENU, self.onNewTopic)
        filemenu.AppendItem(change_topic_menu_item)

        save_bag_menu_item = wx.MenuItem(filemenu, -1, text="&Save Bag File", help="Save EtherCAT data in bag file")
        self.Bind(wx.EVT_MENU, self.onSaveBag, save_bag_menu_item)
        filemenu.AppendItem(save_bag_menu_item)

        save_bag_default_menu_item = wx.MenuItem(filemenu, -1, text="&Save Bag Default", help="Save data to default location")
        self.Bind(wx.EVT_MENU, self.onSaveBagDefault, save_bag_default_menu_item)
        filemenu.AppendItem(save_bag_default_menu_item)

        open_bag_menu_item = wx.MenuItem(filemenu, -1, text="&Open Bag File", help="Load data from bag file")
        self.Bind(wx.EVT_MENU, self.onLoadBag, open_bag_menu_item)
        filemenu.AppendItem(open_bag_menu_item)

        filemenu.AppendSeparator()
        
        exit_menu_item = wx.MenuItem(filemenu, -1, text="E&xit", help="Terminate this program")
        self.Bind(wx.EVT_MENU, self.onQuit, exit_menu_item)
        # Creating the menubar.
        menuBar = wx.MenuBar()
        menuBar.Append(filemenu,"&File") 
        self.SetMenuBar(menuBar)  
 
        # setup timer to update screen periodically
        self.timer = wx.Timer(self,-1)
        self.Bind(wx.EVT_TIMER, self.onTimer, self.timer)
        self.timer.Start(500)

        vsizer.SetMinSize((1200,600))

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.Fit(self)

        self.Show(True)


    def addPanel(self, reader):
        panel = EtherCATMonitorReaderPanel(self.notebook, reader)
        self.panels.append(panel)        
        self.notebook.AddPage(panel, reader.getTitle())
        self.notebook.ChangeSelection(len(self.panels)-1)

    def getCurrentPanel(self):
        index = self.notebook.GetSelection()
        if index >= 0:
            return self.panels[index]
        else:
            return None
            
    def update(self):
        panel = self.getCurrentPanel()
        if panel is not None:
            panel.update()
        self.Layout()

    def onNewTopic(self, event):
        dlg = TopicSelectDialog(None)
        dlg.ShowModal()
        if dlg.selected_topic is not None:
            topic_name = str(dlg.selected_topic)
            for panel in self.panels:
                if panel.getReader().getTopic() == topic_name:
                    errorDialog("Topic '%s' alread has a connection")
                    return
            reader = EtherCATSubscriber(topic_name)
            self.addPanel(reader)
        else:
            print "Cancel"
        dlg.Destroy()
        self.update()

    def onSaveBag(self, event):
        panel = self.getCurrentPanel()
        if panel is None:
            displayErrorDialog(self, "No tab is selected")
        else:
            try :
                panel.saveBag()
            except Exception, e:                
                displayErrorDialog(self, "Error occurred while saving bag : " + str(e))
                traceback.print_exc()
 
    def onSaveBagDefault(self, event):    
        panel = self.getCurrentPanel()
        if panel is None:
            displayErrorDialog(self, "No tab is selected")
        else:
            try : 
                panel.saveBagDefault()
            except Exception, e:
                displayErrorDialog(self, "Error occurred while saving bag : " + str(e))
                traceback.print_exc()                

    def onLoadBag(self, event):
        dlg = wx.FileDialog(self, "Select bag file to load", style= (wx.FD_OPEN | wx.FD_MULTIPLE) )
        if dlg.ShowModal() == wx.ID_OK:        
            bag_filenames = dlg.GetPaths()
            for bag_filename in bag_filenames:
                if not os.path.isfile(bag_filename): 
                    displayErrorDialog(self, "'%s' is not a file" % bag_filename)
                    return
            for bag_filename in bag_filenames:
                reader = EtherCATBagReader(bag_filename)
                self.addPanel(reader)
        
            
    def onTimer(self, event):
        self.update()

    def onQuit(self, event):
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

    rospy.init_node('ethercat_monitor', anonymous=True)
    
    readers = []
    if len(argv) >= 1:
        for bag_filename in argv:
            readers.append(EtherCATBagReader(bag_filename))
    else:
        readers.append(EtherCATSubscriber('/diagnostics'))

    app = wx.PySimpleApp()
    MainWindow(readers)
    app.MainLoop()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
