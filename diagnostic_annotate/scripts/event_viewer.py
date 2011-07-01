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
Usage: %(progname)s [-h] <event_file>
  Display events in  <event_file> 

Options:
  -h : show this help
"""

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import rospy

from diagnostic_annotate.diag_event import DiagEvent

import wx
import wx.grid
import yaml
import sys
import getopt
import time
import math

def displayErrorDialog(parent, message):
    dlg = wx.MessageDialog(parent, message, caption="Error", style=(wx.OK | wx.CENTRE | wx.ICON_ERROR))
    dlg.ShowModal()
    dlg.Destroy()  


def pretty_duration(duration):    
    is_negative = False
    is_now = False 
    if math.fabs(duration) < 60.0:
        return "%.2f secs " % duration
        
    if (duration < 0):
        duration = -duration
        is_negative = True

    secs_per_min  = 60.0
    secs_per_hour = secs_per_min * 60.0
    secs_per_day  = secs_per_hour * 24.0

    days = math.floor(duration / secs_per_day)
    duration -= days * secs_per_day
    hours = math.floor(duration / secs_per_hour)
    duration -= hours * secs_per_hour
    mins = math.floor(duration / secs_per_min)
    duration -= mins * secs_per_min
    result = ""
    if days > 0:    
        result += ("%d d "%(days))
    if hours > 0:
        result += ("%d h "%(hours))
    if mins > 0:
        result += ("%d m"%(mins))
    if len(result) > 0:
        result += " "
    result += "%.2f s " % duration

    if is_negative:
        result =  "- " + result
    else:
        result = "+ " + result
    return result



class EventViewerFrame(wx.Frame):
    def __init__(self, events):
        wx.Frame.__init__(self, None, -1, "Event Viewer")

        self.events = events
        self.current_selection = -1

        # Setting up the menu.
        filemenu= wx.Menu()
        filemenu.AppendSeparator()
        
        exit_menu_item = wx.MenuItem(filemenu, -1, text="E&xit", help="Terminate this program")
        self.Bind(wx.EVT_MENU, self.onQuit, exit_menu_item)

        # Creating the menubar.
        menuBar = wx.MenuBar()
        menuBar.Append(filemenu,"&File") 
        self.SetMenuBar(menuBar)  

        self.set_ref_button = wx.Button(self, -1, "Set Ref Time")
        self.Bind(wx.EVT_BUTTON, self.onSetReferenceTime, self.set_ref_button)
        self.view_children_button = wx.Button(self, -1, "View Children")
        self.Bind(wx.EVT_BUTTON, self.onViewChildren, self.view_children_button)

        # button bar
        hsizer = wx.BoxSizer(wx.HORIZONTAL)       
        hsizer.Add(self.set_ref_button, 0, wx.EXPAND)
        hsizer.Add(self.view_children_button, 0, wx.EXPAND)

        grid = wx.grid.Grid(self)
        self.grid = grid
        self.Bind(wx.grid.EVT_GRID_SELECT_CELL, self.onSelectGridCell, self.grid)

        num_cols = 8
        grid.CreateGrid(0,num_cols)

        grid.SetColLabelValue(0, "Relative\nTime")
        grid.SetColLabelValue(1, "Date")
        grid.SetColLabelValue(2, "Time")
        grid.SetColLabelValue(3, "Event\nType")
        grid.SetColLabelValue(4, "Component\nName");
        grid.SetColLabelValue(5, "Total\nChildren")
        grid.SetColLabelValue(6, "Description")
        grid.SetColLabelValue(7, "Data")

        #header_grid.SetRowLabelSize(0) # hide the row labels
        grid.SetRowLabelSize(0) # hide the row labels

        grid.SetColLabelSize(grid.GetDefaultColLabelSize() * 2)
        grid.AutoSize()

        grid.EnableDragGridSize(False)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(hsizer, 0, wx.EXPAND)
        vsizer.Add(self.grid, 1, wx.EXPAND)

        vsizer.SetMinSize((1200,600))

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        #self.SetupScrolling()

        grid.AppendRows(len(events))
        for row,event in enumerate(events):
            for col in range(num_cols):
                grid.SetReadOnly( row, col );

            localtime = time.localtime(event.t.to_sec())
            time_str = time.strftime("%I:%M.%S %p", localtime)
            date_str = time.strftime("%a, %b %d %Y", localtime)
            child_count = EventViewerFrame.sumChildren(event)

            grid.SetCellValue(row,1,date_str)
            grid.SetCellValue(row,1,date_str)
            grid.SetCellValue(row,2,time_str)
            grid.SetCellValue(row,3,event.type)
            grid.SetCellValue(row,4,event.name)
            grid.SetCellValue(row,5,str(child_count))
            grid.SetCellValue(row,6,event.desc)
            grid.SetCellValue(row,7,str(event.data))
        
        self.setReferenceTime(events[0])

        grid.AutoSizeColumns()

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.Fit(self)

        self.current_selection = -1

        self.Show(True)

    @staticmethod
    def sumChildren(event):
        sum = 0
        for child in event.children:
            sum += 1 + EventViewerFrame.sumChildren(child)
        return sum

    def onSelectGridCell(self, event):
        self.current_selection = event.GetRow()
        #print "current selection = ", self.current_selection
        event.Skip()

    def onSetReferenceTime(self, event):
        if self.current_selection < 0:
            displayErrorDialog(self, "Please select a cell from a given row")
        else:
            event = self.events[self.current_selection]
            self.setReferenceTime(event)

    def setReferenceTime(self, ref_event):
        grid = self.grid
        reft = ref_event.t
        for row,event in enumerate(self.events):
            duration_str = pretty_duration((event.t-reft).to_sec())
            grid.SetCellValue(row,0,duration_str)
        grid.AutoSizeColumns()

    def onViewChildren(self, event):
        if self.current_selection < 0:
            displayErrorDialog(self, "Please select a cell from a given row")
        else:
            #print "current selection = ", self.current_selection
            event = self.events[self.current_selection]
            if len(event.children) == 0:
                displayErrorDialog(self, "Event has no children")
            else:
                viewer = EventViewerFrame(event.children)
                #viewer.SetSize(wx.Size(600, 600))
                #viewer.Layout()
                #viewer.Show(True)
                viewer.Raise()

    def onQuit(self, event):
        self.Close(True)


def usage(progname):
    print __doc__ % vars()


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


    if len(argv) != 1:
      usage(progname)
      return 1

    input_filename = argv[0]
    fd = open(input_filename)
    y = yaml.load(fd)
    fd.close()    
    yaml_events = y['events']
    events = [ DiagEvent.from_yaml(yaml_event) for yaml_event in yaml_events ]


    app = wx.PySimpleApp()
    EventViewerFrame(events)
    app.MainLoop()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
