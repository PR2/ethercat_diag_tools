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
from diagnostic_annotate.merge_filters import filterPipeline1, filterPassThrough, filterBreakerTrips, filterMultiRunstop, filterOnlyIgnored, filterMtrace, filterEcatCommunication, filterLostLinks, filterMotorModel, filterEcatMerge, filterMotorsHalted, filterRecalibrate, filterCalibration, filterMotorHaltedNoFirst

from diagnostic_annotate.event_tools import sortEvents

import wx
import wx.grid
import yaml
import sys
import getopt
import time
import math
import copy
import traceback

def displayErrorDialog(parent, message):
    dlg = wx.MessageDialog(parent, message, caption="Error", style=(wx.OK | wx.CENTRE | wx.ICON_ERROR))
    dlg.ShowModal()
    dlg.Destroy()  


def pretty_duration(duration):    
    is_negative = False
    is_now = False 
    if math.fabs(duration) < 1.0:
        return "%.4f secs " % duration
    if math.fabs(duration) < 60.0:
        return "%.2f secs " % duration

        
    if (duration < 0.0):
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


class EventGroup:
    def __init__(self, event, indent):
        self.indent = indent
        self.show = True if (indent == 0) else False
        self.event = event
        self.subgroups = []
        for event in event.children:
            self.subgroups.append( EventGroup(event,indent+1) )

    def generateVisibleList(self):
        results = [self]
        for group in self.subgroups:
            if group.show:
                results += group.generateVisibleList()
        return results

    def showChildren(self, show):
        for group in self.subgroups:
            group.show = show

    def generateGridData(self, ref_time):
        event = self.event
        localtime = time.localtime(event.t.to_sec())
        indent_str = ('-' * self.indent) + '+'        
        duration_str = pretty_duration((event.t-ref_time).to_sec())
        time_str = time.strftime("%I:%M.%S %p", localtime)
        date_str = time.strftime("%a, %b %d %Y", localtime)
        child_count = EventViewerFrame.sumChildren(event)        
        row_data = [indent_str, duration_str, date_str, time_str, event.type, event.name, str(child_count), event.desc, str(event.data)]
        return row_data



class BagEvent:
    def __init__(self, input_filename):        
        fd = open(input_filename)
        y = yaml.load(fd)
        fd.close()
        # make event for bagfile header
        bag_yaml = y['bag']
        self.bag_start = rospy.Time(bag_yaml['start'])
        bag_end = rospy.Time(bag_yaml['end'])
        bag_path = bag_yaml['path']
        self.bag_event = DiagEvent('BagFileStart', 'Bag start', self.bag_start, bag_path)
        yaml_events = y['events']
        events = [ DiagEvent.from_yaml(yaml_event) for yaml_event in yaml_events ]
        self._events = events

    def filter(self, filter_pipeline):
        # filtering may change event objects, 
        # filter on deep copy of original events since we want to keep original events unchanged 
        events = copy.deepcopy(self._events)
        self.bag_event.children = filter_pipeline(events)


class EventViewerFrame(wx.Frame):
    def __init__(self, input_filenames):
        wx.Frame.__init__(self, None, -1, "Event Viewer")

        self.ref_time = None

        self.bag_events = []
        for filename in input_filenames:            
            try:
                be = BagEvent(filename)
                self.bag_events.append(be)
            except Exception,e:
                print "Exception processing file %s : %s" % (filename, str(e))

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
        self.autosize_grid_button = wx.Button(self, -1, "AutoSize Grid")
        self.Bind(wx.EVT_BUTTON, self.onAutoSizeGrid, self.autosize_grid_button)


        self.filters={'none':filterPassThrough, 'default':filterPipeline1}
        self.filters['breaker trip'] = filterBreakerTrips
        self.filters['multi-runstop'] = filterMultiRunstop
        self.filters['ecat communication'] = filterEcatCommunication
        self.filters['ecat merge'] = filterEcatMerge
        self.filters['any motor trace'] = filterMtrace
        self.filters['ignored'] = filterOnlyIgnored
        self.filters['lost links'] = filterLostLinks
        self.filters['motor model'] = filterMotorModel
        self.filters['motors halted'] = filterMotorsHalted
        self.filters['motors halted (no first)'] = filterMotorHaltedNoFirst
        self.filters['recalibrate'] = filterRecalibrate
        self.filters['calibration'] = filterCalibration
        self.filter_combobox = wx.ComboBox(self, -1, choices=self.filters.keys(), style=(wx.CB_READONLY | wx.CB_DROPDOWN))
        self.Bind(wx.EVT_COMBOBOX, self.onFilterSelect)

        # button bar
        hsizer = wx.BoxSizer(wx.HORIZONTAL)       
        hsizer.Add(self.set_ref_button, 0, wx.EXPAND)
        hsizer.Add(self.view_children_button, 0, wx.EXPAND)
        hsizer.Add(self.autosize_grid_button, 0, wx.EXPAND)
        hsizer.Add(self.filter_combobox, 0, wx.EXPAND)
        
        grid = wx.grid.Grid(self)
        self.grid = grid
        self.Bind(wx.grid.EVT_GRID_SELECT_CELL, self.onSelectGridCell, self.grid)
        
        num_cols = 9
        grid.CreateGrid(0,num_cols)

        grid.SetColLabelValue(0, "Indent")
        grid.SetColLabelValue(1, "Relative\nTime")
        grid.SetColLabelValue(2, "Date")
        grid.SetColLabelValue(3, "Time")
        grid.SetColLabelValue(4, "Event\nType")
        grid.SetColLabelValue(5, "Component\nName");
        grid.SetColLabelValue(6, "Total\nChildren")
        grid.SetColLabelValue(7, "Description")
        grid.SetColLabelValue(8, "Data")

        #header_grid.SetRowLabelSize(0) # hide the row labels
        grid.SetRowLabelSize(0) # hide the row labels
        grid.SetColLabelSize(grid.GetDefaultColLabelSize() * 2)
        grid.EnableDragGridSize(False)
        
        grid.AutoSizeColumns()

        self.tree = wx.TreeCtrl(self, style=(wx.TR_HIDE_ROOT|wx.TR_HAS_BUTTONS))
        self.Bind(wx.EVT_TREE_SEL_CHANGED, self.onTreeSelectionChanged, self.tree)
        self.Bind(wx.EVT_TREE_ITEM_EXPANDED, self.onTreeItemExpanded, self.tree)
        self.Bind(wx.EVT_TREE_ITEM_COLLAPSED, self.onTreeItemCollapsed, self.tree)

        self.runFilter('none')
        self.generateGridData()
        self.grid.AutoSizeColumns()

        hsizer2 = wx.BoxSizer(wx.HORIZONTAL)
        hsizer2.Add(self.tree, 2, wx.EXPAND)
        hsizer2.Add(self.grid, 5, wx.EXPAND)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(hsizer, 0, wx.EXPAND)
        vsizer.Add(hsizer2, 1, wx.EXPAND)

        vsizer.SetMinSize((1200,600))

        self.SetSizer(vsizer)
        self.SetAutoLayout(1)
        vsizer.Fit(self)
        self.mysizer = vsizer
        self.Show(True)


    @staticmethod
    def sumChildren(event):
        sum = 0
        for child in event.children:
            sum += 1 + EventViewerFrame.sumChildren(child)
        return sum

    def regenerateGridData(self):
        self.generateGridData()
        self.Layout()


    def generateGridData(self):
        self.visible_event_groups = []
        for event_group in self.event_groups:
            self.visible_event_groups += event_group.generateVisibleList()
                    
        row_change = len(self.visible_event_groups) - self.grid.GetNumberRows() 
        if row_change > 0:
            self.grid.AppendRows(row_change)
        elif row_change < 0:
            self.grid.DeleteRows(0, -row_change)

        for row_index,event_group in enumerate(self.visible_event_groups):
            row_data = event_group.generateGridData(self.ref_time)
            for col_index,cell_data in enumerate(row_data):
                self.grid.SetCellValue(row_index,col_index,cell_data)
                #self.grid.SetReadOnly( row_index, col_index );


    def addEventGroupsToTree(self, event_groups, subtree):
        """ Recursively wraps DiagEvent in EventGroup class"""
        for event_group in event_groups:
            event = event_group.event
            desc = event.type + " : " + event.name
            new_subtree = self.tree.AppendItem(subtree, desc)
            self.tree.SetPyData(new_subtree, event_group)
            self.addEventGroupsToTree(event_group.subgroups, new_subtree)


    def runFilter(self, filter_name):
        """ Run new filter on all bag files, then update tree control and grid"""
        try:
            filter_func = self.filters[filter_name]
            print "Filtering", filter_name
            for be in self.bag_events:
                be.filter(filter_func)
        except Exception, e:
            print traceback.format_exc()
            displayErrorDialog(self, "Error with filter (%s) : %s" % (filter_name, str(e)) )

        events = sortEvents([be.bag_event for be in self.bag_events])
        self.events = events
        if len(events) == 0:
            displayErrorDialog(self, "No events")

        self.event_group_selection = None
        if self.ref_time == None:
            self.ref_time = events[0].t if (len(events)>0) else rospy.Time(0)
        self.event_groups = [EventGroup(event,0) for event in events]        
        self.visible_event_groups = None

        self.tree.DeleteAllItems()
        root = self.tree.AddRoot('ROOT')
        self.addEventGroupsToTree(self.event_groups, root)

    def onAutoSizeGrid(self, event):
        self.grid.AutoSizeColumns()

    def onFilterSelect(self, event):
        filter_name = self.filter_combobox.GetValue()
        self.runFilter(filter_name)
        self.regenerateGridData()


    def onSelectGridCell(self, event):
        self.current_event_group_selection = self.visible_event_groups[event.GetRow()]
        event.Skip()

    def onTreeSelectionChanged(self, event):
        item = event.GetItem()
        if item:
            self.current_event_group_selection = self.tree.GetPyData(item)
            index = self.visible_event_groups.index(self.current_event_group_selection)
            self.grid.SelectRow(index)

    def onTreeItemExpanded(self, event):
        item = event.GetItem()
        if item:
            self.tree.GetPyData(item).showChildren(True)
            self.regenerateGridData()

    def onTreeItemCollapsed(self, event):
        item = event.GetItem()
        if item:
            self.tree.GetPyData(item).showChildren(False)
            self.regenerateGridData()

    def onSetReferenceTime(self, event):
        if self.current_event_group_selection is None:
            displayErrorDialog(self, "Please select a cell or tree item")
        else:
            self.ref_time = self.current_event_group_selection.event.t
            self.regenerateGridData()

    def onTreeItemRightClick(self, event):
        print event

    def RightClickCb( self, event ):
        # record what was clicked
        self.list_item_clicked = right_click_context = event.GetText()

        ### 2. Launcher creates wxMenu. ###
        menu = wxMenu()
        for (id,title) in menu_title_by_id.items():
            ### 3. Launcher packs menu with Append. ###
            menu.Append( id, title )
            ### 4. Launcher registers menu handlers with EVT_MENU, on the menu. ###
            EVT_MENU( menu, id, self.MenuSelectionCb )

        ### 5. Launcher displays menu with call to PopupMenu, invoked on the source component, passing event's GetPoint. ###
        self.frame.PopupMenu( menu, event.GetPoint() )
        menu.Destroy() # destroy to avoid mem leak

    def onViewChildren(self, event):
        if self.current_event_group_selection is None:
            displayErrorDialog(self, "Please select a cell from a given row")
        else:
            self.current_event_group_selection.showChildren(True)
            self.regenerateGridData()
            #viewer = EventViewerFrame(self.current_event_selection.children, self.input_filename)
            #viewer.Raise()

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


    if len(argv) < 1:
      usage(progname)
      return 1

    input_filenames = argv

    app = wx.PySimpleApp()
    EventViewerFrame(input_filenames)
    app.MainLoop()

    if False:
        bag_events=[]
        for input_filename in argv:
            try:
                fd = open(input_filename)
                y = yaml.load(fd)
                fd.close()
                # make event for bagfile header
                bag_yaml = y['bag']
                bag_start = rospy.Time(bag_yaml['start'])
                bag_end = rospy.Time(bag_yaml['end'])
                bag_path = bag_yaml['path']
                bag_event = DiagEvent('BagFileStart', 'Bag start', bag_start, bag_path)
                yaml_events = y['events']
                events = [ DiagEvent.from_yaml(yaml_event) for yaml_event in yaml_events ]
                if len(events) == 0:
                    print "No events in ", input_filename
                else:
                    events = filterPipeline1(events)
                    if len(events) == 0:
                        print "No events after filtering from ", input_filename
            except Exception,e:
                print "Exception reading bag file %s : %s" % (input_filename, str(e))

            bag_event.children = events
            bag_events.append(bag_event)

        # make window 
        bag_events = sortEvents(bag_events)


    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
