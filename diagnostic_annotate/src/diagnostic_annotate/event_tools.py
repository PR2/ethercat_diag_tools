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


PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import unittest


def saturate(index, minval, maxval):
    if index < minval:
        return minval
    if index > maxval:
        return maxval
    return index


def getIndexBefore(t, events, hint):
    """ 
    Return index of event that occurs at or just before time <t> 
    Events is assumed to be sorted.  
    Hint is meant to be an index at where to start searching
    """
    index = hint
    index = saturate(index, 0, len(events)-1)
    while (index<len(events)) and (events[index].t <= t) :
        index += 1
    index = saturate(index, 0, len(events)-1)
    while (index>=0) and (events[index].t > t) :
        index -= 1
    index = saturate(index, 0, len(events)-1)
    return index


def getIndexAfter(t, events, hint):
    """ 
    Return index of event that occurs at or just after time <t> 
    events[index].t >= t   or   events[index].t
    Events is assumed to be sorted and not empty.
    If no event in list has time greater or equal to t 
    then, this will return index of last event
    Hint is meant to be an index at where to start searching.
    """
    index = hint
    index = saturate(index, 0, len(events)-1)
    while (index>=0) and (events[index].t >= t) :
        index -= 1
    while (index<len(events)) and (events[index].t < t) :
        index += 1
    index = saturate(index, 0, len(events)-1)
    return index



def getEventIndexRange(events, start_time, stop_time, start_hint=0, stop_hint=0):
    """ Returns an iterator of all events in <event_list> 
    that occur a ceratin time <before>, or a certain time <after> an certain <event> """
    start_index = getIndexAfter(start_time, events, start_hint)
    stop_index = getIndexBefore(stop_time,  events, stop_hint) + 1
    return (start_index, stop_index)


def sortEvents(events):
    timed_events = [ (event.t, event) for event in events ]
    timed_events.sort()    
    return [ event for t,event in timed_events ]


class StartAndStopCache(object):
    def __init__(self):
        self.start_index = 0
        self.stop_index  = 0

    def getEventIndexRange(self, events, start_time, stop_time):
        start_index,stop_index =  getEventIndexRange(events, start_time, stop_time, self.start_index, self.stop_index)
        self.start_index,self.stop_index = (start_index,stop_index)
        return (start_index,stop_index)


class _FakeEvent(object):
    def __init__(self, t):
        self.t = t

class TestEventTools(unittest.TestCase):

    def eventsFromTimes(self, times):
        return [ _FakeEvent(t) for t in times ]

    def verifyGetIndexBeforeSolution(self, t, events, index):
        self.assertTrue(event[index].t >= t)

    def verifyGetIndexAfterSolution(self, t, events, index):
        # make sure that event time fullfills requirements                
        self.assertTrue( (events[index].t >= t) or (index == (len(events)-1)) )
        # also make sure that earlier event does not fullfill requirements
        if (index > 0) : # 
            self.assertFalse(events[index-1].t >= t)


    # Test that ErrorDelay class delays error messages properly
    def setUp(self):
        self.merge_list = []
    
    def runGetIndexBeforeWithHints(self, t, events, expected_index):
        for hint in range(-3, len(events)+3):
            index = getIndexBefore(t, events, hint)
            self.assertTrue(index == expected_index, msg=(t,hint,expected_index,index))

    def runGetIndexAfterWithHints(self, t, events, expected_index):
        self.verifyGetIndexAfterSolution(t, events, expected_index)
        for hint in range(-3, len(events)+3):
            index = getIndexAfter(t, events, hint)
            self.assertTrue(index == expected_index, msg=(t,hint,expected_index,index))

    # Simple test that only inserts one new element at a time
    def test_1_getIndexBefore(self):
        #        0  1  2  3  4  5  6  7  8  9 10 11  12  13
        times = [1, 1, 1, 2, 3, 4, 4, 6, 7, 7, 7, 9, 10, 10]
        events = self.eventsFromTimes(times)

        self.runGetIndexAfterWithHints(0, events, 0)
        self.runGetIndexAfterWithHints(0.5, events, 0)
        self.runGetIndexAfterWithHints(1.5, events, 3)
        self.runGetIndexAfterWithHints(2, events, 3)
        self.runGetIndexAfterWithHints(2.5, events, 4)
        self.runGetIndexAfterWithHints(3, events, 4)
        self.runGetIndexAfterWithHints(4, events, 5)
        self.runGetIndexAfterWithHints(8, events, 11)
        self.runGetIndexAfterWithHints(9, events, 11)
        self.runGetIndexAfterWithHints(10, events, 12)
        self.runGetIndexAfterWithHints(100, events, 13)

if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG,'test_delay_queue', TestEventTools)
    #unittest.main()
