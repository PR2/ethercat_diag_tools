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
import sys 
from itertools import islice

class SortedMarker:
    def __init__(self, key):
        self.key = key
        self.cached_index = 0

    def find(self, L, value):
        """ Returns first index in sorted list <L> where value > key(L[index]) 
        uses cached index to speed lookup
        returns length of array when index can not be found
        """
        if len(L) == 0:
            raise RuntimeError("input list L is empty")

        key = self.key
        index = self.cached_index
        if index >= len(L):
            index = len(L)-1

        if (value >= key(L[index])):
            while (index < len(L)) and (value >= key(L[index])):
                index += 1
        else:
            while (index >= 0) and (value < key(L[index])):
                index -= 1
            index+=1

        self.cached_index = index
        return index

    def findEx(self, L, value):
        """ Throws exception when index cannot be found"""
        index = self.find(L, value)
        if index>=len(L):
            raise RuntimeError("Could not find value in list > %s" % str(value))
        return index


    def slow_find(self, L, value):
        """ Returns smallest index in sorted list <L> where value > key(L[index]) """
        index = 0        
        while (index < len(L)) and (value >= self.key(L[index])):
            index += 1
        if index>=len(L):
            raise RuntimeError("Could not find value in list > %s" % str(value))
        return index


class TestSortedMarker(unittest.TestCase):
    # Test that ErrorDelay class delays error messages properly
    def setUp(self):
        pass

    # 
    def run_lookup(self, marker, cached_index, L, value):
        try :        
            self.run_lookup_ex(marker, cached_index, L, value)
        except RuntimeError:
            self.fail("RuntimeError cached_index=%d, value=%f, " % (cached_index, value))


    def run_lookup_ex(self, marker, cached_index, L, value):
        marker.cached_index = cached_index
        index = marker.findEx(L, value)
        cached_val = str(L[cached_index]) if cached_index < len(L) else 'X'
        self.assertTrue(L[index] > value, "%d=%s,%f,%d=%f" % (cached_index,cached_val,value,index,L[index]))
        if index > 0:
            self.assertTrue(L[index-1] <= value, "%d=%s,%f,%d=%f" % (cached_index,cached_val,value,index,L[index-1]))
            
    
    # runs list with different cached indexes
    def run_list(self, marker, L):
        epsilon = 0.01
        for cached_index in range(len(L)+1):            
            for index in range(len(L)-1):
                self.run_lookup(marker, cached_index, L, L[index]-epsilon)
                if L[index] < L[-1]:
                    self.run_lookup(marker, cached_index, L, L[index])
                    self.run_lookup(marker, cached_index, L, L[index]+epsilon)
                else:
                    self.assertRaises(RuntimeError, self.run_lookup_ex, marker, cached_index, L, L[-1])
                    self.assertRaises(RuntimeError, self.run_lookup_ex, marker, cached_index, L, L[-1]+epsilon)

    # Simple test that only inserts one new Error at a time
    def test_1_single(self):        
        self.run_list(SortedMarker(float), [0.0])
        
    def test_2_double(self):
        self.run_list(SortedMarker(float), [0.0,1.0])

    def test_3_equal(self):
        self.run_list(SortedMarker(float), [0.0,0.0,1.0,1.0])

    def test_4_equal2(self):
        self.run_list(SortedMarker(float), [0.0,0,0,1.0,2.0,2.0])



class DelayQueue:
    def __init__(self, merge, key, future_delay, past_delay):
        self.queue = []
        self.key = key
        self.merge = merge
        self.end_marker = SortedMarker(key=key)
        self.mid_marker = SortedMarker(key=key)
        self.begin_marker = SortedMarker(key=key)
        self.future_delay = future_delay
        self.past_delay = past_delay
        self.last_mid_index = 0
    
    def process(self, input_list):
        #  queue  = [2, 4, 4, 5, 6, 6, 7, 10]
        #            |                    |
        #            |                    +- newest element (start)
        #            |
        #            +- oldest element (end)
        #
        if len(input_list) == 0:
            return []
                
        key = self.key        
        queue = self.queue + input_list
        #print 'Q', queue
        # now figure out where new mid and end times are
        start_time = key(queue[-1])
        mid_time = start_time - self.future_delay
        #print 'start', start_time, 'mid', mid_time
        # find index's of min and end
        new_mid_index = self.mid_marker.findEx(queue,mid_time)
        #print 'new mid',new_mid_index, 'last mid', self.last_mid_index

        for index in range(self.last_mid_index, new_mid_index):
            current = queue[index]
            mid_time = key(current)
            start_time = mid_time + self.future_delay
            end_time = mid_time - self.past_delay
            start_index = self.end_marker.find(queue,start_time)
            end_index = self.end_marker.findEx(queue,end_time)
            future_list = islice(queue,index+1,start_index)
            past_list = islice(queue,end_index,index)
            self.merge(current,future_list,past_list)
            
        # now cut off new end of queue
        start_time = key(queue[-1])
        end_time = start_time - self.past_delay - self.future_delay
        end_index = self.end_marker.findEx(queue,end_time)     
        #print 'end time', end_time, 'end_index', end_index

        output_list = queue[:end_index]
        #print 'Q$-1', queue
        self.queue = queue[end_index:]
        #print 'queue[end_index:]', queue[end_index:]
        #print 'Q$', self.queue
        self.last_mid_index = new_mid_index-end_index

        return output_list


def print_merge(current, future, past):
    print [ [i for i in past], current, [i for i in future] ]


def find_sublist(mainlist, sublist):
    """ Returns index in mainlist where sublist first occurs, or -1 if sublist cannot be found """
    match_index = -1
    for start in range( len(mainlist)-len(sublist)+1 ):
        local_match = True
        for i in range(len(sublist)):
            if (mainlist[start+i]!=sublist[i]):
                local_match = False
                break
        if local_match:
            match_index = start
            break
    return match_index


class TestDelayQueue(unittest.TestCase):
    # Test that ErrorDelay class delays error messages properly
    def setUp(self):
        self.merge_list = []
    
    # 
    def run_lookup(self, marker, cached_index, L, value):
        try :        
            self.run_lookup_ex(marker, cached_index, L, value)
        except RuntimeError:
            self.fail("RuntimeError cached_index=%d, value=%f, " % (cached_index, value))


    def run_lookup_ex(self, marker, cached_index, L, value):
        marker.cached_index = cached_index
        index = marker.find(L, value)
        cached_val = str(L[cached_index]) if cached_index < len(L) else 'X'
        self.assertTrue(L[index] > value, "%d=%s,%f,%d=%f" % (cached_index,cached_val,value,index,L[index]))
        if index > 0:
            self.assertTrue(L[index-1] <= value, "%d=%s,%f,%d=%f" % (cached_index,cached_val,value,index,L[index-1]))
            
    
    # runs list with different cached indexes
    def run_list(self, marker, L):
        epsilon = 0.01
        for cached_index in range(len(L)+1):            
            for index in range(len(L)-1):
                self.run_lookup(marker, cached_index, L, L[index]-epsilon)
                if L[index] < L[-1]:
                    self.run_lookup(marker, cached_index, L, L[index])
                    self.run_lookup(marker, cached_index, L, L[index]+epsilon)
                else:
                    self.assertRaises(RuntimeError, self.run_lookup_ex, marker, cached_index, L, L[-1])
                    self.assertRaises(RuntimeError, self.run_lookup_ex, marker, cached_index, L, L[-1]+epsilon)

    def merge(self, current, future, past):
        self.merge_list.append(current)
        # Make sure past+current+future is actual sequence in sublist
        L = [i for i in past] + [current] + [i for i in future]
        index = find_sublist(self.input, L)
        self.assertTrue(index!=-1, "can't find %s in input %s" %(str(L), str(self.input)))

    def verify_results(self, output):
        self.assertTrue(output == self.input, "out=%s, in=%s" % (str(output), str(self.input)))
        self.assertTrue(self.merge_list == self.input, "merge=%s, in=%s" % (str(self.merge_list), str(self.input)))
    
    # Simple test that only inserts one new element at a time
    def test_1(self):
        dq = DelayQueue(self.merge,int,2,5)
        output = []
        self.input = [i for i in range(10)]
        for i in self.input:
            output += dq.process([i])
        output += dq.process([1000]) # flush
        self.verify_results(output)

    # Put in all elements at a time
    def test_2(self):
        dq = DelayQueue(self.merge,int,2,5)
        output = []
        self.input = [i for i in range(10)]
        output += dq.process(self.input)
        output += dq.process([1000]) # flush
        self.verify_results(output)

    # Try test with matching elements
    def test_3(self):
        dq = DelayQueue(self.merge,int,2,5)
        output = []
        self.input = [0,1,2,3,4,5,5,5,6,7,8,9]
        output += dq.process(self.input)
        output += dq.process([1000]) # flush
        self.verify_results(output)



if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG,'test_delay_queue', TestDelayQueue)
    #unittest.main()
