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

import time
import math

def prettyTimestamp(timestamp):
    """ Returns ros timestamp in pretty format """
    return time.strftime("%a, %b %d %Y, %I:%M.%S %p", time.localtime(timestamp.to_sec()))

def prettyDuration(duration):
    """ Returns duration into pretty format  Hours,Min,Sec """
    secs = duration.to_sec()
    in_future = False
    if (secs < 0):
        secs = -secs
        in_future = True
    secs_per_min  = 60.0
    secs_per_hour = secs_per_min * 60.0
    secs_per_day  = secs_per_hour * 24.0
    secs_per_week = secs_per_day * 7.0

    weeks = math.floor(secs / secs_per_week)
    secs -= weeks * secs_per_week
    days = math.floor(secs / secs_per_day)
    secs -= days * secs_per_day
    hours = math.floor(secs / secs_per_hour)
    secs -= hours * secs_per_hour
    mins = math.floor(secs / secs_per_min)
    secs -= mins * secs_per_min
    result = ""
    if weeks > 0:    
        result += ("%d week%s "%(weeks, "s" if weeks > 1 else ""))
    if days > 0:    
        result += ("%d day%s "%(days, "s" if days > 1 else ""))
    if hours > 0:
        result += ("%d hour%s "%(hours, "s" if hours > 1 else ""))
    if mins > 0:
        result += ("%d minute%s "%(mins, "s" if mins > 1 else ""))
    if len(result) > 0:
        result += "and "
    result += "%d seconds "%int(secs)
    if in_future:
        result += "in the future"
    else:
        result += "ago"
    return result


def mergeDevices(devices1, devices2):
    """ creates a dict of device tuple based on the device name """
    dev_map = {}
    for dev in devices1:
        if dev.name in dev_map:
            raise Exception('Device with name %s in list twice' % dev.name)
        else:
            dev_map[dev.name] = [dev,None]
        
    for dev in devices2:
        if dev.name in dev_map:
            dev_pair = dev_map[dev.name]
            if (dev_pair[1] != None):
                raise Exception ('Device with name s% in list twice' % dev.name)
            dev_pair[1] = dev
        else:
            dev_map[dev.name] = [None,dev]

    return dev_map
