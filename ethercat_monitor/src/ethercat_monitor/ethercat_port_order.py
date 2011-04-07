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
##\brief Arranges EtherCAT devices and ports in order packet would be recieved at various ports


def Pooooo():
    pass

# Notes EtherCAT device port forwarding order
#  
# 4-port device
#  0 -> EPU -> 3 -> 1 -> 2 -> 0
# 
# 3-port device 
#  0 -> EPU -> 1 -> 2 -> 0
#
# 2-port device
#  0 -> EPU -> 1 -> 0
#

# The order the devices are numbered in matches the order in which a 
# packet first hits the EPU of each device. 
# In properly cable system all port0 are pointing toward master.  
# This means that for each device packet comes in port 0 first


# Below is a theroetical system 
# 
#                 D------      E------
#            -----| 0  1 | --- | 0  1 |
#  M        |      ------       ------
#  a    A-------     
#  s    |   3   |     B------      C------
#  t -- | 0   1 | --- | 0  1 | --- | 0  1 |
#  e    |   2   |      ------       ------
#  r     -------
#           |     F------ 
#            -----| 0  1 |
#                  ------
#
# The ordering of devices:
#
#    Master -> A -> D -> E -> B -> C -> F
#
# The order that a packet would be recieved on a port
#
#    A0 -> D0 -> E0 -> D1 -> A3 -> B0 -> C0 -> B1 -> A1 -> F0 -> A2 -> Master
#


# Inputs (What we know)
#  * The order the devices are present in the system 
#  * Whether a given port is open/closed

# Output (What we want to figure out) 
#  * What order packet would be received on each port

# Pseduo code
#
# Create Stack S that contains tuple with (device, port_number)
# Make Stack P with all devices in device order
# Push first device and port 0 on stack  (first_device, 0)
# while (stuff in S)
#   dev,portnum = pop S
#   next_port = lookup(portnum)   use lookup table to determine what the next port number is
#   if next_port is not None
#     if next_port is open 
#       push this device next port on stack (this_dev, next_port)
#       pop next device off of P
#       push device P port 



def getNextPort(device, port_num):
    """ Returns tupple (port,port_num) of next open port of device (in port order).
    If there is no next port, returns (None, -1)"""

    # port order 0->3->1->2
    # 0 is first (0)
    # 1 is third (2)
    # 2 is last (3)
    # 3 is second (1)
    order_index = [0,2,3,1]
    port_order = [0,3,1,2]

    next_index = order_index[port_num] + 1    
    while next_index < 4:
        next_port_num = port_order[next_index]
        if next_port_num < len(device.ports):
            next_port = device.ports[next_port_num]
            if next_port.open:
                return (next_port, next_port_num)
        next_index = order_index[next_port_num] + 1    
    return (None, -1)
    

def genPortOrder(device_map):
    """Generates a orderred list of tuples (dev_name, device, port_num, port) that represents 
    The order that a packet should travel through a list of devices.  
    The input to this function is a map of device name to devices
    """

    if (len(device_map) == 0):
        return []

    # Make list of tuples (position,dev_name,device) that can be easily sorted by position
    devices = [ (device.ring_position, dev_name, device) for dev_name,device in device_map.iteritems() ]    
    devices.sort()
    
    # first do quick check to make sure devices is ordered by position
    for expected_position,(position,dev_name,device) in enumerate(devices):
        if expected_position != position:
            raise RuntimeError("Expected device %d to have position %d, not %d" % (dev_name, expected_position, position))
        
    # now put devices in reverse order
    devices.reverse()
    
    # make stack of stuff to look at
    # stack consists of tuple, ( (position, dev_name, dev), port_num )
    stack = [ (devices.pop(), 0) ]
    result = []

    while len(stack) > 0:
        dev_info, port_num = stack.pop()
        (position, dev_name, device) = dev_info
        result.append(  (dev_name, device, port_num, device.ports[port_num])  )
        next_port,next_port_num = getNextPort(device, port_num)
        if next_port is not None:
            # first append next port of this device to stack
            stack.append( (dev_info, next_port_num) )
            # then append next port of next device to stack
            stack.append( (devices.pop(), 0) )

    return result
