#!/usr/bin/env python
# filter events

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import rospy

from diagnostic_annotate.diag_event import DiagEvent

import sys
import yaml

def saturate(index, minval, maxval):
    if index < minval:
        return minval
    if index > maxval:
        return maxval
    return index


def getTimeIndex(t, events, hint):
    index = hint
    index = saturate(index, 0, len(events))
    while (index<len(events)) and (events[index].t < t) :
        index += 1
    index = saturate(index, 0, len(events))
    while (index>=0) and (events[index].t > t) :
        index -= 1
    index = saturate(index, 0, len(events))
    return index


def getEventIndexRange(events, start_time, stop_time, start_hint=0, stop_hint=0):
    """ Returns an iterator of all events in <event_list> 
    that occur a ceratin time <before>, or a certain time <after> an certain <event> """
    start_index = getTimeIndex(start_time, events, start_hint)
    stop_index   = getTimeIndex(stop_time,  events, stop_hint)
    return (start_index, stop_index)


class StartAndStopCache(object):
    def __init__(self):
        self.start_index = 0
        self.stop_index  = 0

    def getEventIndexRange(self, events, start_time, stop_time):
        start_index,stop_index =  getEventIndexRange(events, start_time, stop_time, self.start_index, self.stop_index)
        self.start_index,self.stop_index = (start_index,stop_index)
        return (start_index,stop_index)


class RunstopMerge(object):
    """ Merge Runstop and undervoltage events into one """
    def __init__(self):
        pass

    def process(self,events):
        index_cache = StartAndStopCache()
        before = rospy.Duration(100.0)
        after = rospy.Duration(100.0)

        for current_event in events:
            if current_event.type == "RunStopEvent":
                print "Runstop", current_event
                start_time = current_event.t - before
                stop_time  = current_event.t + after    
                (start_index,stop_index) = index_cache.getEventIndexRange(events, start_time, stop_time)
                for event in events[start_index:stop_index]:
                    if (not event.hide) and (event.type == "UndervoltageLockoutEvent"):
                        print " Runstop Merge", event
                        event.hide = True
                        current_event.children.append(event)
            
        results = []
        for event in events:
            if not event.hide:
                results.append(event)

        return results

class UndervoltageMerge(object):
    """ Merge Undervoltage events for different devices into one """
    def __init__(self):
        pass

    def process(self,events):
        index_cache = StartAndStopCache()
        before = rospy.Duration(1.0)
        after = rospy.Duration(3.0)

        for current_event in events:
            if (current_event.type == "UndervoltageLockoutEvent") and (not current_event.hide):
                print "Undervoltage Merge", current_event
                start_time = current_event.t - before
                stop_time  = current_event.t + after    
                (start_index,stop_index) = index_cache.getEventIndexRange(events, start_time, stop_time)
                uv_count = 1
                for event in events[start_index:stop_index]:
                    if (not event.hide) and (event.type == "UndervoltageLockoutEvent") and (event is not current_event):
                        uv_count += 1
                        print " UV Merge", event
                        event.hide = True
                        #current_event.children.append(event)
                current_event.name = "%d devices" % uv_count

            
        results = []
        for event in events:
            if not event.hide:
                results.append(event)

        return results


class RemoveEventTypes(object):
    """ Merge Runstop and undervoltage events into one """
    def __init__(self, event_types):
        self.event_types = set(event_types)
        pass

    def process(self,events):
        results = []
        for event in events:
            if event.type not in self.event_types:
                results.append(event)                

        return results

        
    
def main(argv):    
    input_filename = argv[1]
    output_filename = argv[2]    
    #process_bag(inbag_filename, output_filename)

    fd = open(input_filename)
    y = yaml.load(fd)
    fd.close()
    
    yaml_events = y['events']
    events = [ DiagEvent.from_yaml(yaml_event) for yaml_event in yaml_events ]

    events = UndervoltageMerge().process(events)
    events = RunstopMerge().process(events)
    events = RemoveEventTypes(['RxError', 'DroppedPacket', 'LatePacket', 'GenericEvent']).process(events)
    
    yaml_events = [ event.to_yaml() for event in events ]

    fd = open(output_filename,'w')
    yaml.dump(yaml_events,stream=fd)
    fd.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
