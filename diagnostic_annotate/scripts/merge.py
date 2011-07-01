#!/usr/bin/env python
# filter events

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import rospy

from diagnostic_annotate.diag_event import DiagEvent
from diagnostic_annotate.event_tools import StartAndStopCache, sortEvents

import sys
import yaml

class RunstopMerge(object):
    """ Merge Runstop and undervoltage events into one """
    def __init__(self):
        pass

    def process(self,events):
        if len(events) == 0:
            return []

        index_cache = StartAndStopCache()
        before = rospy.Duration(10.0)
        after = rospy.Duration(10.0)

        for current_event in events:
            if current_event.type == "RunStopEvent":
                print "Runstop", current_event
                start_time = current_event.t - before
                stop_time  = current_event.t + after    
                (start_index,stop_index) = index_cache.getEventIndexRange(events, start_time, stop_time)
                for event in events[start_index:stop_index]:
                    if (not event.hide):
                        if (event.type == "UndervoltageLockoutEvent") or (event.type == "UndervoltageLockoutMerge"):
                            print " Runstop Merge", event
                            event.hide = True
                            current_event.children.append(event)
                        if event.type == "CircuitBreakerTrip":
                            print " Runstop Merge", event
                            event.hide = True
                            current_event.children.append(event)
                            current_event.data['breaker_trip'] = True
                        if event.type == "OnlyMotorHaltedEvent":
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
        if len(events) == 0:
            return []

        new_events = []

        index_cache = StartAndStopCache()
        before = rospy.Duration(1.0)
        after = rospy.Duration(2.0)

        for current_event in events:
            if (current_event.type == "UndervoltageLockoutEvent") and (not current_event.hide):
                new_event = DiagEvent('UndervoltageLockoutMerge', '<MERGE>', current_event.t, "")
                start_time = current_event.t - before
                stop_time  = current_event.t + after    
                (start_index,stop_index) = index_cache.getEventIndexRange(events, start_time, stop_time)
                uv_count = 0
                for event in events[start_index:stop_index]:
                    if (not event.hide) and (event.type == "UndervoltageLockoutEvent"):
                        uv_count += 1
                        event.hide = True
                        new_event.children.append(event)
                new_event.desc = "Merge of %d devices" % uv_count
                new_events.append(new_event)
                #current_event.name = "%d devices" % uv_count
        
        results = new_events
        for event in events:
            if not event.hide:
                results.append(event)
        return results


class IntervalMerge(object):
    """ Merges that occur within certain time of each other into IntervalGroup. """
    def __init__(self, interval_break):
        """ interval break is the min of seconds between two events before a new interval is created """
        self.interval_break = interval_break

    @staticmethod
    def genInterval(event):    
        interval_event =  DiagEvent('IntervalMerge', '<interval>', event.t, "")
        interval_event.children.append(event)
        return interval_event

    @staticmethod
    def finalizeInterval(results, interval):
        """ If interval only has one child, just add child to results not interval"""
        if len(interval.children) == 1:
            #print "Single element Interval", interval.children[0]
            results.append(interval.children[0])
        else:
            #print "Interval with %d children" % len(interval.children)
            results.append(interval)
        
    def process(self, events):
        results = []
        interval = IntervalMerge.genInterval(events[0])
        for event in events[1:]:
            if abs( (interval.t-event.t).to_sec() ) < self.interval_break:
                interval.children.append(event)
            else:                
                IntervalMerge.finalizeInterval(results, interval)
                interval = IntervalMerge.genInterval(event)

        IntervalMerge.finalizeInterval(results, interval)
        return results



class RemoveEventTypes(object):
    """ Remove certain Event types from list """
    def __init__(self, event_types):
        self.event_types = set(event_types)

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
   
    events = sortEvents(events)
    events = RemoveEventTypes(['RxError', 'DroppedPacket', 'LatePacket', 'GenericEvent']).process(events)
    events = sortEvents(events)
    events = UndervoltageMerge().process(events)
    events = sortEvents(events)
    events = RunstopMerge().process(events)
    events = sortEvents(events)
    events = IntervalMerge(12.0).process(events)
    events = sortEvents(events)    

    yaml_events = [ event.to_yaml() for event in events ]
    y['events'] = yaml_events
    y['merged'] = True

    fd = open(output_filename,'w')
    yaml.dump(y,stream=fd)
    fd.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
