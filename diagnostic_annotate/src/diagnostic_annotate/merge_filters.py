PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import rospy
import re
import copy

from diagnostic_annotate.diag_event import DiagEvent
from diagnostic_annotate.event_tools import StartAndStopCache, sortEvents

class RunstopMerge(object):
    """ Merge Runstop and undervoltage events into one """
    def __init__(self, no_motors_halted=False):
        self.matching_types = set( ("UndervoltageLockoutEvent", "UndervoltageLockoutMerge", "CircuitBreakerTrip", "MtraceSafetyLockout") )
        if not no_motors_halted:
            self.matching_types.add("OnlyMotorHaltedEvent")
            self.matching_types.add("MotorsHalted")

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
                        if event.type in self.matching_types:
                            event.hide = True
                            current_event.children.append(event)

        results = []
        for event in events:
            if not event.hide:
                results.append(event)

        return results


class CircuitBreakerMerge(object):
    """ Make any UndervoltageLockout event a child of neaby CircuitBreakerTrips"""
    def __init__(self):
        pass

    def process(self,events):
        if len(events) == 0:
            return []

        index_cache = StartAndStopCache()
        before = rospy.Duration(10.0)
        after = rospy.Duration(10.0)

        matching_types = set( ("UndervoltageLockoutEvent","UndervoltageLockoutMerge","MtraceSafetyLockout") )

        for current_event in events:
            if current_event.type == "CircuitBreakerTrip":
                start_time = current_event.t - before
                stop_time  = current_event.t + after    
                (start_index,stop_index) = index_cache.getEventIndexRange(events, start_time, stop_time)
                for event in events[start_index:stop_index]:
                    if (not event.hide):
                        if event.type in matching_types:
                            event.hide = True
                            current_event.children.append(event)

        results = []
        for event in events:
            if not event.hide:
                results.append(event)

        return results


class MotorsHaltedMerge(object):
    """ Merge Any Events Leading up to, or following Motor Halted"""
    def __init__(self, ignore_first_event=False):
        self.ignore_first_event = ignore_first_event
        pass    

    def process(self,events):
        if len(events) == 0:
            return []

        new_events = []

        index_cache = StartAndStopCache()
        before = rospy.Duration(10.0)
        after = rospy.Duration(10.0)

        for current_event in events:
            if (current_event.type == "MotorsHalted") and (not current_event.hide):
                current_event.hide = True
                if ('first' not in current_event.data) or not current_event.data['first'] or not self.ignore_first_event:
                    new_event = DiagEvent('MotorsHaltedMerge', '<MERGE>', current_event.t, "")
                    new_event.data = copy.deepcopy(current_event.data)
                    start_time = current_event.t - before
                    stop_time  = current_event.t + after    
                    (start_index,stop_index) = index_cache.getEventIndexRange(events, start_time, stop_time)
                    for event in events[start_index:stop_index]:
                        if not event.hide:
                            if event.type == "MotorsHalted":
                                break
                            event.hide = True
                            new_event.children.append(event)
                    subtypes = ', '.join(set( [c.type for c in new_event.children]))
                    new_event.name = subtypes
                    new_event.desc = "Motors Halted Merge : " + subtypes
                    new_events.append(new_event)
                
        results = new_events
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
                new_event.name = "%d devices" % uv_count
                new_events.append(new_event)
                #current_event.name = "%d devices" % uv_count
        
        results = new_events
        for event in events:
            if not event.hide:
                results.append(event)
        return results


class MultiRunstopMerge(object):
    """ Find Runstops that occur multiple times in succession"""
    def __init__(self):
        pass

    def process(self,events):
        last_runstop = None
        new_events = []
        for event in events:
            if event.type == "RunStopEvent":
                new_runstop = event
                if last_runstop is not None:
                    if abs((new_runstop.t - last_runstop.t).to_sec()) < 10:
                        print "Multi Runstop Merge"
                        new_event = DiagEvent('MultiRunstopMerge', '<>', last_runstop.t, "")
                        new_event.children = [last_runstop, event]     
                        last_runstop.hide = True
                        new_runstop.hide = True
                        new_events.append(new_event)
                last_runstop = new_runstop
                
        for event in events:
            if not event.hide:
                new_events.append(event)
        return  new_events


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
            #put set of children types in interval description
            subtypes = ', '.join(set( [c.type for c in interval.children ] ))
            interval.name = subtypes
            interval.desc = 'Interval : ' + subtypes
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



class EcatDeviceMerge(object):
    """ Merge all RX errors, lost links and, invalid frames for given device and port into 1 event"""
    def __init__(self):
        self.devices = {}

    def process(self, events):
        results = []
        types = ['RxError', 'LostLink']
        for event in events:
            if event.type in types:
                name = event.name
                port = event.data['port']
                if (name,port) not in self.devices:
                    #print "New ECat Device Merge", name, port
                    d = DiagEvent('EcatDeviceMerge', name, event.t, '%s port %d' % (name,port))
                    d.data = {'port':port, 'rx_errors':0, 'invalid_frames':0, 'lost_links':0}
                    self.devices[(name,port)] = d
                    results.append(d)
                d = self.devices[(name,port)]
                if event.type == 'RxError':
                    d.data['rx_errors'] += event.data['rx_errors']
                    d.data['invalid_frames'] += event.data['invalid_frames']
                elif event.type == 'LostLink':
                    d.data['lost_links'] += event.data['lost_links']
            else:
                results.append(event)
        return results


class EcatMasterMerge(object):
    """ Merge all packet drops and late packets for EtherCAT master """
    def __init__(self):
        self.d = None

    def process(self, events):
        results = []
        types = ['LatePacket', 'DroppedPacket']
        for event in events:
            if event.type in types:
                if self.d is None:
                    self.d = DiagEvent('EcatMasterMerge', event.name, event.t, event.name)
                    self.d.data = {'lates':0, 'drops':0}
                    results.append(self.d)
                d = self.d
                if event.type == 'LatePacket':                    
                    d.data['lates'] += event.data['lates']
                elif event.type == 'DroppedPacket':
                    d.data['drops'] += event.data['drops']                    
            else:
                results.append(event)
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


class KeepEventTypes(object):
    """ Keeps certain Event types in list and removes all others """
    def __init__(self, event_types):
        self.event_types = set(event_types)

    def process(self,events):
        results = []
        for event in events:
            if event.type in self.event_types:
                results.append(event)
        return results


class KeepEventTypesRegEx(object):
    """ Keeps certain Event types in list and removes all others """
    def __init__(self, regex_list):
        self.regex_list = [re.compile(regex) for regex in regex_list]

    def process(self,events):
        results = []
        for event in events:
            for regex in self.regex_list:
                if regex.match(event.type):
                    results.append(event)
        return results



def runFilters(filters, events):
    """ Run a list of filters on events """
    for f in filters:
        if len(events) == 0:
            return []
        for event in events:
            event.hide = False
        events = sortEvents(events)
        events = f.process(events)
    
    events = sortEvents(events)
    return events


def filterPassThrough(events):
    return sortEvents(events)


def filterBreakerTrips(events):
    """ Filters everthing out except circuit breaker trips that are not caused by Runstops """
    filters = []
    filters.append( UndervoltageMerge() )
    filters.append( RunstopMerge() )
    filters.append( CircuitBreakerMerge() )
    filters.append( KeepEventTypes(['CircuitBreakerTrip']) )
    return runFilters(filters,events)

def filterRecalibrate(events):
    """ Filter everything out except recalibrate events """
    filters = []
    filters.append( KeepEventTypes(['Recalibration']) )
    return runFilters(filters,events)

def filterCalibration(events):
    """ Filter everything out except recalibrate events """
    filters = []
    filters.append( KeepEventTypes(['Recalibration','Calibrated']) )
    return runFilters(filters,events)

def filterMultiRunstop(events):
    """ Filters everthing out except multiple runstops events """
    filters = []
    filters.append( UndervoltageMerge() )
    filters.append( RunstopMerge() )
    filters.append( MultiRunstopMerge() )
    filters.append( KeepEventTypes(['MultiRunstopMerge']) )
    return runFilters(filters,events)

def filterOnlyIgnored(events):
    filters = []
    filters.append( KeepEventTypes(['Ignored']) )
    return runFilters(filters,events)

def filterEcatCommunication(events):
    filters = []
    filters.append( KeepEventTypes(['RxError', 'DroppedPacket', 'LatePacket', 'LostLink']) )
    return runFilters(filters,events)

def filterEcatMerge(events):
    filters = []
    filters.append( KeepEventTypes(['RxError', 'DroppedPacket', 'LatePacket', 'LostLink']))
    filters.append( EcatDeviceMerge() )
    filters.append( EcatMasterMerge() )
    return runFilters(filters,events)

def filterMtrace(events):
    filters = []
    filters.append( KeepEventTypesRegEx(['.*Mtrace.*']) )
    filters.append( IntervalMerge(2.0) )
    return runFilters(filters,events)

def filterMotorModel(events):
    filters = []
    filters.append( KeepEventTypesRegEx(['.*MtraceMotorModel.*']) )
    filters.append( IntervalMerge(2.0) )
    return runFilters(filters,events)


def filterMotorsHalted(events):
    return _filterMotorsHalted(events, False)

def filterMotorHaltedNoFirst(events):
    return _filterMotorsHalted(events, True)

def _filterMotorsHalted(events, ignore_first_event):
    """ Only keeps MotorHalted events and any other event that occurs near them """
    filters = []
    filters.append( RemoveEventTypes(['Ignored']) )
    filters.append( UndervoltageMerge() )
    filters.append( RunstopMerge(no_motors_halted=True) )  #don't merge in motor's halted events
    filters.append( MotorsHaltedMerge(ignore_first_event))
    filters.append( KeepEventTypes(['MotorsHalted', 'MotorsHaltedMerge']) )
    return runFilters(filters,events)


def filterLostLinks(events):
    """ Filter out everything except lost links on EtherCAT chain """
    filters = []
    filters.append( KeepEventTypes(['LostLink']) )
    filters.append( IntervalMerge(2.0) )  # merge lost links that happen at same time (since lost links often occur in pairs)
    return runFilters(filters,events)

def filterPipeline1(events):
    filters = []
    filters.append( RemoveEventTypes(['Ignored', 'RxError', 'DroppedPacket', 'LatePacket']) )
    filters.append( UndervoltageMerge() )
    filters.append( RunstopMerge() )
    filters.append( MultiRunstopMerge() )
    filters.append( IntervalMerge(12.0) )
    return runFilters(filters,events)
