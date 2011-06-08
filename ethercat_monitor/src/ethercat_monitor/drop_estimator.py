
from ethercat_monitor.util import mergeDevices


class DropEstimator:
    """ Assigns packet drops to one or more EtherCAT devices.
    Usually frame errors counters increase a cycle after a packet drops occurs. 
    To handle this, this keeps some history to properly assign packet 
    drops to the correct device.
    """
    def __init__(self):
        self.last_unassigned_drops = 0.0
        self.timestamp_data_old = None
        self.cycle_count = 0
    
    def process(self, timestamp_data):        
        """ Takes EtherCAT device and Master data and calculates 
            estimate dropped packets for each EtherCAT device port"""

        tsd_new = timestamp_data
        tsd_old = self.timestamp_data_old
        if tsd_old is None:
            self.timestamp_data_old = timestamp_data
            return 
        
        # This is a little sloppy because sometimes packets are counted as
        # late before they are counted as dropped, however this situation is
        # not incredibly common 
        new_master = tsd_new.getMaster()
        old_master = tsd_old.getMaster()
        dropped_new = new_master.dropped - new_master.late
        dropped_old = old_master.dropped - old_master.late
        drops = dropped_new - dropped_old

        # determine which device caused dropped packets, by looking for devices
        # where frame error count increased.  If multiple devices have frame errors
        # at same time, split dropped between devices based on how much frame error
        # increased for each device

        # Create list of port pairs between old and new data
        # port_pairs list is tupple of (device_name, port_number, port_new, port_old)
        dev_map = mergeDevices(tsd_new.getDevices(), tsd_old.getDevices())
        port_pairs = []
        for name, (dev_new, dev_old) in dev_map.iteritems():
            if (dev_new is None) or (dev_old is None):
                print "Warning, no device pair for device %s" % name
            else:
                if len(dev_old.ports) != len(dev_new.ports):
                    print "Number of ports do not match beween old (%d) an new (%d) devices" % (len(dev_old.ports), len(dev_new.ports))
                else:
                    for port_num in range(len(dev_new.ports)):                        
                        port_pairs.append( (name, port_num, dev_new.ports[port_num], dev_old.ports[port_num]) )


        # sum new frame_errors for all devices
        frame_error_sum = 0.0  # should be float
        for dev_name, port_num, port_new, port_old in port_pairs:
            frame_error_sum += port_new.frame_errors - port_old.frame_errors

        # multiple devices might have a frame_error for same packet 
        # However, only one packet is dropped from master's point of view.
        # There is also the possibility that the packet is corrupted (dropped) on the 
        # way back to the computer which would not generate any frame errors

        # First assign each unassigned drops from last cycle to frame errors for this cycle
        # Any remaining drops from last cycle will be given to ethercat master as unassigned drops
        device_drops = min(self.last_unassigned_drops, frame_error_sum)
        master_unassigned_drops = self.last_unassigned_drops - device_drops
        remaining_frame_error_sum  = frame_error_sum - device_drops
        new_master.unassigned_drops = old_master.unassigned_drops + master_unassigned_drops

        # take remaining drop and assign them to frame errors from this cycle
        # any remaining drops get passed to next cycle
        unassigned_drops = max(drops - remaining_frame_error_sum, 0)
        device_drops += drops - unassigned_drops

        self.cycle_count += 1
        if (device_drops > 0.0) or (frame_error_sum > 0.0) or (drops > 0.0) or (unassigned_drops > 0.0) or (self.last_unassigned_drops > 0.0):
            print
            print "cycle", self.cycle_count
            print "frame_error_sum", frame_error_sum
            print "drops", drops
            print "device_drops", device_drops
            print "remaining_frame_error_sum", remaining_frame_error_sum
            print "master_unassigned_drops", master_unassigned_drops
            print "last_unassigned_drops", self.last_unassigned_drops
            print "unassigned_drops", unassigned_drops
            print

        self.last_unassigned_drops = unassigned_drops

        if device_drops > 0:
            # divide drops against all devices with increased frame_error count            
            for dev_name, port_num, port_new, port_old in port_pairs:
                frame_errors = port_new.frame_errors - port_old.frame_errors
                port_new.est_drops = port_old.est_drops + device_drops * (frame_errors / frame_error_sum)    
        else:
            for dev_name, port_num, port_new, port_old in port_pairs:
                port_new.est_drops = port_old.est_drops

        self.timestamp_data_old = tsd_new

