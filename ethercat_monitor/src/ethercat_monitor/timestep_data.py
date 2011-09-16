
import ethercat_monitor.msg

from ethercat_monitor.util import mergeDevices

def getPortDiff(port_new, port_old):
    """ Returns difference in error counters between this and old values"""
    port_diff = ethercat_monitor.msg.EtherCATDevicePortStatus()
    port_diff.rx_errors = port_new.rx_errors - port_old.rx_errors
    port_diff.forwarded_rx_errors = port_new.forwarded_rx_errors - port_old.forwarded_rx_errors
    port_diff.frame_errors = port_new.frame_errors - port_old.frame_errors
    port_diff.lost_links = port_new.lost_links - port_old.lost_links
    port_diff.est_drops = port_new.est_drops - port_old.est_drops

    if port_new.open != port_old.open:
        port_diff.open = None
    else:
        port_diff.open = port_new.open
    return port_diff



def getDeviceDiff(ds_new, ds_old):        
    """ Gets (error) difference between new and old device """     
    num_ports = max(len(ds_new.ports), len(ds_old.ports))
    ds_diff = ethercat_monitor.msg.EtherCATDeviceStatus()
    if ds_new.name != ds_old.name:
        raise Exception("Name of old and new device does not match")
    ds_diff.name = ds_new.name
    ds_diff.epu_errors = ds_new.epu_errors - ds_old.epu_errors
    ds_diff.pdi_errors = ds_new.pdi_errors - ds_old.pdi_errors
    ds_diff.valid = ds_new.valid and ds_old.valid
    if ds_new.hardware_id == ds_old.hardware_id:
        ds_diff.hardware_id = ds_new.hardware_id
    else:
        ds_diff.hardware_id = "Mismatch %s != %s" % (ds_new.hardware_id, ds_old.hardware_id)
    for num in range(num_ports):
        if (num >= len(ds_new.ports)) or (num >= len(ds_old.ports)):
            ds_diff.ports.append(EtherCATDevicePortMissing())
        else:
            ds_diff.ports.append(getPortDiff(ds_new.ports[num], ds_old.ports[num]))
    if ds_new.ring_position != ds_old.ring_position:
        ds_diff.ring_position = None
    else:
        ds_diff.ring_position = ds_new.ring_position
    return ds_diff


def getMasterDiff(new, old):
    """ Returns difference in counters between new and old master structures"""
    diff = ethercat_monitor.msg.EtherCATMasterStatus()
    diff.sent    = new.sent    - old.sent
    diff.dropped = new.dropped - old.dropped
    diff.late    = new.late    - old.late
    diff.unassigned_drops = new.unassigned_drops - old.unassigned_drops
    return diff


class EtherCATTimestepSummary:
    def __init__(self):
        self.timestamp = None
        self.dropped = 0
        self.late = 0
        self.lost_links = 0
        self.frame_errors = 0
        self.sent = 0

class EtherCATHistoryTimestepData:
    def __init__(self,system):
        self.system = system
        self.timestamp_old = None

    def getTimestamp(self):
        return self.system.stamp
        
    def hasData(self):        
        if (len(self.system.devices) > 0) and self.has_master:
            return True
        elif (len(self.system.devices) == 0) and not self.has_master:
            return False
        elif (len(self.system.devices) > 0) and not self.has_master:
            raise RuntimeError("Devices but no master")
        else:
            raise RuntimeError("Master but no devices")

    def getSummary(self):
        summary = EtherCATTimestepSummary()
        frame_errors = 0
        lost_links = 0
        for dev in self.getDevices():
            for port in dev.ports:
                frame_errors += port.frame_errors
                lost_links += port.lost_links
        summary.timestamp    = self.getTimestamp()
        summary.frame_errors = frame_errors
        summary.lost_links   = lost_links
        summary.late         = self.getMaster().late
        summary.dropped      = self.getMaster().dropped
        summary.dropped      = self.getMaster().sent
        return summary

    def numDevices(self):
        return len(self.system.devices)

    def addDevice(self, device_status):        
        self.system.devices.append(device_status)        

    def addMaster(self, master):
        self.system.master = master

    def __str__(self):
        result = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.getTimestamp().to_sec())) + '\n'
        result += "Master\n" + str(self.master)
        for name,device in self.devices.iteritems():
            result += " " + name + " : \n" + str(device)
        result += '--\n'
        return result

    def getDevices(self):
        return self.system.devices

    def getMaster(self):
        return self.system.master

    def getDiff(self, timestamp_data_old):
        """ returns new EthercatHistoryTimestampData that represents difference
        between this timestamp and older timestamp"""

        sys_status = ethercat_monitor.msg.EtherCATSystemStatus()
        sys_status.stamp = self.system.stamp
        tsd_diff = EtherCATHistoryTimestepData(sys_status)

        tsd_old = timestamp_data_old
        tsd_diff.timestamp_old = tsd_old.getTimestamp()

        tsd_diff.addMaster(getMasterDiff(self.getMaster(), tsd_old.getMaster()))
        dev_map = mergeDevices(self.getDevices(), tsd_old.getDevices())

        #First match every device in this list to device in old data
        for name, (dev_new, dev_old) in dev_map.iteritems():
            if (dev_new is None) or (dev_old is None):
                print "Device '%s' not found" % name
                tsd_diff.addDevice(EtherCATDeviceMissing(name))
            else:
                dev_diff = getDeviceDiff(dev_new, dev_old)
                tsd_diff.addDevice(dev_diff)

        return tsd_diff

    
    def generateYaml(self): 
        """ Generated YAML description of data.  Might be deprecated in future """
        master = self.system.master
        master_out = {'sent':master.sent, 'dropped':master.dropped, 'late':master.late}

        devices_out = {}
        for dev in self.system.devies:
            dev_out = {'valid':dev.valid, 'position':dev.ring_position }
            dev_out['epu_errors']=dev.epu_errors
            dev_otu['pdi_errors']=dev.pdi_errors
            for port in dev.ports:
                port_out = {'rx_errors':port.rx_errors}
                port_out['forwarded_rx_errors'] = port.forwarded_rx_errors
                port_out['frame_errors'] = port.frame_errors
                port_out['lost_links'] = port.lost_links
            devices_out[dev.name] = dev_out
        out['devices'] = devices_out

        out['master':master_out, 'devices':devices_out]
        out['devices'] = device_out
        
        timestamp = self.getTimestamp()
        if self.timestamp_old is not None:
            duration = timestamp - self.timestamp_old
            out['duration'] = prettyDuration(duration)
        else:
            out['date'] = prettyTimestamp(timestamp)
            out['ros_time'] = {'secs':timestamp.secs, 'nsecs':timestamp.nsecs}

        return out



class EtherCATHistoryTimestepDataNote:
    """ Represents save timestamp data with message attached to it """
    def __init__(self, timestep_data, note_msg):
        self.timestep_data = timestep_data
        self.note_msg = note_msg
