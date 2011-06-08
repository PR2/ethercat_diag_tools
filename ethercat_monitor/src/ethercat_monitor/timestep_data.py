
import ethercat_monitor.msg


class EtherCATHistoryTimestepData:
    def __init__(self,system):
        self.system = system
        self.timestamp = system.header.stamp
        self.timestamp_old = None

    def hasData(self):        
        if (len(self.system.devices) > 0) and self.has_master:
            return True
        elif (len(self.system.devices) == 0) and not self.has_master:
            return False
        elif (len(self.system.devices) > 0) and not self.has_master:
            raise RuntimeError("Devices but no master")
        else:
            raise RuntimeError("Master but no devices")

    def numDevices(self):
        return len(self.system.devices)

    def addDevice(self, device_status):        
        self.system.devices.append(device_status)        

    def addMaster(self, master):
        if self.has_master:
            raise RuntimeError("Master data already set")        
        self.system.master = master
        self.has_master = True

    def __str__(self):
        result = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.timestamp.to_sec())) + '\n'
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

        tsd_old = timestamp_data_old
        tsd_diff = EtherCATHistoryTimestepData(self.system)
        tsd_diff.timestamp_old = tsd_old.timestamp

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

        if self.timestamp_old is not None:
            duration = self.timestamp - self.timestamp_old
            out['duration'] = prettyDuration(duration)
        else:
            out['date'] = prettyTimestamp(self.timestamp)
            out['ros_time'] = {'secs':self.timestamp.secs, 'nsecs':self.timestamp.nsecs}

        return out



class EtherCATHistoryTimestepDataNote:
    """ Represents save timestamp data with message attached to it """
    def __init__(self, timestep_data, note_msg):
        self.timestep_data = timestep_data
        self.note_msg = note_msg
