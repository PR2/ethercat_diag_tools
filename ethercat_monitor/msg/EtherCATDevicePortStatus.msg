# Represents diagnostic information for a signle port of an EtherCAT device
int32 rx_errors
int32 forwarded_rx_errors
int32 frame_errors
int32 lost_links
bool open

# estimate of dropped packets that where cause by this port
# calculated by correlating occurence of frame_errors with drops from ethercat master data
int32 est_drops  

