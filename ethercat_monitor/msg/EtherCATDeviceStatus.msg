# represents the data for a single EtherCAT device
string name
string hardware_id
int32  ring_position
int32  epu_errors
int32  pdi_errors
bool   valid 
EtherCATDevicePortStatus[] ports  # EtherCAT devices have 1-4 ports