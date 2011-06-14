# Header message containing information about saved bag file
time   save_time     # wall time when bag was saved
string topic_name    # name of ROS topic (probabbly /diagnostics) that data was recorded from
string machine_name  # name of machine that bag was saved from 
string user_name     # name of current user when bag was saved
int32  major_version # major version (should be 1)
int32  minor_version # minor version 
string yaml_data     # Extra data encoded in yaml format.  
                     # When format of yaml_data changes in backwards compatible manor, minor version should increase.
                     # When format of yaml_data changes in non-compatible way, major version should increase   