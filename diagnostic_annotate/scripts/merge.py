#!/usr/bin/env python
# filter events

PKG = 'diagnostic_annotate'
import roslib
roslib.load_manifest(PKG)
import rospy

from diagnostic_annotate.diag_event import DiagEvent
from diagnostic_annotate.event_tools import StartAndStopCache, sortEvents
from diagnostic_annotate.merge_filters import filterPipeline1

import sys
import yaml

    
def main(argv):    
    input_filename = argv[1]
    output_filename = argv[2]    
    #process_bag(inbag_filename, output_filename)

    fd = open(input_filename)
    y = yaml.load(fd)
    fd.close()
    
    yaml_events = y['events']
    events = [ DiagEvent.from_yaml(yaml_event) for yaml_event in yaml_events ]

    events = filterPipeline1(events)

    yaml_events = [ event.to_yaml() for event in events ]
    y['events'] = yaml_events
    y['merged'] = True

    fd = open(output_filename,'w')
    yaml.dump(y,stream=fd)
    fd.close()

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
