import labscript_utils.h5_lock
import h5py
import numpy as np
import re

import labscript_utils.properties as properties
from labscript_utils import dedent
from labscript import LabscriptError

# Jan-Feb 2024, modified by Andi to generate pseudoclock with NIDAQmx counter.

#from user_devices.NI_DAQmx_counter_as_pseudoclock.counter.blacs_workers import get_clock_ticks
#from .counter.blacs_workers import get_clock_ticks
#from .blacs_workers import get_clock_ticks
from user_devices.iPCdev.runviewer_parsers import iPCdev_parser

# if True show all traces even if they are constant or zero
# if False show only traces which are changing or nonzero
SHOW_ALL = False

class NI_DAQmx_parser(iPCdev_parser):
    def __init__(self, path, device):
        super(NI_DAQmx_parser, self).__init__(path, device)

        if False:
            # this is called for all intermediate devices = NI boards
            self.path   = path
            self.name   = device.name
            self.device = device
            print("runviewer loading class '%s' device '%s'" % (device.device_class, device.name))

            # get dictionary with key = physical channel name, value = user given name
            self.channels = {}
            for child_name, child in device.child_list.items():
                self.channels[child.parent_port] = child_name
            #print(self.channels)
            # get dictionary of ports with 'num_lines' and 'supports_buffered'
            self.ports = device.properties['ports']
            #print(self.ports)

    def old_get_traces(self, add_trace, clock=None):
        traces = {}
        
        # times in seconds for device
        times  = clock[0]
        #print(times, times.shape)

        if len(times) > 0:
            with h5py.File(self.path, 'r') as f:

                # load data tables for analog and digital outputs
                group = f['devices/' + self.name]
                AO_table = {}
                try:
                    data = group['AO']
                    for name in data:
                        AO_table[name] = data[name][:]
                except KeyError:
                    AO_table = None

                DO_table = {}
                try:
                    data = group['DO']
                    for name in data:
                        DO_table[name] = data[name][:]
                except KeyError:
                    DO_table = None

            if AO_table is not None:
                #print(AO_table)
                for ao, values in AO_table.items():
                    # name = user given name and not the physical channel name, otherwise get strange errors.
                    name = self.channels[ao]
                    values = get_clock_ticks(times, values=values)
                    if np.any(values != values[0]): # dynamic trace
                        data = (times,values)
                    elif len(values) > 0: # constant trace
                        if not SHOW_ALL and np.all(values == 0.0): continue # skip constant 0 traces
                        data = (np.array([times[0],times[-1]]),np.array([values[0],values[0]]))
                    else: continue
                    add_trace(name, data, None, None)
                    traces[name] = data
                    
            if DO_table is not None:
                #print(DO_table)
                for port, values in DO_table.items():
                    # digital outputs are organized in ports
                    for i in range(self.ports[port]['num_lines']):
                        # name = user given name and not the physical channel name, otherwise get strange errors.
                        name = self.channels['%s/line%i'%(port,i)]
                        # get channel data and expand to match times
                        v = get_clock_ticks(times, values = (values>>i)&1)
                        if np.any(v != v[0]): # dynamic trace
                            data = (times,v)
                        elif len(v) > 0: # constant trace
                            if not SHOW_ALL and np.all(v == 0): continue # skip constant 0 traces
                            data = (np.array([times[0],times[-1]]),np.array([v[0],v[0]]))
                        else: continue
                        print('%i times mu ='%len(data[0]), data[0])
                        print('%i values   ='%len(data[1]), data[1])
                        add_trace(name, data, None, None)
                        traces[name] = data

        return traces
