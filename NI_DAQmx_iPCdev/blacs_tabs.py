#####################################################################
#                                                                   #
# /NI_DAQmx/blacs_tab.py                                            #
#                                                                   #
# Copyright 2018, Monash University, JQI, Christopher Billington    #
#                                                                   #
# This file is part of the module labscript_devices, in the         #
# labscript suite (see http://labscriptsuite.org), and is           #
# licensed under the Simplified BSD License. See the license.txt    #
# file in the root of the project for the full license.             #
#                                                                   #
#####################################################################
import labscript_utils.h5_lock
import h5py
from labscript_utils import dedent

#from .utils import split_conn_AO, split_conn_DO
from labscript_devices.NI_DAQmx.utils import split_conn_AO, split_conn_DO
import warnings

from user_devices.iPCdev.blacs_tabs import iPCdev_tab

# Jan-April 2024, modified by Andi to generate pseudoclock with NIDAQmx counter.
worker_path = 'user_devices.NI_DAQmx_iPCdev.blacs_workers.NI_DAQmx_OutputWorker'

# status monitor update time
UPDATE_TIME_MS = 250

from time import sleep

class NI_DAQmx_tab(iPCdev_tab):
    def initialise_GUI(self):
        # set shared clocklines between boards:
        # - displays all channels belonging to boards and not to clocklines.
        # - self.channels given to worker contains only channels belonging to board.
        # - self.clocklines given to worker contains the IM devices belonging to board.
        self.set_shared_clocklines(True)
        # set update time how often status_monitor is called
        self.set_update_time_ms(UPDATE_TIME_MS)
        # call super class
        super(NI_DAQmx_tab, self).initialise_GUI()

    def init_tab_and_worker(self):

        # get min and max voltage range
        AO_base_units = 'V'
        if self.device.properties['num_AO'] > 0:
            # note: 'AO_range' from connection_table or from labscript_devices.models CAPABILITIES
            AO_base_min, AO_base_max = self.device.properties['AO_range']
        else:
            AO_base_min, AO_base_max = None, None
        AO_base_step = 0.1
        AO_base_decimals = 3
        self.worker_args.update({
            'Vmin': AO_base_min,
            'Vmax': AO_base_max,
        })

        # TODO: wait monitor is not implemented and purpose and implementation unclear?
        # We only need a wait monitor worker if we are if fact the device withx the wait monitor input.
        with h5py.File(self.settings['connection_table'].filepath, 'r') as f:
            waits = f['waits']
            wait_acq_device = waits.attrs['wait_monitor_acquisition_device']
            wait_acq_connection = waits.attrs['wait_monitor_acquisition_connection']
            wait_timeout_device = waits.attrs['wait_monitor_timeout_device']
            wait_timeout_connection = waits.attrs['wait_monitor_timeout_connection']
            try:
                timeout_trigger_type = waits.attrs['wait_monitor_timeout_trigger_type']
            except KeyError:
                timeout_trigger_type = 'rising'
        self.worker_args.update({
            'wait_timeout_device': wait_timeout_device,
            'wait_timeout_connection': wait_timeout_connection,
            'wait_timeout_rearm_value': int(timeout_trigger_type == 'falling'),
        })

        # create worker
        # note: updated from parent class
        print('create worker', worker_path)
        self.create_worker(
            name        = self.primary_worker,
            WorkerClass = worker_path,
            workerargs  = self.worker_args,
        )

        # Set the capabilities of this device
        self.supports_remote_value_check(False)
        self.supports_smart_programming(True)
        
