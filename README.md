# labscript_NI_DAQmx_iPCdev
labscript-suite internal pseudoclock device for National Instruments DAQmx cards

This labscript-device uses the internal counters of the National Instruments cards PXIe-6738 as pseudoclock to trigger the analog and digital outputs of the same card and of other cards like the PXIe-6535 for additional digital outputs. Each of the PXIe-6738 cards can provide two counters as pseudoclocks. The counters can be shared with other cards.

For clocking the counters use always the interal 100MHz clock of the PXIe-6738 cards but which can be optionally locked to an external reference clock applied on one of the PFI ports. This was successfully tested with 10MHz. 

To synchronize several counters an external start trigger must be used.

The implementation builds on the [internal pseudoclock device](https://github.com/INO-quantum/labscript_iPCdev) as base class. Please copy the entire `iPCdev` and `NI_DAQmx_iPCdev` folders into your `user_devices` folder. [Here](https://github.com/INO-quantum/labscript_NI_DAQmx_iPCdev/tree/main/example_experiment) you find the `connection_table` and an example experiment script. 

Using the counters in this way is working but we have encountered issues when running the digital outputs with a faster output rate than 1-2MHz. Few short pulses at up to 10MHz are fine but running contiguously causes this error: `PyDAQmx.DAQmxFunctions.OutputFIFOUnderflow2Error: Onboard device memory underflow. Because of system and/or bus-bandwidth limitations, the driver could not write data to the device fast enough to keep up with the device output rate. Reduce your sample rate. If your data transfer method is interrupts, try using DMA or USB Bulk. You can also reduce the number of programs your computer is executing concurrently. Task Name: CounterTask__PXI1Slot3_ctr1`. 

We interpret this error, that the rather small FIFO of the counters of the PXIe-6738 card is getting empty during the execution of the experimental sequence. This can have two reasons: the data transmission rate of the chassis is not sufficient to update the FIFO fast enough. We use the chassis PXIe-1073 with only 250MB/s which is certainly not the best but according to our estimates should be sufficient. Second reason is the latency of the data transmission which is not specified by National Instruments. The FIFO has only 127 samples which means that there is not much margin for delays in the DMA data transmission to refill this buffer before it becomes empty. I would rather think this is the main reason for the error. We have tested several options regarding DMA but without success. Using a faster chassis might help but not sure and since the latency is not specified it is hard to judge. One should mention, that the latency depends on the entire system, i.e. also the control computer. Using counters with larger FIFO size could help but a fast survey of available counters by National Instruments has not shown any with larger size.

> [!IMPORTANT]
> In case you are planning to output data at rates >1MHz do not use this approach but use an external pseudoclock device!

We provide also a [C code](https://github.com/INO-quantum/labscript_NI_DAQmx_iPCdev/tree/main/C_code) which has the same functionality as the labscript/Python code. It was tested on Windows using Visual Studio and on Ubuntu with gcc. We used it for our initial development of the Python code and to test and understand better the above described error. It is much simpler than the rather involved labscript implementation where special care must be taken to synchronize the workers processes for the different boards.

> [!Note]
> This project was developed in collaboration with a different laboratory where NI hardware is used. I work with different hardware and therefore cannot perform additional tests with NI hardware. Most of this code was developed with simulated hardware and was then later tested and debugged on the real hardware.
