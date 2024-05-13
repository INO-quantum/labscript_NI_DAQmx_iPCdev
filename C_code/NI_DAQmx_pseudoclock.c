// DigPulseTrain-Cont-Buff-Implicit.c
// February - March 2024, by Andi, LENS and INO-CNR, University of Florence, Italy
// last modification 24/3/2024 by Andi
// modified ANSI C example code to implement programmable 'pseudoclock' to trigger analogue and digital outputs.

// general description:
// - the counters can be programmed to change its output state at arbitrary times.
//   this gives a programmable pattern of pulses on the output which we use as 'pseudoclock',
//   i.e. a not contiguous clock = a clock which can be halted. 
//   one can see this also as a programmable 'trigger' which triggers the output of other devices.
// - the analogue and digital outputs can use the counter output 'pseudoclock' as sample clock input
//   and output new samples on the 'pseudoclock' rising (or falling) edge.
// - this way the analogue and digital outputs need to be programmed only when the output should be changing.
//   this is contrary to the waveform mode where for each clock 'tick' a sample needs to be programmed,
//   regardless if the output changes or not.
//   this allows to ouput sequences with high output rate and intermittent long waiting times.

/**************************************************************************************************************/
// ATTENTION! 
// arbitrary analogue and digital patterns are created on the specified analogue channels and digital ports! 
// Do not attach physical devices on the used analogue channels and digital ports!
// each digital port comprises of up to 32 digital channels!
/**************************************************************************************************************/

// example implementation:
// - analogue and digital output samples are created for arbitrary number of devices and channels.
// - each of the devices has a counter associated. 
//   as long as the sample rate is the same, counters can be shared between several devices.
// - the example uses the internal 100MHz clock of the card hosting the counter as time reference. 
//   this is either free running, or is locked with a phase-locked-loop to an external clock (see next point).
// - an external clock input and frequency can be specified.
//   if several counters are used this is needed to synchronize the counters.
// - an external start trigger can be specified. 
//   if several counters are used this is needed to synchronize the counters.
// - either contiguous samples at a given sample rate are created for a given duration.
//   or for a given duration repeatedly contiguous samples at a given sample rate are created for t_full time,
//   followed by t_wait time without samples.
// - after the samples are created the channels and ports are programmed and the counters are started.
// - after the first counter has finished its output a 'done' signal is emitted and all outputs are stopped.
// - the number of output samples is evaluated for each device
// - this can be repeated for a given number of loops or indefinitely until user presses a key on the keyboard.

// hardware tested: 
// chassis PXIe-1073 with 250MB/s system bandwidth (we are limited by the bandwidth).
// 1x NI PXIe-6535 digital out (DO) card with 32 DO at 10MHz (PXI1Slot2)
// 2x NI PXIe-6738 analogue out (AO) cards with 2x2 independent counters and 2x8 AO at 1MHz or 2x32 AO at 400kHz (PXI1Slot3 and PXI1Slot4) 

// modified example from here:
// C:\Users\Public\Documents\National Instruments\NI-DAQ\Examples\DAQmx ANSI C\
// header and library is here:
// C:\Program Files (x86)\National Instruments\NI-DAQ\DAQmx ANSI C Dev\
// VS finds header here:
// C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\include\NIDAQmx.h

// include NI header file. if not found try commandline (-I<path> with gcc)
#include <NIDAQmx.h>

// if defined simulates NIDAQmx commands when is not installed. NIDAQmx.h is still needed.
//#define SIMULATE

#ifdef _MSC_VER
// include library for Visual Studio
// other compilers: include in commandline (-l<path> and -LNIDAQmx with gcc)
#pragma comment(lib,"NIDAQmx.lib")
#endif

#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <stdint.h>     // int32_t and uint32_t

#if defined(_WIN32) || defined(_WIN64) 
// Windows
#include <conio.h>		// _kbhit macro. windows specific.

#else
// not Windows

#include <sys/ioctl.h>
#include <termios.h>
int _kbhit()
{
    int byteswaiting;
    struct termios term, term2;
    tcgetattr(0, &term);

    term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

#endif

#ifdef SIMULATE
int32 __CFUNC DAQmxCreateTask(const char taskName[], TaskHandle *taskHandle) { *taskHandle = (TaskHandle)1; return 0; }
#define DAQmxStartTask(taskHandle)  0
#define DAQmxWaitUntilTaskDone(taskHandle, timeToWait)  0
#define DAQmxStopTask(taskHandle)   0
#define DAQmxClearTask(taskHandle)  0
#define DAQmxRegisterDoneEvent(task, options, callbackFunction, callbackData)   0
#define DAQmxCreateAOVoltageChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, customScaleName)    0
#define DAQmxCreateDOChan(taskHandle, lines, nameToAssignToLines, lineGrouping) 0
#define DAQmxCreateCOPulseChanTicks(taskHandle, counter, nameToAssignToChannel, sourceTerminal, idleState, initialDelay, lowTicks, highTicks) 0
#define DAQmxSetRefClkSrc(taskHandle, data)     0
#define DAQmxSetRefClkRate(taskHandle, data)    0
#define DAQmxCfgDigEdgeStartTrig(taskHandle, triggerSource, triggerEdge)    0
#define DAQmxCfgImplicitTiming(taskHandle, sampleMode, sampsPerChan)    0
#define DAQmxCfgSampClkTiming(taskHandle, source, rate, activeEdge, sampleMode, sampsPerChan)   0
#define DAQmxWriteCtrTicks(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, highTicks, lowTicks, numSampsPerChanWritten, reserved)    0
#define DAQmxWriteAnalogF64(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)     0
#define DAQmxWriteDigitalU32(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)    0
int32 __CFUNC DAQmxGetCOPulseTerm(TaskHandle taskHandle, const char channel[], char *data, uInt32 bufferSize) {
    snprintf(data, bufferSize, "%s/sim_port", channel);
    return 0;
}
#define DAQmxSetAODataXferMech(taskHandle, channel, data)       0
#define DAQmxSetAODataXferReqCond(taskHandle, channel, data)    0
#define DAQmxSetDODataXferMech(taskHandle, channel, data)       0
#define DAQmxSetDODataXferReqCond(taskHandle, channel, data)    0
#define DAQmxSetCODataXferMech(taskHandle, channel, data)       0
#define DAQmxSetCODataXferReqCond(taskHandle, channel, data)    0
#define DAQmxGetWriteCurrWritePos(taskHandle, data)             0
#define DAQmxGetWriteTotalSampPerChanGenerated(taskHandle, data)    0
#define DAQmxGetExtendedErrorInfo(errorString, bufferSize)      0
#endif // SIMULATE

// number of loops. <=0: infinite
#define NUM_LOOPS       1

// if defined prints data up to this number of samples
#define PRINTDATA		250

// external clock
// note: if several counters are used this is needed to synchronize the counters!
#define EXT_CLOCK		"/PXI1Slot3/PFI4"		// define to set external clock
#define EXT_CLOCK_RATE	10e6

// define to set external clock also for AO and DO channels. needed, otherwise get error.
#define LOCK_REFCLOCK_AO_DO

// external trigger
// note: if several counters are used this is needed to synchronize the counters!
#define EXT_TRIG		"/PXI1Slot3/PFI0"		// define to set external start trigger

// list of counter names
#define NUM_COUNTER     4                       // number of counters used
#define COUNTER	        {"/PXI1Slot3/ctr0","/PXI1Slot4/ctr0","/PXI1Slot3/ctr1","/PXI1Slot4/ctr1"}
#define COUNTER_RATE	{10e6,10e6,10e6,10e6}	// counter rate when no AO or DO is assigned

// list with analog channel names for each device. 
// if NUM_AO_DEVICES == 0 no analog output channel is used.
// for 1MHz rate use every 4th channel (max 8 channels), for 400kHz rate you can use all 32 channels.
// NUM_AO = number of output channels per device
#define NUM_AO_DEVICES  2
#define NUM_AO          8
#define AO_CHANNELS     {{"/PXI1Slot3/ao0" ,"/PXI1Slot3/ao4","/PXI1Slot3/ao8" ,"/PXI1Slot3/ao12",\
                          "/PXI1Slot3/ao16","/PXI1Slot3/ao20","/PXI1Slot3/ao24","/PXI1Slot3/ao28"},\
                         {"/PXI1Slot4/ao0" ,"/PXI1Slot4/ao4" ,"/PXI1Slot4/ao8" ,"/PXI1Slot4/ao12",\
                          "/PXI1Slot4/ao16","/PXI1Slot4/ao20","/PXI1Slot4/ao24","/PXI1Slot4/ao28"}}
#define AO_RATE			{1.0e6,1.0e6}		// output rate per AO device
#define AO_CTR		    {0,1}			    // counter index per AO device

// list with digital output port name for each device. this uses all channels of the port.
// if NUM_DO_DEVICES == 0 no digital output channel is used.
// NUM_DO = number of output ports per device
#define NUM_DO_DEVICES  3
#define NUM_DO          1
#define DO_PORTS        {{"/PXI1Slot2/port0"},{"/PXI1Slot3/port0"},{"/PXI1Slot4/port0"}}
#define DO_RATE			{10e6,10.0e6, 10e6}     // output rate per DO device
#define DO_CTR		    {2,3,3}					// counter index per DO device

#define TIME_MIN		60e-9
#define TICKS_MIN		3		// 3 for 100MHz, 2 for 10MHz

// internal clock rate. must be 100e6.
#define INT_CLOCK_RATE	100e6

// timing settings
#define DURATION		1.0     				// total time in seconds
#define T_START	        0.0						// start time
#define T_FULL	        110*(1.0/20.0e6)		// if >0 full data rate time in seconds
#define T_WAIT	        3000*(1.0/10.0e6)		// if >0 wait time in seconds after T_FULL

// analog voltage ramps (double)
#define VOLTAGE_RATE    10.0					// ramp rate in V/s (<0 if should ramp down initially)
#define VOLTAGE_INIT	0.0						// first ramp start voltage = last ramp end voltage
#define VOLTAGE_HIGH	10.0					// high voltage
#define VOLTAGE_LOW		-10.0					// low voltage
#define VOLTAGE_RES		(20.0/(double)(1<<16))  // voltage resolution

// digital pulses pattern (uint32_t)
#define PATTERN_STEP    0x01010101				// increment per step
#define PATTERN_INIT	0x04030201				// initial pattern
#define PATTERN_HIGH	0x100f0e0d		        // pattern where PATTERN_STEP is decremented
#define PATTERN_LOW		0x04030201			    // pattern where PATTERN_STEP is incremented

// write timeout in seconds
#define WRITE_TIMEOUT	10.0

// memory transfer settings. comment to use default.
// for MODE: DAQmx_Val_Interrupts or DAQmx_Val_DMA
//           DAQmx_Val_ProgrammedIO and DAQmx_Val_USBbulk are not supported.
// for COND: DAQmx_Val_OnBrdMemEmpty or DAQmx_Val_OnBrdMemNotFull or DAQmx_Val_OnBrdMemHalfFullOrLess
//           counter (CO) supports only: DAQmx_Val_OnBrdMemNotFull
//           digital (DO) support only: DAQmx_Val_OnBrdMemNotFull, DAQmx_Val_OnBrdMemHalfFullOrLess
//			 analog  (AO) supports all
// for PXIe-6535 these functions are not supported. we use has_counter to check this.
//               this works but might not be the right way for all types of boards.
#define MEM_XFER_MODE_CO	DAQmx_Val_DMA
#define MEM_XFER_COND_CO	DAQmx_Val_OnBrdMemNotFull
#define MEM_XFER_MODE_AO	DAQmx_Val_DMA
#define MEM_XFER_COND_AO	DAQmx_Val_OnBrdMemNotFull
#define MEM_XFER_MODE_DO	DAQmx_Val_DMA
#define MEM_XFER_COND_DO	DAQmx_Val_OnBrdMemNotFull

// simple error checking. requires int error and Error label.
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// counter names, output channels and rates
// rates are overwritten by AO and DO channel rate unless no AO or DO is assigned.
// assigned is 1 if counter is used by an AO or DO channel.
// if counter is not assigned times_co for counter are created with COUNTER_RATE.
#define COUNTER_BUFSIZE		128
static char * counter     [NUM_COUNTER] = COUNTER;
static char * counter_pfi [NUM_COUNTER] = {0};
static double counter_rate[NUM_COUNTER] = COUNTER_RATE;
static unsigned char counter_used[NUM_COUNTER] = {0};

// AO channel names and counter indices
#if NUM_AO_DEVICES > 0
static char * ao_channel[NUM_AO_DEVICES][NUM_AO] = AO_CHANNELS;
static int    ao_ctr    [NUM_AO_DEVICES]         = AO_CTR;
static double ao_rate   [NUM_COUNTER]			 = AO_RATE;
#endif

// DO channel names and counter indices
#if NUM_DO_DEVICES > 0
static char * do_port[NUM_DO_DEVICES][NUM_DO] = DO_PORTS;
static int    do_ctr [NUM_DO_DEVICES]         = DO_CTR;
static double do_rate[NUM_COUNTER]			  = DO_RATE;
#endif

typedef union {
	double		real;
	uint32_t	uint;
	int32_t		_int;
} varg;

static varg V_rate = { .real = VOLTAGE_RATE };
static varg V_init = { .real = VOLTAGE_INIT };
static varg V_low  = { .real = VOLTAGE_LOW  };
static varg V_high = { .real = VOLTAGE_HIGH };

static varg P_rate = { ._int = PATTERN_STEP };
static varg P_init = { .uint = PATTERN_INIT };
static varg P_low  = { .uint = PATTERN_LOW  };
static varg P_high = { .uint = PATTERN_HIGH };

#define BUFSIZE 1024
char buffer[BUFSIZE] = { 0 };

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData);

// create analog or digital output samples with given duration and sample rate.
// for analog output creates triangular signal from initial to max/min voltages with given ramp rate.
// for digital output creates digital pulse pattern from initial incrementing with ramp_rate and inverted with max/min.
// if no error returns number of samples >0 and pointer to times and values. on error returns 0.
// duration      = total time in seconds. last time is truncated to next lower integer multiple of 1/sample_rate.
// sample_rate   = sample rate in Hz
// num_channels  = number of channels, i.e. number the value is copied per sample (use DAQmx_Val_GroupByScanNumber)
// is_analog     = if != 0 analog ramp is created, otherwise digital pulses
// initial       = initial analog voltage or initial digital sample pattern
// min           = minimum analog voltage or digital pattern where to inverse direction
// max           = maximum analog voltate or digital pattern where to inverse direction
// ramp_rate     = analog voltage ramp rate in V/s or digital pattern increment per sample
// times         = returned pointer to array of allocated times in seconds (double). use free to release. cannot be NULL;
// values        = returned pointer to array of allocated values in Volts (double) or digital states (uInt32). use free to release. can be NULL.
// t_start       = start time in seconds
// t_full,t_wait = if t_full>0 and t_wait>0 repeatedly creates samples for t_full seconds, then waits for t_wait seconds without samples.
//                 use this to test high sample rates for short time t_full where the onboard FIFO memory is emptied,
//                 followed by sufficient time t_wait to fill again the FIFO.
//                 this way higher samples rates can be reached which would not be possible in contiguous output mode.
//				   t_full and t_wait are rounded to the nearest integer multiple of 1/sample_rate, or 1 if round would give 0.
size_t create_samples(	double duration, double sample_rate,
                        int num_channels, unsigned char is_analog,
                        varg initial, varg min, varg max, varg ramp_rate,
                        double **times, void **values,
                        double t_start, double t_full, double t_wait) {
	int k;
	double *t, value = initial.real, resolution = VOLTAGE_RES, tmp;
	size_t i, num_samples, num_stop = 0, tk, tk_stop, tk_full, tk_wait = 0, tk_add = 0, tk_rest = 0;
	uint32_t port = initial.uint;
	union { double* p_double; uint32_t* p_uint32; } v;

    // setup waiting time
	num_samples = (size_t)floor(duration * sample_rate); // truncated to nearest integer
	if ( (t_full > 0.0) && (t_wait > 0.0) ) {
	    // use waiting time
		tk_full     = (size_t) round(t_full * sample_rate);
		if (tk_full == 0) tk_full = 1;
		tk_wait     = (size_t) round(t_wait * sample_rate);
		if (tk_wait == 0) tk_wait = 1;
		tk_add      = tk_full + tk_wait;
	    tk_stop     = tk_full; // next stop ticks
		num_stop    = (size_t) floor(((double)num_samples) / tk_add);
		tk_rest     = num_samples % tk_add; // rest
		num_samples = num_stop * tk_full;
		if (tk_rest >= tk_full) num_samples += tk_full + 1;
		else                    num_samples += tk_rest + 1;
		printf("sample rate %.3f MHz, duration %.3es, full %.3fns (%lu), wait %.3fns (%lu), # samples %lu, # stop %lu, rest %lu\n", 
		    sample_rate / 1e6, 
		    duration, 
		    t_full*1e9, tk_full, 
		    t_wait*1e9, tk_wait, 
		    num_samples, 
		    num_stop, 
		    tk_rest);
	}
	else {
	    // do not use waiting time (set after t_start + duration)
	    tk_stop     = num_samples + 1;
	}
	
	// allocate times and values
	t = *times = (double*) malloc(num_samples * sizeof(double));
	if (t) {
	    if (values != NULL) {
	        if (is_analog) {
		        *values = malloc(num_samples * num_channels * sizeof(double));
		        v.p_double = (double *) *values;
		    }
		    else {
		        *values = malloc(num_samples * num_channels * sizeof(uint32_t));
		        v.p_uint32 = (uint32_t *) *values;
		    }
		}
		if ((values == NULL) || (v.p_double != NULL)) {

		    // create samples
			// time is calculated in ticks to avoid numerical errors
			if (is_analog) {
				ramp_rate.real = ramp_rate.real / sample_rate; // V/sample
			}
		    for (i = tk = 0; i < num_samples; ++i, ++tk) {
		        // time (t += dt)
			    if (tk >= tk_stop) {
					if (i == (num_samples - 1)) { 
						// last sample
						tk = num_stop * tk_add + tk_rest;
					}
					else {
						tk += tk_wait;
						tk_stop += tk_add;
					}
			    }
			    *t++ = t_start + (((double)tk)/sample_rate);
			    if (values != NULL) {
			        if (is_analog) {
			            // create triangular signal
				        for (k = 0; k < num_channels; ++k) {
					        *v.p_double++ = value;
				        }
						value = round((value + ramp_rate.real)/resolution)*resolution;
						if (value >= max.real) {
							value = max.real;
							ramp_rate.real = -ramp_rate.real;
						}
						else if (value <= min.real) {
							value = min.real;
							ramp_rate.real = -ramp_rate.real;
						}
					}
				    else {
				        // create digital pattern
				        for (k = 0; k < num_channels; ++k) {
        			        *v.p_uint32++ = port;
				        }
						port += ramp_rate.uint;
						if ( (port == max.uint) || (port == min.uint) ) {
							ramp_rate.uint = ~ramp_rate.uint + 1;
						}
					}
			    }
		    }
		    
#ifdef _DEBUG
			if (values != NULL) {
				// ensure we have generated the right number of samples
				if (is_analog) {
					if (v.p_double != ((double*)(*values)) + num_samples * num_channels) {
						printf("create_samples error: unexpected pointer %p != %p! this is a bug.", v.p_double, ((double*)(*values)) + num_samples * num_channels);
						goto Error;
					}
				}
				else {
					if (v.p_uint32 != ((uint32_t*)(*values)) + num_samples * num_channels) {
						printf("create_samples error: unexpected pointer %p != %p! this is a bug.", v.p_uint32, ((uint32_t*)(*values)) + num_samples * num_channels);
						goto Error;
					}
				}
			}
#endif
			if (values != NULL) {
				// set the last sample to the initial value
				if (is_analog) {
					v.p_double = (double*)*values;
					v.p_double += (num_samples - 1)*num_channels;
					for (k = 0; k < num_channels; ++k) {
						*v.p_double++ = initial.real;
					}
				}
				else {
					v.p_uint32 = (uint32_t*)*values;
					v.p_uint32 += (num_samples - 1)*num_channels;
					for (k = 0; k < num_channels; ++k) {
						*v.p_uint32++ = initial.uint;
					}
				}
			}

#ifdef PRINTDATA
			if ( num_samples <= PRINTDATA ) {
				// print samples for testing
				if (values == NULL) {
					printf("\n%lu samples with %.3f MHz (counter only):", num_samples, sample_rate / 1e6);
					printf("\n%6s: %8s", "sample", "t(mu)");
					t = *times;
					for (i = 0; i < num_samples; ++i, ++t) {
						printf("\n%6lu: %8.3f", i, (*t) * 1e6);
					}
					printf("\n\n");
				}
				else if (is_analog) {
					printf("\n%lu samples on %i AO channels with %.3f MHz:", num_samples, num_channels, sample_rate/1e6);
					printf("\n%6s: %8s", "sample", "t(mu)");
					for (k = 0; k < num_channels; ++k) {
						printf(" %8s", "value");
					}
					t = *times;
					v.p_double = (*values);
					for (i = 0; i < num_samples; ++i, ++t) {
						printf("\n%6lu: %8.3f", i, (*t) * 1e6);
						for (k = 0; k < num_channels; ++k, ++v.p_double) {
							printf(" %8.3f", *v.p_double);
						}
					}
					printf("\n\n");
				}
				else {
					printf("\n%lu samples on %i DO channels with %.3f MHz:\n", num_samples, num_channels, sample_rate / 1e6);
					//printf("\n%i, %u, %x", ramp_rate._int, ramp_rate.uint, PATTERN_HIGH);
					printf("\n%6s: %8s", "sample", "t(mu)");
					for (k = 0; k < num_channels; ++k) {
						printf(" %8s", "port");
					}
					t = *times;
					v.p_uint32 = (*values);
					for (i = 0; i < num_samples; ++i, ++t) {
						printf("\n%6lu: %8.3f", i, (*t) * 1e6);
						for (k = 0; k < num_channels; ++k, ++v.p_uint32) {
							printf(" %08x", *v.p_uint32);
						}
					}
					printf("\n\n");
				}
				printf("press <Enter> to continue ...\n");
				getchar();
			}
#endif

#ifdef _DEBUG
			// ensure last time is next lower integer multiple of 1/sample_rate
			value = (*times)[num_samples - 1];
			tmp = duration * sample_rate;
			if (fabs(round(tmp) - tmp) <= 1e-10) tmp = round(tmp) / sample_rate; // near or equal integer multiple
			else                                 tmp = floor(tmp) / sample_rate; // next lower integer
			if (fabs(value - tmp) >= 1e-10) {
				printf("create_samples: last time %.3es must be %.3es \n", value, tmp);
				printf("rate %.3fMHz, dt = %.3es, duration = %.3es, error = %.3es\n", sample_rate/1e6, 1.0 / sample_rate, duration, fabs(value - tmp));
				goto Error;
			}
#endif

		    // return number of samples
		    return num_samples;
		}
	}
	
Error:
	// an error occurred
	if(*times) {
		free(*times);
		*times = NULL;
	}
	if (values) {
		if (*values) {
			free(*values);
			*values = NULL;
		}
	}

	// return 0 = error
	return 0;
}

// print one pulse repeptition = repetition of same t_low and t_high pulses
void print_pulse_rep(size_t start, size_t i, size_t low, size_t high, double clock_rate, char *line_end) {
    double duration = (double)((i-start)*(low+high))/clock_rate;
    printf("%6lu - %6lu: %9lu %9lu %9lu %9lu %12.6f %9lu ", start, i,
    low, high, (size_t)round(((double)low/clock_rate)*1e9), (size_t)round(((double)high/clock_rate)*1e9), 
    clock_rate/((double)(low+high))*1e-6, 
    i - start);
    if      ( duration < 1e-6 ) printf("%12lu ns" , (size_t)round(duration*1e9));
    else if ( duration < 1e-3 ) printf("%12.3f us", duration*1e6);
    else if ( duration < 1.0  ) printf("%12.6f ms", duration*1e3);
    else                        printf("%12.9f s " , duration);
    printf("%s", line_end);
}

// sequence of pulses saved as double-linked list
struct pulses {
    uint32_t start;                 // first index of same pulses
    uint32_t num_pulses;            // number of same pulses
    uint32_t low;                   // low ticks
    uint32_t high;                  // high ticks
    struct pulses *next;            // next pulse sequence or NULL
    struct pulses *prev;            // previous pulse sequence or NULL
    uint32_t num_prev;              // number of pulse sequences to go backward/forward
    uint32_t num_seq;               // if >0 number of pulse sequences to generate in a loop from num_prev to actual one.
};

// print sequence of repeated pulses
// TODO: not sure if the unicode characters are printed correctly on windows?
void print_pulse_seq(struct pulses *first, double clock_rate) {
    struct pulses *next;
    uint32_t steps = 0;
    while(first) {
        next = first->next;
        if ((first->num_prev > steps) && (first->num_seq == 0)) steps = first->num_prev;
        print_pulse_rep(first->start, first->start+first->num_pulses, first->low, first->high, clock_rate, 
            ((steps != 0) || (first->num_seq > 0)) ? "" : "\n");
        if (first->num_seq > 0) {
            if (steps == 0) {
                printf("─┘ x%u + %u (-%u)\n", first->num_seq/(first->num_prev+1), first->num_seq%(first->num_prev+1), first->num_prev);
            }
            else {
                printf("─┤ x%u + %u (-%u)\n", first->num_seq/(first->num_prev+1), first->num_seq%(first->num_prev+1), first->num_prev);
                --steps;
            }   
        }
        else if (steps != 0) {
            if (first->num_prev == steps) printf("─┐ (+%u)\n", first->num_prev);
            else if (first->num_prev > 0) printf("─┤ (+%u)\n", first->num_prev);
            else                          printf(" │\n");
            --steps;
        }
        free(first);
        first = next;
    }
}

// get pulse sequences and pulse repetitions in ticks
// with show_seq == 0 does not print repetitions of pulses (simpler, but more output)
// with show_seq != 0 prints repetitions of different pulses sequences (more complex, less output)
void analyse_pulse_sequences(uint32_t *ticks, size_t num_times, double clock_rate, int show_seq) {
 	size_t i, start, low, high;
	struct pulses *first = NULL, *act = NULL, *last = NULL;
	uint32_t *p, num_prev;
	
	if (show_seq) printf("\npulse sequence with %lu ticks (%lu times) with loops:\n", num_times + 1, num_times);
	else          printf("\npulse sequence with %lu ticks (%lu times):\n", num_times + 1, num_times);
    printf("%6s - %6s  %9s %9s %9s %9s %12s %9s %15s %s\n", "index", "start", "low (tk)", "high (tk)", "low (ns)", "high (ns)", "f (MHz)", "reps", "duration", show_seq ? " loops + rest (jump)" : "");

    p = ticks;
    low   = *p;
    high  = *(p+1); 
    start = 0;
    ++p;

    for (i = 1; ; ++i, ++p) {
        if ( (i == num_times) || (*p != low) || (*(p+1) != high) ) {
            // pulses have changed
            if (!show_seq) print_pulse_rep(start, i, low, high, clock_rate, "\n");
            else {
                // check if next pulses in sequence match
                if ( (act != NULL) && (act->num_pulses == (i-start)) && (act->low == low) && (act->high == high) ) {
                    // match: expand last sequence with one more pulses
                    last->num_seq++;
                    //printf("%4lu increase seq last: start %6u low %6u high %6u rep %6u seq %6u prev %6u\n", i, last->start, last->low, last->high, last->num_pulses, last->num_seq, last->num_prev);
                    //printf("                  act : start %6u low %6u high %6u rep %6u seq %6u prev %6u\n", act->start, act->low, act->high, act->num_pulses, act->num_seq, act->num_prev);
                    // set act to next pulses. if act == last set back by num_prev.
                    if (act == last) { 
                        num_prev = 0;
                        while (num_prev < last->num_prev) {
                            act = act->prev;
                            ++num_prev;
                        }
                    }
                    else act = act->next;
                }
                else {
                    // backward search for same pulse sequence
                    act = last;
                    num_prev = 0;
                    while (act) {
                        if ( (act->num_pulses == (i-start)) && (act->low == low) && (act->high == high) ) {
                            // same found
                            last->num_prev = num_prev;
                            last->num_seq++;
                            act->num_prev = (num_prev > act->num_prev) ? num_prev : act->num_prev;
                            //printf("%4lu same found   last: start %6u low %6u high %6u rep %6u seq %6u prev %6u\n", i, last->start, last->low, last->high, last->num_pulses, last->num_seq, last->num_prev);
                            //printf("                  act : start %6u low %6u high %6u rep %6u seq %6u prev %6u\n", act->start, act->low, act->high, act->num_pulses, act->num_seq, act->num_prev);
                            // set act to next pulses 
                            act = act->next;
                            break;
                        }
                        act = act->prev;
                        num_prev++;
                    }
                    if ( act == NULL ) {
                        // not found: add new pulse sequence
                        act = malloc(sizeof(struct pulses));
                        act->prev       = last;
                        act->next       = NULL;
                        act->start      = start;
                        act->num_pulses = i-start;
                        act->low        = low;
                        act->high       = high;
                        act->num_prev = act->num_seq = 0;
                        if (first == NULL) first = last = act;
                        else               last = last->next = act;
                        //printf("%4lu add pulse        : start %6u low %6u high %6u rep %6u seq %6u prev %6u\n", i, act->start, act->low, act->high, act->num_pulses, act->num_seq, act->num_prev);
                        // no active sequence
                        act = NULL;
                    }
                }
            }
            if (i >= num_times) break;
            // start new pulse repetition
            start = i;
            low   = *p;
            high  = *(p+1); 
        }
    }
    if (show_seq) print_pulse_seq(first, clock_rate);
    printf("\n");
}

// program low and high ticks for given times in seconds and clock rate in Hz
// returns 0 if ok, otherwise error.
int write_ticks(TaskHandle task_handle, double clock_rate, double* times, size_t num_times) {
	int error = -1;
	size_t i;
	//size_t start, low, high;
	uInt32 *ticks, *p, tmp;
	double last_time, clock_rate_half = clock_rate/2.0;
	// allocate low and high times (since we use 50 : 50 on/off ratio this can be a single array)
	p = ticks = malloc((num_times + 1) * sizeof(uInt32));
	if (ticks) {
		printf("times      [%8.6f, %8.6f, %8.6f ... %8.6f, %8.6f, %8.6f]\n", times[0], times[1], times[2], times[num_times - 3], times[num_times - 2], times[num_times-1]);
		// first low time = last hight time = minimum time
		*ticks = *(ticks+num_times) = TICKS_MIN;
		last_time = *times;
			
		// save time differences / 2 in units of ticks = 1/clock_rate (10ns at 100MHz)
		p = ticks + 1;
		times += 1;
		for (i = 1; i < num_times; ++i, ++times) {
			tmp = ((uInt32)round((*times - last_time) * clock_rate_half));
			*p++ = (tmp > TICKS_MIN) ? tmp : TICKS_MIN;
			last_time = *times;
		}
				
		// write low and high ticks
		if (DAQmxFailed(error = DAQmxWriteCtrTicks(task_handle, num_times, FALSE, WRITE_TIMEOUT, DAQmx_Val_GroupByChannel, ticks + 1, ticks, NULL, NULL))) {
			// write error
			printf("DAQmxWriteCtrTicks %lu times error %d\n", num_times, error);
		}
		else {
			// write ok
			printf("DAQmxWriteCtrTicks %lu times ok\n", num_times);
			
			if (num_times <= 500) {
			    printf("\n%lu ticks (%lu times):", num_times+1, num_times);
			    p = ticks;
			    for (i = 0; i <= num_times; ++i, ++p) {
			        if ((i%20) == 0) printf("\n%6lu: %6u", i, *p);
			        else             printf(" %6u", *p);
			    }
			    printf("\n");
			}
			else {
			    p = ticks;
			    printf("\nticks_low  [%8u, %8u, %8u ... %8u, %8u, %8u]\n", p[0], p[1], p[2], p[num_times - 3], p[num_times - 2], p[num_times - 1]);
			    p = ticks + 1;
			    printf("ticks_high [%8u, %8u, %8u ... %8u, %8u, %8u]\n", p[0], p[1], p[2], p[num_times - 3], p[num_times - 2], p[num_times - 1]);
			}

		    // print pulse sequences
		    if (num_times <= 500) analyse_pulse_sequences(ticks, num_times, clock_rate, 0);
		    analyse_pulse_sequences(ticks, num_times, clock_rate, 1);
		}

		// free buffer
		free(ticks);
	}
	return error;
}

// configure and write AO,DO or counter task with given number of samples
// if ok returns TaskHandle, otherwise 0
#define AO_TASK     0
#define DO_TASK     1
#define CO_TASK     2
TaskHandle write_task(char **channels, int num_channels, size_t num_samples, void* values, int type, int device_index, int *error_out) {
	TaskHandle taskHandle = 0;
	int error = 0, i, ctr_index, has_counter;
	int32 written = 0;
	char* ptr;

	if (values == NULL) {
		printf("\nwrite_task type %i device %i called with values = NULL!\n", type, device_index);
		error = -20;
		goto Error;
	}

	// determine if board has counters.
	// we first copy the device name into buffer without initial '/' until '/' of following channel or port name
	ptr = (channels[0][0] == '/') ? channels[0] + 1 : channels[0];
	for (i = 0; (ptr[i] != '/') && (ptr[i] != '\0'); ++i) buffer[i] = ptr[i];
	buffer[i] = '\0';
	// instead of getting all counter names we can just ask how big is the required buffer size. if it is 0 there is no counter.
	//DAQmxErrChk(DAQmxGetDevCOPhysicalChans(buffer, buffer + i + 1, BUFSIZE - i - 1));
	//if (buffer[i + 1] == '\0') {
	has_counter = DAQmxGetDevCOPhysicalChans(buffer, NULL, 0);	// 0 = no counter, >0 has counter. does not give the number of counters.
	
	if (type == AO_TASK) {
	    // create analog ouput task for all channels
		printf("\nAO device %i create task\n", device_index);
	    DAQmxErrChk(DAQmxCreateTask(NULL, &taskHandle));
		for (i = 0; i < num_channels; ++i) {
			DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandle,
				channels[i],
				"",
				VOLTAGE_LOW,
				VOLTAGE_HIGH,
				DAQmx_Val_Volts,
				NULL
			));
#ifdef MEM_XFER_MODE_AO
			if (has_counter) {
				DAQmxErrChk(DAQmxSetAODataXferMech(taskHandle, channels[i], MEM_XFER_MODE_AO));
			}
#endif
#ifdef MEM_XFER_COND_AO
			if (has_counter) {
				DAQmxErrChk(DAQmxSetAODataXferReqCond(taskHandle, channels[i], MEM_XFER_COND_AO));
			}
#endif
		}
	}
	else if (type == DO_TASK){
	    // create digital ouput task for all ports
		printf("\nDO device %i create task\n", device_index);
	    DAQmxErrChk(DAQmxCreateTask(NULL, &taskHandle));
		for (i = 0; i < num_channels; ++i) {
			DAQmxErrChk(DAQmxCreateDOChan(taskHandle,
				channels[i],
				"",
				DAQmx_Val_ChanForAllLines // DAQmx_Val_ChanForAllLines or DAQmx_Val_ChanPerLine
			));
#ifdef MEM_XFER_MODE_DO
			if (has_counter) {
				DAQmxErrChk(DAQmxSetDODataXferMech(taskHandle, channels[i], MEM_XFER_MODE_DO));
			}
#endif
#ifdef MEM_XFER_COND_DO
			if (has_counter) {
				DAQmxErrChk(DAQmxSetDODataXferReqCond(taskHandle, channels[i], MEM_XFER_COND_DO));
			}
#endif
		}
	}
	else {
		// create counter task as timer
		// check if counter existing
		if (!has_counter) {
			printf("\nCO device %i has no counters!\n", device_index);
			error = -21;
			goto Error;
		}
		// num_channels must be 1 and device_index must be set correctly
		if ((num_channels != 1) || (channels[0] != counter[device_index])) {
			printf("\nCO device %i error! num channels %i != 1, %s != %s\n", device_index, num_channels, channels[0], counter[device_index]);
			error = -22;
			goto Error;
		}
		DAQmxErrChk(DAQmxCreateTask(NULL, &taskHandle));
		for (i = 0; i < num_channels; ++i) {
			printf("\nCO device %i create counter %s task\n", device_index, channels[i]);
			DAQmxErrChk(DAQmxCreateCOPulseChanTicks(taskHandle,
				channels[i], "",
				NULL,
				DAQmx_Val_Low,
				0,
				TICKS_MIN, TICKS_MIN
			));
#ifdef MEM_XFER_MODE_CO
			DAQmxErrChk(DAQmxSetCODataXferMech(taskHandle, channels[i], MEM_XFER_MODE_CO));
#endif
#ifdef MEM_XFER_COND_CO
			DAQmxErrChk(DAQmxSetCODataXferReqCond(taskHandle, channels[i], MEM_XFER_COND_CO));
#endif
		}
	}

#ifdef EXT_CLOCK
	// note: 
	// - this has to be called for ALL tasks on the same board! otherwise get this error:
	//   DAQmx Error -89137: Specified route cannot be satisfied, because it requires resources that are currently in use by another route.
    //   Property: DAQmx_RefClk_Src
	// - but if this is called for device without counters get this error:
	//   DAQmx Error -200452: Specified property is not supported by the device or is not applicable to the task.
	//   Property: DAQmx_RefClk_Src
	// - so we read the counter channels for device (without initial '/') and if returns empty buffer has no counter and can skip,
	//   otherwise set RefClkSrc and Rate.
#ifdef LOCK_REFCLOCK_AO_DO // do this for all tasks except for board without counters
	{
#else
	if (type == CO_TASK) { // do this only for counter task
#endif
		printf("%s device %i setup external clock %s", (type == AO_TASK) ? "AO" : (type == DO_TASK) ? "DO" : "CO", device_index, EXT_CLOCK);
		if (!has_counter) {
			printf(" (skip for board %s)\n", buffer);
		}
		else {
			printf("\n");
			DAQmxErrChk(DAQmxSetRefClkSrc(taskHandle, EXT_CLOCK));
			DAQmxErrChk(DAQmxSetRefClkRate(taskHandle, EXT_CLOCK_RATE));
		}
	}
#endif // EXT_CLOCK

    if (type == CO_TASK) {
    
#ifdef EXT_TRIG
	    // external trigger
	    // note: to better synchronize the counters with not clean start trigger:
	    //       - use one primary counter which triggers on external trigger
	    //       - other secondary counters trigger on "<CO_primary>/StartTrigger"
	    // see:  https://www.ni.com/docs/en-US/bundle/daqhelp/page/syncstarttrigger.html
		printf("CO device %i setup external trigger %s\n", device_index, EXT_TRIG);
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(taskHandle, EXT_TRIG, DAQmx_Val_Rising));
#endif

		// configure implicit timing with finite samples
		printf("CO device %i setup implicit timing\n", device_index);
		DAQmxErrChk(DAQmxCfgImplicitTiming(taskHandle, DAQmx_Val_FiniteSamps, num_samples));

		// write times
		printf("CO device %i write %lu samples for %i channels\n", device_index, num_samples, num_channels);
		printf("            first time %.3e, last time %.3e\n", ((double*)values)[0], ((double*)values)[num_samples - 1]);
		if ((error = write_ticks(taskHandle, INT_CLOCK_RATE, (double*)values, num_samples))) {
			printf("CO device %i write %lu samples for %i channels failed with error %d\n", device_index, num_samples, num_channels, error);
			goto Error;
		}

		// get the counter output port
		// this code assumes num_channels = 1 and device_index corresponding to channel. we check this above.
		// note: if this is called too early gives strange error -200216:
		// DAQmx Error: Buffered operations cannot use a Data Transfer Mechanism of Programmed I/O for this device and Channel Type.
		//for (i = 0; i < num_channels; ++i) {
		{
		    i = 0;
			counter_pfi[device_index] = malloc(COUNTER_BUFSIZE+1);
			if (counter_pfi[device_index] == NULL) {
				printf("CO device %i counter %s buffer allocation failed\n", device_index, channels[i]);
				error = -24;
				goto Error;
			}
			DAQmxErrChk(DAQmxGetCOPulseTerm(taskHandle, channels[i], counter_pfi[device_index], COUNTER_BUFSIZE));
			printf("CO device %i counter %s, port %s\n", device_index, channels[i], counter_pfi[device_index]);
		}
	}

#if NUM_AO_DEVICES > 0
	else if (type == AO_TASK) {
		// get associated counter
		ctr_index = ao_ctr[device_index];

		// check if counter port is initialized
		if (counter_pfi[ctr_index] == NULL) {
			printf("AO device %i setup sample clock counter %s, port not given! call write_task for counter first!\n", device_index, counter[ctr_index]);
			error = -22;
			goto Error;
		}

		printf("AO device %i setup sample clock counter %s, port %s, output rate %.3fMHz\n", device_index, counter[ctr_index], counter_pfi[ctr_index], ao_rate[device_index] / 1e6);

        // configure PFI port of counter as time base
	    DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle,
		    counter_pfi[ctr_index],
			ao_rate[device_index],
		    DAQmx_Val_Rising,
		    DAQmx_Val_FiniteSamps,
		    num_samples
	    ));

		// write analog data
		// note: we could use also DAQmxWriteBinaryI16
		printf("AO device %i write %lu samples for %i channels\n", device_index, num_samples, num_channels);
		DAQmxErrChk(DAQmxWriteAnalogF64(taskHandle,
			num_samples,
			FALSE, // autostart
			WRITE_TIMEOUT, // timeout
			DAQmx_Val_GroupByScanNumber, //DAQmx_Val_GroupByChannel or DAQmx_Val_GroupByScanNumber
			(double*)values,
			&written,
			NULL
		));
	}
#endif

#if NUM_DO_DEVICES > 0
	else if (type == DO_TASK) {
		// get associated counter
		ctr_index = do_ctr[device_index];

		// check if counter port is initialized
		if (counter_pfi[ctr_index] == NULL) {
			printf("DO device %i setup sample clock counter %s, port not given! call write_task for counter first!\n", device_index, counter[ctr_index]);
			error = -23;
			goto Error;
		}

		printf("DO device %i setup sample clock counter %s, port %s, output rate %.3fMHz\n", device_index, counter[ctr_index], counter_pfi[ctr_index], do_rate[device_index] / 1e6);

		// configure PFI port of counter as time base
		DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle,
			counter_pfi[ctr_index],
			do_rate[device_index],
			DAQmx_Val_Rising,
			DAQmx_Val_FiniteSamps,
			num_samples
		));

		// write digital data
		printf("DO device %i write %lu samples for %i channels\n", device_index, num_samples, num_channels);
		DAQmxErrChk(DAQmxWriteDigitalU32(taskHandle,
			num_samples,
			FALSE, // autostart
			WRITE_TIMEOUT, // timeout
			DAQmx_Val_GroupByChannel,
			(uInt32*)values,
			&written,
			NULL
		));
	}
#endif
	
	// return task if ok
	return taskHandle;
Error:
	if (taskHandle != 0) {
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	// return error code and 0 on error
	*error_out = error;
	return 0;
}

int main(void)
{
	int         error = 0, i, j, loop = 0;
	TaskHandle  taskHandle = 0;
	int32       written = 0;
	uInt64		pos = 0, samples = 0;

	// samples for each counter
	size_t num_samples_co	[NUM_COUNTER] = { 0 };
	double* times_co		[NUM_COUNTER] = { 0 };
	TaskHandle COtaskHandle	[NUM_COUNTER] = { 0 };

#if NUM_AO_DEVICES > 0
	// samples for each AO device
	size_t num_samples_ao	[NUM_AO_DEVICES] = { 0 };
	double* times_ao		[NUM_AO_DEVICES] = { 0 };
	double* values			[NUM_AO_DEVICES] = { 0 };
	TaskHandle AOtaskHandle	[NUM_AO_DEVICES] = { 0 };
#endif

#if NUM_DO_DEVICES > 0
	// samples for each DO device
	size_t num_samples_do	[NUM_DO_DEVICES] = { 0 };
	double* times_do		[NUM_DO_DEVICES] = { 0 };
	uInt32* pulses			[NUM_DO_DEVICES] = { 0 };
	TaskHandle DOtaskHandle	[NUM_DO_DEVICES] = { 0 };
#endif

	// if several counters are used EXT_CLOCK and EXT_TRIG are needed to synchronize the counters!
#if (NUM_COUNTER > 1)
#if (!defined(EXT_CLOCK)) && (!defined(EXT_TRIG))
	printf("\nATTENTION! %i counters are used without external clock and trigger!\nthe counters are not synchronized!\npress <Enter> to continue ...\n\n", NUM_COUNTER);
	getchar();
#elif (!defined(EXT_CLOCK))
	printf("\nATTENTION! %i counters are used without external clock!\nthe counters are not synchronized!\npress <Enter> to continue ...\n\n", NUM_COUNTER);
	getchar();
#elif (!defined(EXT_TRIG))
	printf("\nATTENTION! %i counters are used without external trigger!\nthe counters are not synchronized!\npress <Enter> to continue ...\n\n", NUM_COUNTER);
	getchar();
#endif
#endif // NUM_COUNTER > 1

#if NUM_AO_DEVICES > 0
	// create samples for each AO device
	for (i = 0; i < NUM_AO_DEVICES; ++i) {
		j = ao_ctr[i]; // counter index
		if ((counter_used[j] != 0) && (counter_rate[j] != ao_rate[i])) {
			printf("AO device %i output rate %.3fMHz incompatible with counter %s rate %.3fMHz!\nshared counter must use the same rate!\n", i, ao_rate[i]/1e6, counter[j], counter_rate[j]/1e6);
			error = -1;
			goto Error;
		}
		num_samples_ao[i] = create_samples(
			DURATION, ao_rate[i],
			NUM_AO, 1,
			V_init, V_low, V_high, V_rate,
			&times_ao[i], (void*)&values[i],
			T_START, T_FULL, T_WAIT);
		if (num_samples_ao[i] == 0) {
			printf("AO device %i generation of samples failed!\n", i);
			error = -2;
			goto Error;
		}
		else {
			// ok: assign with counter
			counter_used  [j] = 1;
			num_samples_co[j] = num_samples_ao[i];
			times_co      [j] = times_ao      [i];
			counter_rate  [j] = ao_rate       [i];
			printf("AO device %i %lu samples generated\n", i, num_samples_ao[i]);
		}
	}
#endif

#if NUM_DO_DEVICES > 0
	// create samples for each DO device
	for (i = 0; i < NUM_DO_DEVICES; ++i) {
		j = do_ctr[i]; // counter index
		if ( (counter_used[j] != 0) && (counter_rate[j] != do_rate[i]) ) {
			printf("DO device %i output rate %.3fMHz is incompatible with counter %s rate %.3fMHz!\nshared counter must use the same rate!\n", i, do_rate[i]/1e6, counter[j], counter_rate[j]/1e6);
			error = -5;
			goto Error;
		}
		num_samples_do[i] = create_samples(
				DURATION, do_rate[i],
				NUM_AO, 0,
				P_init, P_low, P_high, P_rate,
				&times_do[i], (void*)&pulses[i],
				T_START, T_FULL, T_WAIT);
		if (num_samples_do[i] == 0) {
			printf("DO device %i generation of samples failed!\n", i);
			error = -6;
			goto Error;
		}
		else {
			// ok: assign with counter
			counter_used  [j] = 1;
			num_samples_co[j] = num_samples_do[i];
			times_co      [j] = times_do      [i];
			counter_rate  [j] = do_rate       [i];
			printf("DO device %i %lu samples generated\n", i, num_samples_do[i]);
		}
	}
#endif

	// create samples for each counter without AO or DO channel.
	for (i = 0; i < NUM_COUNTER; ++i) {
		if (!counter_used[i]) {
			num_samples_co[i] = create_samples(
				DURATION, counter_rate[i],
				NUM_AO, 0,
				P_init, P_low, P_high, P_rate,
				&times_co[i], NULL,
				T_START, T_FULL, T_WAIT);
			if (num_samples_co[i] == 0) {
				printf("CO device %i counter %s: generation of samples failed! (only counter)\n", i, counter[i]);
				error = -10;
				goto Error;
			}
			else {
				printf("CO device %i counter %s: %lu samples generated (only counter)\n", i, counter[i], num_samples_co[i]);
			}
		}
	}

    // loop
#if NUM_LOOPS > 0    
	for (loop = 0; (!error) && (loop < NUM_LOOPS); ++loop) {
#else
	for (loop = 0; !error; ++loop) {
#endif

		/*********************************************/
		// DAQmx Configure Code
		/*********************************************/

		// configure counter first. 
		// this initializes the PFI output port needed to configure AO and DO channels.
		// for each counter we must create independent task regardless if on the same device or not.
		for (i = 0; i < NUM_COUNTER; ++i) {
			COtaskHandle[i] = write_task(counter + i, 1, num_samples_co[i], times_co[i], CO_TASK, i, &error);
			if (COtaskHandle[i] == 0) {
				if (!error) error = -100;
				goto Error;
			}
		}
		
#if NUM_AO_DEVICES > 0
		// configure analog output task, write all data for all channels
		// same data for all devices
		for (i = 0; i < NUM_AO_DEVICES; ++i) {
	        AOtaskHandle[i] = write_task(ao_channel[i], NUM_AO, num_samples_ao[i], values[i], AO_TASK, i, &error);
			if (AOtaskHandle[i] == 0) {
				if (!error) error = -110;
				goto Error;
			}
	    }
#endif

#if NUM_DO_DEVICES > 0
		// configure digital output task and write for all channels
        // same data for all devices
		for (i = 0; i < NUM_DO_DEVICES; ++i) {
   		    DOtaskHandle[i] = write_task(do_port[i], NUM_DO, num_samples_do[i], pulses[i], DO_TASK, i, &error);
			if (DOtaskHandle[i] == 0) {
				if (!error) error = -120;
				goto Error;
			}
    	}
#endif

		printf("\nregister done event\n\n");
		DAQmxErrChk(DAQmxRegisterDoneEvent(COtaskHandle[0], 0, DoneCallback, NULL));

		/*********************************************/
		// DAQmx Start Code
		/*********************************************/

#if NUM_AO_DEVICES > 0
		// start all analog tasks 
		for (i = 0; i < NUM_AO_DEVICES; ++i) {
		    printf("AO device %i start %i channel(s) ...\n", i, NUM_AO);
		    DAQmxErrChk(DAQmxStartTask(AOtaskHandle[i]));
		}
#endif

#if NUM_DO_DEVICES > 0
		// start all digital tasks 
		for (i = 0; i < NUM_DO_DEVICES; ++i) {
		    printf("DO device %i start %i port(s) ...\n", i, NUM_DO);
		    DAQmxErrChk(DAQmxStartTask(DOtaskHandle[i]));
		}
#endif

		// start all counters
		for (i = 0; i < NUM_COUNTER; ++i) {
			printf("CO device %i start counter %s ...\n", i, counter[i]);
			DAQmxErrChk(DAQmxStartTask(COtaskHandle[i]));
		}

#if NUM_LOOPS > 0
		printf("\nrunning loop %i/%i ... expect done event ... press Enter to stop ...\n", loop, NUM_LOOPS);
#else
		printf("\nrunning loop %i ... expect done event ... press Enter to stop ...\n", loop);
#endif
		getchar();
		while (1.0) {
			error = DAQmxWaitUntilTaskDone(COtaskHandle[0], 1.0);
			if (error == DAQmxErrorWaitUntilDoneDoesNotIndicateDone) error = 0;
			else break;
			if (_kbhit()) {
				printf("\nkeyboard hit!\n\n");
				break;
			}
		}

#if NUM_AO_DEVICES > 0
		for (i = 0; i < NUM_AO_DEVICES; ++i) {
		    DAQmxErrChk(DAQmxGetWriteCurrWritePos(AOtaskHandle[i], &pos));
		    DAQmxErrChk(DAQmxGetWriteTotalSampPerChanGenerated(AOtaskHandle[i], &samples));

		    if ((pos == 0xffffffffffffffff) || (samples == 0xffffffffffffffff)) {
			    printf("AO device %i not started %i channel(s)!\n", i, NUM_AO);
		    }
			else if (num_samples_ao[i] != (size_t)samples) {
				printf("AO device %i done %i channel(s), %llu/%lu samples (partial)\n", i, NUM_AO, samples, num_samples_ao[i]);
			}
			else {
			    printf("AO device %i done %i channel(s), %llu/%lu samples (ok)\n", i, NUM_AO, samples, num_samples_ao[i]);
		    }
		}
#endif

#if NUM_DO_DEVICES > 0
        for (i = 0; i < NUM_DO_DEVICES; ++i) {
		    DAQmxErrChk(DAQmxGetWriteCurrWritePos(DOtaskHandle[i], &pos));
		    DAQmxErrChk(DAQmxGetWriteTotalSampPerChanGenerated(DOtaskHandle[i], &samples));

		    if ((pos == 0xffffffffffffffff) || (samples == 0xffffffffffffffff)) {
			    printf("DO device %i not started %i port(s)!\n", i, NUM_DO);
		    }
			else if (num_samples_do[i] != (size_t)samples) {
				printf("DO device %i done %i port(s), samples %llu/%lu (partial)\n", i, NUM_DO, samples, num_samples_do[i]);
			}
			else {
			    printf("DO device %i done %i port(s), samples %llu/%lu (ok)\n", i, NUM_DO, samples, num_samples_do[i]);
		    }
		}
#endif

		for (i = 0; i < NUM_COUNTER; ++i) {
			DAQmxErrChk(DAQmxGetWriteCurrWritePos(COtaskHandle[i], &pos));
			DAQmxErrChk(DAQmxGetWriteTotalSampPerChanGenerated(COtaskHandle[i], &samples));

			if ((pos == 0xffffffffffffffff) || (samples == 0xffffffffffffffff)) {
				printf("CO device %i not started counter %s!\n", i, counter[i]);
			}
			else if (num_samples_co[i] != (size_t)samples) {
				printf("CO device %i done counter %s, samples %llu/%lu (partial)\n", i, counter[i], samples, num_samples_co[i]);
			}
			else {
				printf("CO device %i done counter %s, samples %llu/%lu (ok)\n", i, counter[i], samples, num_samples_co[i]);
			}
		}
	
	Error:
		if (DAQmxFailed(error)) {
			DAQmxGetExtendedErrorInfo(buffer, BUFSIZE);
		}

		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/

        // stop counter		
        for (i = 0; i < NUM_COUNTER; ++i) {
		    if (COtaskHandle[i] != 0) {
			    DAQmxStopTask(COtaskHandle[i]);
			    DAQmxClearTask(COtaskHandle[i]);
		    }
		}

#if NUM_AO_DEVICES > 0
		// stop analog tasks
        for (i = 0; i < NUM_AO_DEVICES; ++i) {
	        if (AOtaskHandle != 0) {
                DAQmxStopTask(AOtaskHandle[i]);
	            DAQmxClearTask(AOtaskHandle[i]);
	        }
	    }
#endif

#if NUM_DO_DEVICES > 0
	    // stop digital tasks
        for (i = 0; i < NUM_DO_DEVICES; ++i) {
	        if (DOtaskHandle != 0) {
                DAQmxStopTask(DOtaskHandle[i]);
	            DAQmxClearTask(DOtaskHandle[i]);
	        }
	    }
#endif

		if (_kbhit()) {
			printf("\nkeyboard hit!\n\n");
			break;
		}
	} // next loop

	// free allocated memory for each counter, AO and DO device
	// only unused counters have times allocated separately from AO and DO samples.
	for (i = 0; i < NUM_COUNTER; ++i) {
		if (!counter_used[i]) {
			if (times_co[i]) free(times_co[i]);
		}
		if (counter_pfi[i]) free(counter_pfi[i]);
    }

#if NUM_AO_DEVICES > 0
	for (i = 0; i < NUM_AO_DEVICES; ++i) {
		if (times_ao[i]) free(times_ao[i]);
		if (values  [i]) free(values  [i]);
	}
#endif

#if NUM_DO_DEVICES > 0
	for (i = 0; i < NUM_DO_DEVICES; ++i) {
		if (times_do[i]) free(times_do[i]);
		if (pulses  [i]) free(pulses  [i]);
	}
#endif

	if ( error < 0 ) {
	    if (error > -1000) {
	        printf("error %d\n", error);
	    }
	    else {
	        if( DAQmxFailed(error) ) {
		        printf("DAQmx Error %d: %s\n",error, buffer);
            }
            else printf("DAQmx unknown error %d\n", error);
        }
    }
	printf("\nEnd of program\n\n");
	//getchar();

	return 0;
}

int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void *callbackData)
{
	int32   error=0;

	// Check to see if an error stopped the task.
	DAQmxErrChk (status);

Error:
	if( DAQmxFailed(error) ) {
	  DAQmxGetExtendedErrorInfo(buffer, BUFSIZE);
	  DAQmxClearTask(taskHandle);
	  printf("DAQmx Error: %s\n", buffer);
	}
	else {
		printf("\ndone!\n\n");
	}
	return 0;
}


