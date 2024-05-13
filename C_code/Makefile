APP = PseudoClockTest

# Add any other object files to this list below
APP_OBJS = NI_DAQmx_pseudoclock.o

CPPFLAGS += -I.                         #include directory (here)
#LDFLAGS += -L"C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/"            #library directory (not tested)
#LDLIBS += -lNIDAQmx                    #libary without 'lib' (not tested)

all: build

build: $(APP)

.PHONY: clean

clean:
	rm -f *.o

debug: $(APP_OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(APP_OBJS) $(LDLIBS) -O3 -Wall -D_DEBUG=1

$(APP): $(APP_OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(APP_OBJS) $(LDLIBS) -O3 -Wall
