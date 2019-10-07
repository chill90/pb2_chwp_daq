import socket
import numpy
import struct
import copy
import time
from collections import deque
import select
import csv
import datetime
import sys
import os
#import location as loc
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy import signal

# The number of datapoints in every encoder packet from the Arduino
COUNTER_INFO_LENGTH = 150
# The size of the encoder packet from the Arduino (header + 3*150 datapoint information + 3 quadrature readout)
COUNTER_PACKET_SIZE = 4 + 4 * COUNTER_INFO_LENGTH+8 * COUNTER_INFO_LENGTH + 12
# The size of the IRIG packet from the Arduino
IRIG_PACKET_SIZE = 132

#overflow = []
# Class which will parse the incoming packets from the Arduino and store the data in CSV files
class EncoderParser(object):
    # port: This must be the same as the localPort in the Arduino code
    # date, run: Used for naming the CSV file (will be changed later on in the code by a user input)
    # read_chunk_size: This value shouldn't need to change
    
    def __init__(self, saveDir, date = "test", run = "test", beaglebone_port = 8080, read_chunk_size = 8196):
        #Directory to save files
        self.saveDir = saveDir
        
        # Creates three lists to hold the data from the encoder, IRIG, and quadrature respectively
        self.counter_queue = deque()
        self.irig_queue = deque()
        self.quad_queue = deque()

        # Used for procedures that only run when data collection begins
        self.is_start = 1
        # Will hold the time at which data collection started [hours, mins, secs]
        self.start_time = [0,0,0]
        # Will be continually updated with the UTC time in seconds
        self.current_time = 0

        # Creates a UDP socket to connect to the Arduino
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Binds the socket to a specific ip address and port
        # The ip address should be the same as the remote_ip in the Arduino code
        self.s.bind(('192.168.2.54', beaglebone_port))
        self.s.setblocking(0)

        # String which will hold the raw data from the Arduino before it is parsed
        self.data = ''
        self.read_chunk_size = read_chunk_size

        # Keeps track of how many packets have been parsed
        self.counter = 0

        # Date and run number used to name the CSV file
        self.date = date
        self.run = run

        # Creates two CSV files to hold the Encoder and IRIG data
        self.fname1 = self.saveDir+"/Encoder_Data_"+self.run+".csv"
        self.fname2 = self.saveDir+"/IRIG_Data_"+self.run+".csv"
        self.file1 = open(self.fname1, "w")
        self.file2 = open(self.fname2, "w")
        self.file1.close()
        self.file2.close()

        # Opens the files again and writes the headers
        with open(self.fname1, "w") as self.Encoder_Data_CSV, open(self.fname2, "w") as self.IRIG_Data_CSV:
            self.Encoder_CSV = csv.writer(self.Encoder_Data_CSV)
            # Header for the Encoder file
            self.Encoder_CSV.writerows([['1: Quad readout','2-152: capt_cnt/clk_cnt'],[]])

            self.IRIG_CSV = csv.writer(self.IRIG_Data_CSV)
            # Header for the IRIG file
            self.IRIG_CSV.writerows([['1: IRIG_time/clk_cnt','3-13: synch_pulse/clk_cnt'],[]])

    # Converts the IRIG signal into sec/min/hours depending on the parameters
    def de_irig(self, val, base_shift=0):
        return (((val >> (0+base_shift)) & 1) + 
                ((val >> (1+base_shift)) & 1) * 2 + 
                ((val >> (2+base_shift)) & 1) * 4 + 
                ((val >> (3+base_shift)) & 1) * 8 + 
                ((val >> (5+base_shift)) & 1) * 10 + 
                ((val >> (6+base_shift)) & 1) * 20 + 
                ((val >> (7+base_shift)) & 1) * 40 )

    # Takes the IRIG information, prints it to the screen, sets the current time,
    # and returns the current time
    def pretty_print_irig_info(self, v, edge):
        # Calls self.de_irig() to get the sec/min/hour of the IRIG packet
        secs = self.de_irig(v[0], 1)
        mins = self.de_irig(v[1], 0)
        hours = self.de_irig(v[2], 0)

        # If it is the first time that the function is called then set self.start_time
        # to the current time
        if self.is_start == 1:
            self.start_time = [hours, mins, secs]
            self.is_start = 0

        # Find the sec/min/hour digit difference from the start time
        dsecs = secs - self.start_time[2]
        dmins = mins - self.start_time[1]
        dhours = hours - self.start_time[0]

        # Corrections to make sure that dsecs/dmins/dhours are all positive
        if dhours < 0:
            dhours = dhours + 24

        if (dmins < 0)or((dmins == 0)and(dsecs < 0)):
            dmins = dmins + 60
            dhours = dhours - 1

        if dsecs < 0:
            dsecs = dsecs + 60
            dmins = dmins - 1
        
        # Print UTC time, run time, and current clock count of the Arduino
        print ("Current Time:",("%d:%d:%d"%(hours, mins, secs)),"Run Time",("%d:%d:%d"%(dhours, dmins, dsecs)), "Clock Count",edge)

        # Set the current time in seconds
        self.current_time = secs + mins*60 + hours*3600

        return self.current_time

    # Checks to make sure that self.data is the right size
    # Return false if the wrong size, return true if the data is the right size
    def check_data_length(self, start_index, size_of_read):
        if start_index + size_of_read > len(self.data):
            self.data = self.data[start_index:]
            print ("UH OH")
            return False
        else:
            return True

    # Grabs self.data, determine what packet it corresponds to, parses the data, and
    # records it to CSV file
    def grab_and_parse_data(self):
        while True:
            # If there is data from the socket attached to the Arduino then ready[0] = true
            # If not then continue checking for 2 seconds and if there is still no data ready[0] = false
            ready = select.select([self.s],[],[],2)
            if ready[0]:
                # Add the data from the socket attached to the Arduino to the string self.data
                self.data += self.s.recv(self.read_chunk_size)
                while True:
                    # Check to make sure that there is at least 1 int in the packet
                    # The first int in every packet should be the header
                    if not self.check_data_length(0, 4):
                        print ('Error 0')
                        break

                    header = self.data[0 : 4]
                    # Convert a structure value from the Arduino (header) to an int
                    header = struct.unpack('<I', header)[0]

                    # 0x1EAF = Encoder Packet
                    # 0xCAFE = IRIG Packet
                    # 0xE12A = Error Packet

                    # Encoder
                    if header == 0x1EAF:
                        # Make sure the data is the correct length for an Encoder Packet
                        if not self.check_data_length(0, COUNTER_PACKET_SIZE):
                            print ('Error 1')
                            break
                        # Call the meathod self.parse_counter_info() to parse the Encoder Packet
                        self.parse_counter_info(self.data[4 : COUNTER_PACKET_SIZE])
                        # Increment self.counter to signify that an Encoder Packet has been parsed
                        self.counter += 1

                    # IRIG
                    elif header == 0xCAFE:
                        # Make sure the data is the correct length for an IRIG Packet
                        if not self.check_data_length(0, IRIG_PACKET_SIZE):
                            print ('Error 2')
                            break
                        # Call the meathod self.parse_irig_info() to parse the IRIG Packet
                        self.parse_irig_info(self.data[4 : IRIG_PACKET_SIZE])

                    # Error
                    # An Error Packet will be sent if there is a timing error in the 
                    # synchronization pulses of the IRIG packet
                    # If you see 'Packet Error' check to make sure the IRIG is functioning as
                    # intended and that all the connections are made correctly 
                    elif header == 0xE12A:
                        print ('Packet Error')

                    else:
                        print ('Bad header')

                    # Clear self.data
                    self.data = ''
                    break
                break
            
            # If there is no data from the Arduino 'Looking for data ...' will print
            # If you see this make sure that the Arduino has been set up properly
            else:
                print ('Looking for data ...')

    # Meathod to parse the Encoder Packet
    def parse_counter_info(self, data):
        # Convert the Encoder Packet structure into a numpy array
        derter = numpy.array(struct.unpack('<' + 'III'*COUNTER_INFO_LENGTH + 'III', data))

        # [0-149] clock counts of 150 data points
        # [150-299] corresponding clock overflow of the 150 data points (each overflow count
        # is equal to 2^16 clock counts)
        # [300-449] corresponding absolute number of the 150 data points ((1, 2, 3, etc ...)
        # or (150, 151, 152, etc ...) or (301, 302, 303, etc ...) etc ...)
        # [450-452] Readout from the quadrature

        # NOTE: These next two lines are only required if the data is going to be manipulated
        # within this code and not just recorded
    
        # self.counter_queue = [[clock count array],[absolute number array]]
        self.counter_queue.append(( derter[0:150] + (derter[150:300] << 32), derter[300:450]))
        # self.quad_queue = [[quad input 2 array],[quad input 3 array],[quad input 4 array]]
        self.quad_queue.append((derter[450], derter[451], derter[452]))

        # Write the Encoder Data to a CSV file
        with open(self.fname1, "a") as self.Encoder_Data_CSV:
            # What's writen to the Encoder file:
            # [quad input 2, quad input 3, quad input 4]
            # []
            # [absolute number, clock count]*150
            # []
            self.Encoder_CSV = csv.writer(self.Encoder_Data_CSV)
            self.Encoder_CSV.writerows([[derter[450], derter[451], derter[452]], []])
            self.Encoder_CSV.writerows(numpy.transpose([derter[300:450], derter[0:150]+(derter[150:300]<<32)]))
            self.Encoder_CSV.writerows([[]])

    # Meathod to parse the IRIG Packet
    def parse_irig_info(self, data):
        # Convert the IRIG Packet structure into a numpy array
        unpacked_data = struct.unpack('<L' + 'L' + 'L'*10 + 'L'*10 + 'L'*10, data)

        # [0] clock count of the IRIG Packet which the UTC time corresponds to
	# [1] overflow count of initial rising edge
        # [2] binary encoding of the second data
        # [3] binary encoding of the minute data
        # [4] binary encoding of the hour data
        # [5-11] additional IRIG information which we do mot use
        # [12-21] synchronization pulse clock counts
	# [22-31] overflow count at each synchronization pulse

        # Start of the packet clock count
	#overflow.append(unpacked_data[1])
        #print "overflow: ", overflow
        rising_edge_time = unpacked_data[0] + (unpacked_data[1] << 32)
        # Stores IRIG time data
        irig_info = unpacked_data[2:12]

        # Prints the time information and returns the current time in seconds
        irig_time = self.pretty_print_irig_info(irig_info, rising_edge_time)
        # Stores synch pulse clock counts accounting for overflow of 32 bit counter
        synch_pulse_clock_times = (numpy.asarray(unpacked_data[12:22]) + (numpy.asarray(unpacked_data[22:]) << 32)).tolist()

        # NOTE: This next line is only required if the data is going to be manipulated
        # within this code and not just recorded

        # self.irig_queue = [Packet clock count,Packet UTC time in sec,[binary encoded IRIG data],[synch pulses clock counts]]
        self.irig_queue.append((rising_edge_time, irig_time, irig_info, synch_pulse_clock_times))

        # Write the IRIG Data to a CSV file
        with open(self.fname2, "a") as self.IRIG_Data_CSV:
            # What's writen to the IRIG file:
            # [UTC time in sec, clock count]
            # []
            # [0, synch pulse clock count 1]
            # [...]
            # [9, synch pulse clock count 10]
            # []
            self.IRIG_CSV = csv.writer(self.IRIG_Data_CSV)
            self.IRIG_CSV.writerows([[irig_time, rising_edge_time], []])
            self.IRIG_CSV.writerows(numpy.transpose([range(10), synch_pulse_clock_times]))
            self.IRIG_CSV.writerows([[]])

    def __del__(self):
        self.s.close()


# NOTE: The next 4 founctions are only used if data is manipulated within this code and not
# just recorded

# Returns two arrays; diff = array of the difference between adjacent points in the array
# clk_cnts, time = corresponding index for values in the diff array (1, 2, 3, etc ...)
def find_difference(clk_cnts):
    diff = []
    for i in range(len(clk_cnts)-1):
        diff.append(clk_cnts[i+1] - clk_cnts[i])
    time = range(len(clk_cnts)-1)
    return diff, time

# Takes arrays containing the IRIG information and the Encoder infornation and converts
# the information into two new arrays; angle = angle of the CHWP (still needs to be adjusted
# to some referance), time = corresponding time of the values in the 'angle' array
# The function returns the arrayts in the form encoder_data = (angle array, time array)
def convert_to_angle(IRIG_time, IRIG_clock, encoder_clock, encoder_count, slit_scalar = (570*2)):
    if ((len(IRIG_time) != len(IRIG_clock)) or (len(encoder_clock) != len(encoder_count))):
        print ("Data Length Error")
        return
    index = 0
    encoder_data = deque()
    for i in range(len(IRIG_time)-1):
        while index < len(encoder_clock):
            if encoder_clock[index] > IRIG_clock[i+1]:
                break
            elif encoder_clock[index] < IRIG_clock[i]:
                index = index + 1
            else:
                encoder_time = IRIG_time[i]+(IRIG_time[i+1]-IRIG_time[i])*(encoder_clock[index]-IRIG_clock[i])/(IRIG_clock[i+1]-IRIG_clock[i])
                encoder_data.append((encoder_time*2000*numpy.pi/slit_scalar,encoder_time))
                index = index + 1
    return encoder_data

# Whenever input_array[i] > input_array[i+1], add 2^nbits to input_array[j] for all j>i
# This accounts for the Arduino counter overflowing and insures that the the array is always increasing
def account_for_wrapping(input_array, nbits):
    add_val = int(2.0**nbits)
    out_arr = copy.copy(input_array)
    arr = copy.copy(input_array)
    arr[0] = 1
    arr[1:] = arr[1:] - arr[:-1]
    for ind in numpy.where(arr < 0)[0]:
        out_arr[ind:] += add_val
    return out_arr

# Due to the nature of the Arduino code that are times when a clock count is 2^16 lower
# than it should be and this function corrects for that
def account_for_missed_ovflow(input_array):
    out_arr = copy.copy(input_array)
    arr = copy.copy(input_array)
    arr[0] = 1
    arr[1:] = arr[1:] - arr[:-1]
    for ind in numpy.where(numpy.logical_and(arr<-2**12, arr>-2**24))[0]:
        out_arr[ind] += 2**16
    return out_arr

# Accounts for wrapping in the IRIG clock when the day changes
def account_for_next_day(input_array):
    out_arr = copy.copy(input_array)
    arr = copy.copy(input_array)
    arr[0] = 1
    arr[1:] = arr[1:] - arr[:-1]
    for ind in numpy.where(arr < 0)[0]:
        out_arr[ind:] += 24*3600
    return out_arr

# Portion of the code that runs
if __name__ == '__main__':
    '''
    mode = ''
    while True:
        # Asks the user what meathod of data collection they would like to use
        mode = int(raw_input("Runtime in Seconds = 0, Number of Packets to Collect = 1: "))
        if (mode == 1):
            # Asks the user the number of packets to record
            data_points = int(raw_input("Enter Number of Packets to Collect: "))
            break
        elif (mode == 0):
            # Asks the user how many seconds they want to record for
            runtime = int(raw_input("Enter Runtime in Seconds: "))
            break
        else:
            print "Invalid Entry"
    '''
    #Use command-line arguments to obtain save directory
    args = sys.argv[1:]
    if not len(args) == 2:
        sys.exit("Usage: python encoderDAQ.py [Run Name] [Number of seconds to collect data]\n")
    else:
        runName = str(args[0])
        runtime = int(args[1])
        mode = 0 #Keep this fixed for now

        masterDir = "/home/polarbear/data/"
        print ("All encoder data collected on the CHWP NUC PC is stored in %s" % (masterDir))
        
        if not os.path.isdir(masterDir):
            sys.exit('Master path %s does not exist\n' % (masterDir))
        else:
            inputDir = masterDir+runName+"/"
            if os.path.exists(inputDir):
                while True:
                    overwrite = raw_input("CAUTION: Run name %s already exists at location %s. Overwrite existing data? Y/N [Y]: " % (runName, masterDir))
                    if overwrite == "":
                        break
                    elif "y" in overwrite or "Y" in overwrite:
                        break
                    elif "n" in overwtie or "N" in overwrite:
                        sys.exit("FATAL: Raw data at %s will not be overwritten" % (inputDir))
                    else:
                        print ("Did not understand input %s..." % (overwrite))
                        continue
            else:
                os.makedirs(inputDir)
            saveDir = inputDir+"/rawData/"
            if not os.path.exists(saveDir):
                print ("Creating directory %s..." % (saveDir))
                os.makedirs(saveDir)

    # Creates an instance of the EncoderParser class with the current date and asks the user
    # for the run number
    #ep = EncoderParser(date = str(datetime.date.today()), run = raw_input("Enter the Run Number: "))
    ep = EncoderParser(saveDir = saveDir, date = str(datetime.date.today()), run = runName)
    if True:
            print ('Starting')
            
            if (mode == 0):
                # Run until current_time == strat_time + runtime
                start_time = 123456789
                while (start_time+runtime != ep.current_time) and (start_time+runtime != ep.current_time + 24*3600):
                    ep.grab_and_parse_data()
                    start_time = ep.start_time[0]*3600 + ep.start_time[1]*60 + ep.start_time[2]
            elif (mode == 1):
                # Run until the specified number of packets have been collected
                while data_points > ep.counter:
                    ep.grab_and_parse_data()

            print 'Done'
            
            # Additional graphing
#            """encoder_clock_cnts, encoder_capture_cnt = zip(*ep.counter_queue)
#            encoder_clock_cnts = account_for_wrapping(account_for_missed_ovflow(numpy.array(encoder_clock_cnts).flatten()),32)
#            encoder_capture_cnt = account_for_wrapping(numpy.array(encoder_capture_cnt).flatten(),16)

            #plt.figure(12)
            #plt.plot(encoder_clock_cnts, encoder_capture_cnt)
            #plt.savefig('Capture vs Clock.png')

#            try:
#                IRIG_clock, IRIG_time, IRIG_info, synch_pulse = zip(*ep.irig_queue)
#                IRIG_clock = account_for_wrapping(numpy.array(IRIG_clock).flatten(),32)
#                IRIG_time = account_for_next_day(numpy.array(IRIG_time).flatten())
#                synch_pulse = account_for_wrapping(numpy.array(synch_pulse).flatten(),32)
#		encoder_clock_cnts, encoder_capture_cnt = zip(*ep.counter_queue)
#		encoder_clock_cnts = account_for_wrapping(account_for_missed_ovflow(numpy.array(encoder_clock_cnts).flatten()),32)
#            	encoder_capture_cnt = account_for_wrapping(numpy.array(encoder_capture_cnt).flatten(),16)
#
                #encoder_data = convert_to_angle((IRIG_time-IRIG_time[0]).astype(float), IRIG_clock-IRIG_clock[0], encoder_clock_cnts-IRIG_clock[0], encoder_capture_cnt)
                #angle, time = zip(*encoder_data)
                #angle = numpy.array(angle).flatten()
                #time = numpy.array(time).flatten()
		
		#numpy.savetxt('bb_data.txt', numpy.column_stack((time,angle)))
		#numpy.savetxt('bb_irig_data.txt', IRIG_clock)
		#Interpolate encoder data using IRIG time
#		encoder_time = numpy.interp(encoder_clock_cnts, IRIG_clock, IRIG_time)

                
#		time = encoder_time - encoder_time[0]
#		tstep = 1./2000.
#		fid_time = (encoder_capture_cnt - encoder_capture_cnt[0])*tstep
#		print len(time)
#		print len(fid_time)
#		time_jitter = time - numpy.polyval(numpy.polyfit(fid_time, time, deg=1), fid_time)

#		bb_T = float(time[-1]-time[0])
#		bb_N = float(len(time))
#		bb_win  = numpy.hanning(bb_N)
#		bb_norm = numpy.sqrt(bb_T/(bb_N**2))*(1./(numpy.trapz(bb_win)/bb_N))
		
#		bb_freq = numpy.fft.rfftfreq(len(encoder_time), d=tstep)
#		bb_pow = abs(bb_norm*numpy.fft.rfft(time_jitter*bb_win))**2
#		plt.figure(0)
		#plt.loglog(a_freq, a_jit, 'r', label='Arduino')
#		plt.loglog(bb_freq, bb_pow, 'k', label = 'Beaglebone')
#		plt.legend()
#		plt.show()
#		sin = numpy.sin(angle)
#		jitter = angle - numpy.polyval(numpy.polyfit(time, angle, 1), time)

#		itime = numpy.linspace(time[0], time[-1], len(time))
#		iangle = numpy.interp(itime, time, angle)
#		isin = numpy.interp(itime, time, sin)
#		ijit = numpy.interp(itime, time, jitter)
#		tstep = itime[1]-itime[0]
               

#                plt.figure(8)
#                plt.plot(time, sin)
#		plt.xlim(0,10)
#                plt.xlabel('Time in Seconds')
#                plt.ylabel('Sine of Angle')
#                plt.title('Sine of Angle vs Time')
#                plt.savefig('Sine of Angle vs Time.png')

#                plt.figure(9)
#                plt.plot(time-time[0], angle-angle[0])
#                plt.xlabel('Time in Seconds')
#                plt.ylabel('Angle in Radians')
#                plt.title('Angle vs Time')
#                plt.savefig('Angle vs Time.png')

		
#		T = itime[-1]-itime[0]
#		N = len(itime)
#		win  = numpy.hanning(N)
#		norm = numpy.sqrt(T/(N**2))*(1./(numpy.trapz(win)/N))
		
		
#		f = numpy.fft.rfftfreq(len(itime), d=tstep)
#		power = abs(norm*numpy.fft.rfft(numpy.sin(iangle)*win))**2
#		jit = abs(norm*numpy.fft.rfft(ijit*win))**2
                
#		print len(itime)/runtime

    
#		plt.figure(10)
#	        plt.plot(time, ijit)
		#plt.xlim(0,10)
#	        plt.xlabel('Time in Seconds')
#	        plt.ylabel('Difference in Angle from Fit')
#	        plt.title('Angle Jitter')
#	        plt.savefig('Angle Jitter.png')

#		plt.figure(13)
#		plt.loglog(f, jit)
#		plt.xlabel('Frequency in Hz')
#		plt.ylabel('Power Spectral Density in $\\frac{Rad^{2}}{Hz}$')
#		plt.title('Power Spectrum of Angle Jitter') 
#		plt.savefig('Power Spectrum of Angle Jitter')

#		plt.figure(14)
#		plt.loglog(f, power)
#		plt.xlim(10 ** -1, 10 **3)
#		plt.xlabel('Frequency in Hz')
#		plt.ylabel('Power Spectral Density in $\\frac{Rad^{2}}{Hz}$')
#		plt.title('Power Spectrum of Sine of Angle')
#		plt.savefig('Power Spectrum of Sine of Angle')

#            except:
#                print "Missing IRIG Data"
#            rising_edge_difference, timeABC = find_difference(encoder_clock_cnts)

#            plt.figure(11)
#            plt.hist(rising_edge_difference)
#            plt.savefig('Histogram of Rising Edge Difference.png')'''
	   
	    
            
