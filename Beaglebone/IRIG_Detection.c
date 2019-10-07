#include <stdlib.h>
#include <stdint.h>

//Addresses in shared memory for variables and data structures shared between PRUs and ARM Processor
#define IRIG_IDENTIFIER 0x00011850
#define IRIG_PACKET_START 0x00011858
#define ON_ADDRESS 0x00010008
#define ERROR_ADDRESS 0x00013008
#define ERROR_IDENTIFIER 0x00013000
#define OVERFLOW_ADDRESS 0x00010010

#define MAX_LOOP_TIME 0x5FFFFFFF //~75% of max counter value, not currently used

// IEP(Industrial Ethernet Peripheral Registers
#define IEP 0x0002e000 //IEP base address
#define IEP_TMR_GLB_CFG ((volatile unsigned long int *)(IEP + 0x00)) //Register IEP Timer configuration
#define IEP_TMR_CNT ((volatile unsigned long int *)(IEP + 0x0c)) //Register for the IEP counter(32-bit, 200MHz)
#define IEP_TMR_COMPEN ((volatile unsigned long int *)(IEP + 0x08)) //Register to configure compensation counter
#define IEP_TMR_GLB_STS ((volatile unsigned long int *)(IEP + 0x04)) //Register to check for counter overflows

#define IRIG_0 0 //2 ms
#define IRIG_1 1 //5 ms
#define IRIG_PI 2  //8 ms
#define IRIG_ERR 3 //Error

#define ERR_NONE 0
#define ERR_DESYNC 1

struct ErrorInfo {
    unsigned int header;
  //0 if all is good, 1 if an error exists
    unsigned int err_code;
};



//Registers to use for PRU input/output, __R31 is input, __R30 is output
volatile register unsigned int __R31, __R30;

//Structure to sample PRU input and determine edges
struct ECAP {
    unsigned long int p_sample; //previous sample of the input register __R31
    unsigned long int ts; //time stamp of edge seen
    unsigned long int p_ts; //time stamp of the last edge that was seen
    unsigned long int trigger; //not used
};

struct IrigInfo {
    unsigned long int random_header;
    unsigned long int rising_edge_time;
    unsigned long int init_overflow; //number of overflows that have occurred when first rising edge seen
    unsigned long int info[10];
    unsigned long int re_count[10];
    unsigned long int re_count_overflow[10]; //number of overflows that have occurred at each synch pulse in IRIG
};




volatile unsigned long int* irig_identifier = (volatile unsigned long int *) IRIG_IDENTIFIER; //Identifies when and which irig struct is ready to be sent over UDP
volatile unsigned long int* on = (volatile unsigned long int *) ON_ADDRESS; //0 if PRUs are still sampling, 1 if they are done
volatile unsigned long int* error_identifier = (volatile unsigned long int *) ERROR_IDENTIFIER; //Identifies if error struct is ready to be sent over UDP
volatile unsigned long int* counter_overflow = (volatile unsigned long int *) OVERFLOW_ADDRESS; //Identifies when and which counter struct is ready to be sent over UDP

int main(void)
{
    //IEP configuration taken care of by the encoder code running on PRU1
    *IEP_TMR_GLB_STS = 1; //Clears Overflow Flags
    *IEP_TMR_GLB_CFG = 0x11; //enables IEP counter to increment by 1 every cycle
    *IEP_TMR_COMPEN = 0; //Disables compensation counter
    volatile struct IrigInfo* irig_packets = (volatile struct IrigInfo *) (IRIG_PACKET_START); //declaring pointer to struct in shared memory for irig structs
    volatile struct ECAP ECAP; //structure to determine when edges are detected
    volatile struct ErrorInfo* error_state = (volatile struct ErrorInfo *) (ERROR_ADDRESS); //declaring point to struct in shared memory for errory struct

    unsigned long long int rising_edge_t; //stores rising edge clock value accounting for overflows
    unsigned long long int falling_edge_t; //stores falling edge clock value accounting for overflows
    unsigned long int short_rising_edge_t; //stores rising edge clock value without accounting for overflows


    //By default, assume bit type is an error
    unsigned char prev_bit_type = IRIG_ERR; //bit '3'
    //By default, assume IRIG parser is not synched
    unsigned char irig_parser_is_synched = 0;

    //Bit position '0' marks the beginning of an IRIG frame
    unsigned char bit_position = 0;

    *irig_identifier = 0; //initial state is that no irig struct is ready to be sent
    *error_identifier = 0; //initial state is that no error struct is ready to be sent

    ECAP.p_sample = 0;
    ECAP.ts = *IEP_TMR_CNT;
    ECAP.p_ts = *IEP_TMR_CNT;
    ECAP.trigger = 1 << 14;

    *counter_overflow = 0; //counts number of times that counter has overflowed
    unsigned int counter_overflow_cached = 0; //used to cache the counter overflow counter
    int i = 0; //used to change between the 2 irig structs that are written to in shared memory
    *on = 0; //*on = 0 means PRUs are taking data

    error_state->header = 0xe12a;
    error_state->err_code = ERR_NONE;

    int x = 0; //(Number of seconds / 2) to take data for
    irig_packets[0].random_header = 0xcafe;
    irig_packets[1].random_header = 0xcafe;
    while(x < 305){
        i = 0;
        while(i < 2){

            //if statement checks the bit that indicates that the counter has overflowed, increments the counter overflow, and resets the bit by writing a 1
            if ((*IEP_TMR_GLB_STS & 1) == 1){
                *counter_overflow += 1;
                *IEP_TMR_GLB_STS = 1;
            }

            unsigned long int sample = (__R31 & (1 << 14)); //register bit 14, or pin P8_16 is sampled
            unsigned long int change = sample ^ ECAP.p_sample; //Checks to see if this sample is different than the last one, if it is, an edge has been seen

            if(change){
                ECAP.p_ts = ECAP.ts;
                ECAP.ts = *IEP_TMR_CNT; //grabs clock value first
                ECAP.p_sample = sample;
                unsigned int overflow_bits = (*IEP_TMR_GLB_STS & 1); //checks again for an overflow that has occurred since the last check
                counter_overflow_cached = *counter_overflow; //caches overflow counter
                unsigned char irig_bit_type = IRIG_ERR;
                //If statement below checks if it is a falling edge
                if ((sample & 1 << 14) == 0){
                    falling_edge_t = ECAP.ts + (((unsigned long long int)(overflow_bits + counter_overflow_cached)) << 32);
                    unsigned long long int delta = falling_edge_t - rising_edge_t;
                    if (delta < 700000) {
                        irig_bit_type = IRIG_0;
                    }
                    else if (delta > 2457600) {
                        irig_bit_type = IRIG_ERR;
                    }
                    else if (delta > 1300000){
                        irig_bit_type = IRIG_PI;
                    }
                    else {
                        irig_bit_type = IRIG_1;
                    }

                    if (irig_parser_is_synched) {

                        if (bit_position == 100) {
                            *irig_identifier = (i+1);
                            if (i == 0) {
                                irig_packets[1].rising_edge_time = short_rising_edge_t; //switches which packet is being written to
                                irig_packets[1].init_overflow = overflow_bits + counter_overflow_cached;
                            }
                            else if (i == 1) {
                                irig_packets[0].rising_edge_time = short_rising_edge_t; //switches which packet is being written to
                                irig_packets[0].init_overflow = overflow_bits + counter_overflow_cached;
                            }
                            bit_position = 1;
                            i += 1;
                        }
                        else if (bit_position % 10 == 9) { //every 9th bit should be synch pulse
                            if (irig_bit_type != IRIG_PI){
                                irig_parser_is_synched = 0;
                                *irig_identifier = 0;
                                error_state->err_code = ERR_DESYNC;
                                *error_identifier = 1;
                            }
                            unsigned char ind = bit_position/10;
                            irig_packets[i].re_count[ind] = short_rising_edge_t;
                            irig_packets[i].re_count_overflow[ind] = overflow_bits + counter_overflow_cached;
                            bit_position += 1;
                        }
                        else {
                            unsigned char offset = bit_position % 10;
                            irig_packets[i].info[bit_position/10] &= ~(1 << offset); //this deals with changing the bit of 'info' to either a 1 or 0 depending on bit type
                            irig_packets[i].info[bit_position/10] |= irig_bit_type << (offset);
                            bit_position += 1;
                        }
                    }
                    else {
                        if (irig_bit_type == IRIG_PI && prev_bit_type == IRIG_PI) { //2 synch pulses in a row means a new irig packet is starting so this is for synchronization
                            bit_position = 1;
                            irig_parser_is_synched = 1;
                            irig_packets[i].rising_edge_time = short_rising_edge_t;
                        }
                    }
                    prev_bit_type = irig_bit_type;
                }
                else {
                    rising_edge_t = ECAP.ts + (((unsigned long long int)(overflow_bits + counter_overflow_cached)) << 32); //gets rising edge time accounting for overflow
                    short_rising_edge_t = ECAP.ts; //only clock count not accounting for overflow is stored in irig struct
                }
            }

        }
        x += 1;




    }
    *on = 1; //lets ARM and other PRU know that this one has stopped taking data after ('x'*2) seconds
    __R31 = 40; //interrupt to ARM to let it know that it is finished
    __halt(); //halts PRU
}
