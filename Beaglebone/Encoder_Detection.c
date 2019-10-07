#include <stdlib.h>

#define COUNTER_ADDRESS 0x00010018 //Address for Counter Packets to start being written to
#define PACKET_ADDRESS 0x00010000 //Address to where the packet identifier will be stored
#define ON_ADDRESS 0x00010008 //Address for shared variable 'on' to tell ARM when the PRU is still sampling
#define OVERFLOW_ADDRESS 0x00010010 //Address for shared variable between PRUs to count the overflows since they use the same clock

#define ENCODER_COUNTER_SIZE 150 //Size of edges to sample before sending packet
#define MAX_LOOP_TIME 0x5FFFFFFF //~75% of max counter value

// IEP(Industrial Ethernet Peripheral Registers
#define IEP 0x0002e000 //IEP base address
#define IEP_TMR_GLB_CFG ((volatile unsigned long int *)(IEP + 0x00)) //Register IEP Timer configuration
#define IEP_TMR_CNT ((volatile unsigned long int *)(IEP + 0x0c)) //Register for the IEP counter(32-bit, 200MHz)
#define IEP_TMR_COMPEN ((volatile unsigned long int *)(IEP + 0x08)) //Register to configure compensation counter
#define IEP_TMR_GLB_STS ((volatile unsigned long int *)(IEP + 0x04)) //Register to check for counter overflows

//Registers to use for PRU input/output, __R31 is input, __R30 is output
volatile register unsigned int __R31, __R30;

//Structure to sample PRU input and determine edges
struct ECAP {
    unsigned long int p_sample;
    unsigned long int ts;
    unsigned long int p_ts;
    unsigned long int trigger;
};

//Structure to store clock count of edges and number of times the counter has overflowed
struct CounterInfo{
    unsigned long int clock_cnt[ENCODER_COUNTER_SIZE];
    unsigned long int counter_ovflow[ENCODER_COUNTER_SIZE];
    unsigned long int encoder_cnt[ENCODER_COUNTER_SIZE];
};

//Structure to store values of quadrature encoder status
struct QuadEncoder{
    unsigned long int encoder_value_2;
    unsigned long int encoder_value_3;
    unsigned long int encoder_value_4;
};

//Variable to let PRU know that a quadrature sample is needed on rising edge
int quad_encoder_needed = 1;

//pointers to packet identifier and the on variable and overflow variable
volatile unsigned long int* packet_address = (volatile unsigned long int *) PACKET_ADDRESS;
volatile unsigned long int* on = (volatile unsigned long int *) ON_ADDRESS;
volatile unsigned long int* counter_overflow = (volatile unsigned long int *) OVERFLOW_ADDRESS; //overflow variable is updated by IRIG code, incremented everytime the counter overflows

//Structure that contains the entire packet to be sent over UDP to host computer
struct CompleteDataPacket{
    unsigned long int counter_info_header;
    volatile struct CounterInfo Counter_Packets;
    volatile struct QuadEncoder Quad;
};

int main(void)
{
    unsigned int input_capture_count = 0; //variable to count number of edges seen
    *IEP_TMR_GLB_STS = 1; //Clears Overflow Flags
    *IEP_TMR_GLB_CFG = 0x11; //enables IEP counter to increment by 1 every cycle
    *IEP_TMR_COMPEN = 0; //Disables compensation counter
    volatile struct CompleteDataPacket* Data_Packets = (volatile struct CompleteDataPacket *) (COUNTER_ADDRESS); //Pointer to complete packet structure
    volatile struct ECAP ECAP; //structure to determine edges
    *packet_address = 0; //packet address is 0 when no counter packets are ready to be sent, and then 1 or 2 depending on which packet is ready
    ECAP.p_sample = 0; //initial previous sample is 0 so the code recognizes the first edge as a rising edge
    ECAP.ts = *IEP_TMR_CNT;
    ECAP.p_ts = *IEP_TMR_CNT;
    ECAP.trigger = 1 << 14; //Currently not used, thought I could use to determine rising/falling edge
    int i = 0; //Variable used to write to two different blocks of memory allocated to an individual instantiation of counter struct
    int x = 0; //Variable used to write to entirety of counter struct; one struct contains 150 edges
    *on = 0; //on = 0 means PRUs are executing code, 1 means they are done
    *counter_overflow = 0; //resets the overflow variable when code starts
    Data_Packets[0].counter_info_header = 0x1eaf; //header for counter packet 0
    Data_Packets[1].counter_info_header = 0x1eaf; //header for counter packet 1
    while(*on == 0){ //IRIG controls on variable so when IRIG code has sampled a certain amount of seconds it will set *on to 1
        i = 0;
        while(i < 2){
            x = 0;
            quad_encoder_needed = 1; //variable stating that the quadrature pins need to be read for the current packet still

            while(x < 150){
                //Samples the input register for bit 10(pin P8_28), bit 8(pin P8_27) bit 9(pin P8_29) and bit 11(pin P8_30)
                unsigned long int sample = (__R31 & ((1 << 10) + (1 << 8) + (1 << 9) + (1 << 11)));
                /*edge_sample samples bit 10(pin P8_28) because this is the pin used for the encoder signal not the quadrature
                 * so this sample is used to detect edges only on that one input and ignores the quadrature*/
                unsigned long int edge_sample = (__R31 & (1 << 10));
                // compares the encoder bit sample to the previous one and if there is a change, an edge occured
                unsigned long int change = edge_sample ^ ECAP.p_sample;
                if (change){
                        input_capture_count += 1; //increments number of edges that have been detected

                        if ((edge_sample & 1 << 10) && quad_encoder_needed){ //if quadrature needs to be read and its a rising edge
                            Data_Packets[i].Quad.encoder_value_2 = ((1 << 8) & (sample)) >> 8; //Reading value of quad encoder pins
                            Data_Packets[i].Quad.encoder_value_3 = ((1 << 9) & (sample)) >> 9;
                            Data_Packets[i].Quad.encoder_value_4 = ((1 << 11) & (sample)) >> 11;
                            quad_encoder_needed = 0; //sets variable that quad reading is no longer needed, is changed when next packet begins to be written to
                        }
                        ECAP.p_ts = ECAP.ts; //stores current time stamp as previous time stamp
                        ECAP.ts = *IEP_TMR_CNT; //updates time stamp
                        ECAP.trigger = ECAP.trigger ^ 1 << 14; //trigger isn't ever used, can probably remove it
                        ECAP.p_sample = edge_sample; //stores current sample of encoder bit as previous sample
                        Data_Packets[i].Counter_Packets.clock_cnt[x] = ECAP.ts; //writes time stamp to counter struct
                        //writes the number of overflows to counter struct
                        Data_Packets[i].Counter_Packets.counter_ovflow[x] = *counter_overflow + ((*IEP_TMR_GLB_STS & 1) && (Data_Packets[i].Counter_Packets.clock_cnt[x] < MAX_LOOP_TIME));
                        //writes number of edges detected to counter struct
                        Data_Packets[i].Counter_Packets.encoder_cnt[x] = input_capture_count;
                        x += 1;
                }

            }

            *packet_address = (i + 1); //sets packet identifier variable to 1 or 2 and to notify ARM a packet is ready
            i += 1;

        }



    }

    __R31 = 40;
    __halt();
}
