//Used to load code for detecting Encoder and IRIG signals onto PRUs
//Encoder code is loaded onto PRU1 and IRIG code onto PRU0
//
// Usage:
// $ ./loader Encoder1.bin Encoder2.bin IRIG1.bin IRIG2.bin
//
// Compile with:
// gcc -o loader loader.c -lprussdrv


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <prussdrv.h> //PRU Subsystem Driver (installed from this package: https://github.com/beagleboard/am335x_pru_package)
#include <pruss_intc_mapping.h> //PRU Subsystem Interupt Controller Mapping (installed from this package: https://github.com/beagleboard/am335x_pru_package)
#include <string.h>
//The rest of these libraries are for UDP service
#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//arbitrary port used for UDP
#define PORT 8080


//Below variables are defined in pruss_intc_mapping and prussdrv, they are mapping interrupts from PRUs to ARM processor
#define PRUSS_INTC_CUSTOM {   \
  { PRU0_PRU1_INTERRUPT, PRU1_PRU0_INTERRUPT, PRU0_ARM_INTERRUPT, PRU1_ARM_INTERRUPT, ARM_PRU0_INTERRUPT, ARM_PRU1_INTERRUPT,  24, (char)-1  },  \
  { {PRU0_PRU1_INTERRUPT,CHANNEL1}, {PRU1_PRU0_INTERRUPT, CHANNEL0}, {PRU0_ARM_INTERRUPT,CHANNEL2}, {PRU1_ARM_INTERRUPT, CHANNEL3}, {ARM_PRU0_INTERRUPT, CHANNEL0}, {ARM_PRU1_INTERRUPT, CHANNEL1}, {24, CHANNEL3}, {-1,-1}},  \
  {  {CHANNEL0,PRU0}, {CHANNEL1, PRU1}, {CHANNEL2, PRU_EVTOUT0}, {CHANNEL3, PRU_EVTOUT1}, {-1,-1} },  \
  (PRU0_HOSTEN_MASK | PRU1_HOSTEN_MASK | PRU_EVTOUT0_HOSTEN_MASK | PRU_EVTOUT1_HOSTEN_MASK) /*Enable PRU0, PRU1, PRU_EVTOUT0 */ \
}

//Definining the offsets from the start of shared memory for the structures and variables used by PRUs
#define ENCODER_COUNTER_SIZE 150
#define COUNTER_OFFSET 6
#define ON_OFFSET 2
#define IRIG_OFFSET 1558
#define IRIG_IDENTIFIER_OFFSET 1556
#define ERROR_IDENTIFIER_OFFSET 3072
#define ERROR_OFFSET 3074

//function to return pointer to shared memory, needs to be typecasted everytime you want to use a datatype that is not only a 32 bit integer
volatile int32_t* init_prumem()
{
	volatile int32_t* p;
	prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void**)&p);
	return p;
}

//Structure containing clock information of encoder and absolute count
struct CounterInfo{
    unsigned long int clock_cnt[ENCODER_COUNTER_SIZE];
    unsigned long int counter_ovflow[ENCODER_COUNTER_SIZE];
    unsigned long int encoder_cnt[ENCODER_COUNTER_SIZE];
};

//Structure containing values of quadrature pins
struct QuadEncoder{
    unsigned long int encoder_value_2;
    unsigned long int encoder_value_3;
    unsigned long int encoder_value_4;
};

//Complete encoder structure comining two above structures and adding a header
struct CompleteDataPackets{
    unsigned long int counter_info_header;
    volatile struct CounterInfo Counter_Packets;
    volatile struct QuadEncoder Quad;
};

//Complete structure for IRIG information
struct IrigInfo{
    unsigned long int random_header;
    unsigned long int rising_edge_time;
    unsigned long int init_overflow;
    unsigned long int info[10];
    unsigned long int re_count[10];
    unsigned long int re_count_overflow[10];
};

//Structure for error packets, only sent when IRIG isn't synched
struct ErrorInfo{
    unsigned int header;
    unsigned int err_code;
};

int main(int argc, char **argv) {
  
  system("./pinconfig"); //runs a bash file to configure the pins needed
  
  //checks that the file is executed with correct arguments passed
  if (argc !=5) {
    printf("Usage: %s loader Encoder1.bin Encoder2.bin IRIG1.bin IRIG2.bin\n", argv[0]);
    return 1;
  }

  prussdrv_init(); //initializes the PRU subsystem driver

  //allows the use of interrupt: PRU_EVTOUT_1; currently only used to notify the ARM that the PRUs have finished executing their code
  if (prussdrv_open(PRU_EVTOUT_1) == -1) {
    printf("prussdrv_open() failed\n");
    return 1;
  }

  //functions to map and initialize the interrupts defined above
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_CUSTOM;
  prussdrv_pruintc_init(&pruss_intc_initdata);

  
  
  //Creating pointers to all shared variables and data structures in shared memory
  volatile unsigned long int* packet_address = (volatile unsigned long int *) init_prumem(); //pointer to variable to identify which encoder packet is ready to be written to UDP
  volatile unsigned long int* on = (volatile unsigned long int *) (init_prumem()+ON_OFFSET); //pointer to variable to let the ARM know that the PRUs are still executing code
  volatile unsigned long int* irig_identifier = (volatile unsigned long int *) (init_prumem() + IRIG_IDENTIFIER_OFFSET); //pointer to varialbe to identify which IRIG packet is ready to be written to UDP
  volatile unsigned long int* error_identifier = (volatile unsigned long int *) (init_prumem() + ERROR_IDENTIFIER_OFFSET);//pointer to variable to identify that an error packet is ready to be writtento UDP
  volatile struct IrigInfo* irig_packets = (volatile struct IrigInfo *) (init_prumem() + IRIG_OFFSET); //pointer to data structure for IRIG packets
  volatile struct ErrorInfo* error_state = (volatile struct ErrorInfo *) (init_prumem() + ERROR_OFFSET); //pointer to data structure for error packets
  volatile struct CompleteDataPackets* Data_Packets = (volatile struct CompleteDataPackets *) (init_prumem()+COUNTER_OFFSET); //pointer to data structure for encoder/counter packets

  //resets identifier variables to 0 meaning nothing is ready to be sent
  *packet_address = 0;
  *irig_identifier = 0;

  //following bit of code creates socket to write UDP packets with
  int sockfd;

  struct sockaddr_in servaddr;
	
  if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	perror("socket creation failed");
	exit(EXIT_FAILURE);
  }

  memset(&servaddr, 0, sizeof(servaddr));

  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(PORT);
  inet_pton(AF_INET, "192.168.2.54", &(servaddr.sin_addr.s_addr));

  //sets memory to be used by data structures to 0
  
  memset((struct IrigInfo *) &irig_packets[0], 0, sizeof(*irig_packets));
  memset((struct IrigInfo *) &irig_packets[1], 0, sizeof(*irig_packets));
  memset((struct CompleteDataPackets *) &Data_Packets[0], 0, sizeof(*irig_packets));
  memset((struct CompleteDataPackets *) &Data_Packets[1], 0, sizeof(*irig_packets));

  //following bit of code loads .bin files into their respective PRUs and executes
  printf("Executing program on PRU1 and waiting for termination\n");
  if (argc > 2) {
    if (prussdrv_load_datafile(1, argv[2]) < 0) {
      fprintf(stderr, "Error loading %s\n", argv[2]);
      exit(-1);
    }
  }
  if (prussdrv_exec_program(1, argv[1]) < 0) {
    fprintf(stderr, "Error loading %s\n", argv[1]);
    exit(-1);
  }

  printf("Executing program on PRU0 and waiting for termination\n");
  if (argc == 5) {
    if (prussdrv_load_datafile(0, argv[4]) < 0) {
      fprintf(stderr, "Error loading %s\n", argv[4]);
      exit(-1);
    }
  }
  if (prussdrv_exec_program(0, argv[3]) < 0) {
    fprintf(stderr, "Error loading %s\n", argv[3]);
    exit(-1);
  }

  //continuously loops while PRUs are still executing code and checks if data structures are ready to be written to UDP
  while(*on != 1) {
  	if(*packet_address != 0) {
        	unsigned long int offset = *packet_address - 1;
		sendto(sockfd, (struct CompleteDataPackets *) &Data_Packets[offset], sizeof(*Data_Packets), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
        	*packet_address = 0;
  	}
	if(*irig_identifier != 0) {
		unsigned long int y = *irig_identifier - 1;
		sendto(sockfd, (struct IrigInfo *) &irig_packets[y], sizeof(*irig_packets), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
		*irig_identifier = 0;
	}
	//error packets currently not used, should be able to in the future
	/*if(*error_identifier != 0) {
		unsigned long int z = *error_identifier - 1;
		sendto(sockfd, (struct ErrorInfo *) &error_state[z], sizeof(*error_state), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
		*error_identifier = 0;
	}*/
  }
    
  
  //disables PRUs when they are done executing code
  if(*on == 1) {
  	prussdrv_pru_wait_event(PRU_EVTOUT_1);
        printf("All done\n");
        prussdrv_pru_disable(1);
	prussdrv_pru_disable(0);
        prussdrv_exit();
  }
   
 
  return 0;
}
