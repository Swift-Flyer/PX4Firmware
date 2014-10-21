#ifndef _CONFIG_ffff_H_
#define _CONFIG_ffff_H_

// Needed defines by Canfestival lib
#define MAX_CAN_BUS_ID 1
#define SDO_MAX_LENGTH_TRANSFER 32
#define SDO_BLOCK_SIZE 16
#define SDO_MAX_SIMULTANEOUS_TRANSFERS 1
#define NMT_MAX_NODE_ID 128
#define SDO_TIMEOUT_MS 3000U
#define MAX_NB_TIMER 8

// CANOPEN_BIG_ENDIAN is not defined
#define CANOPEN_LITTLE_ENDIAN 1


#define REPEAT_SDO_MAX_SIMULTANEOUS_TRANSFERS_TIMES(repeat)\
repeat
#define REPEAT_NMT_MAX_NODE_ID_TIMES(repeat)\
repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat repeat

#define EMCY_MAX_ERRORS 8
#define REPEAT_EMCY_MAX_ERRORS_TIMES(repeat)\
repeat repeat repeat repeat repeat repeat repeat repeat

#endif