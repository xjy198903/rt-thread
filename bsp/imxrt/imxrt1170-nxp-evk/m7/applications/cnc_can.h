#ifndef __CNC_CAN_H__
#define __CNC_CAN_H__

#define CAN_THREAD_PRIORITY 12
#define CAN_THREAD_TICK 5
#define CAN_STACK_SIZE 1024
#define CAN_DEVICE_NAME "can2"

void can_comm_entry(void*);



#endif