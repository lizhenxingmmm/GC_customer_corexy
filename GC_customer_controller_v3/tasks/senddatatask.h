#ifndef __SEND_DATA_TASK__
#define __SEND_DATA_TASK__

#define FRAME_HEADER_LENGTH 5 
#define CMD_ID_LENGTH 2       
#define DATA_LENGTH 30        
#define FRAME_TAIL_LENGTH 2   

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) 

#define CONTROLLER_CMD_ID 0x0302 

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              
        uint16_t data_length;     
        uint8_t seq;              
        uint8_t crc8;             
    } frame_header;               
    __packed uint16_t cmd_id;     
    __packed uint8_t data[30];    
    __packed uint16_t frame_tail; 
} Controller_t;                 

#endif
