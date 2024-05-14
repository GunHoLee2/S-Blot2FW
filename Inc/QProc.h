#ifndef	QPROC_H
#define	QPROC_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#define BUFFER_SIZE 50
#define QUEUE_SIZE 50

typedef unsigned char byte;

typedef enum 
{
    QUSB_TX,
    QUSB_RX,
    QLED_TX,    
    QLED_RX,
    QID_COUNT
}QID;

typedef struct
{
    byte buffer[BUFFER_SIZE];
    uint32_t dwDev_num;
    uint32_t length;
}QData;

typedef struct 
{
    QData data[QUEUE_SIZE];
    int front, rear;     
}CQData;

typedef struct 
{  
    void (*init)(QID id);
    bool (*isFull)(QID id);
    bool (*isEmpty)(QID id);

    bool (*enqueue)(QID id, byte data);
    bool (*enqueueTX)(QID id, uint32_t dwDev_num, byte *item, uint16_t length);
    
    bool (*dequeue)(QID id, byte *data);
    bool (*dequeueTX)(QID id, QData *data);

    bool (*front)(QID id, QData *data);
    bool (*back)(QID id, QData *data);
    bool (*next)(QID id);
    int (*getIndex)(QID id);   
    size_t (*size)(QID id);       
}CQueue;

extern CQueue createQueue();
void initAllQueue();
#endif