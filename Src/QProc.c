#include "QProc.h"


CQData queue[QID_COUNT];
void initQueue(QID id)
{        
    queue[id].front = queue[id].rear = -1;
    //memset(queue.data.buffer, 0, sizeof(queue.data.buffer));
}

bool isFull(QID id) 
{
    return (queue[id].front == 0 && queue[id].rear == QUEUE_SIZE - 1) || (queue[id].front == queue[id].rear + 1);
}

bool isEmpty(QID id) 
{
    return queue[id].front == -1;
}

bool enqueue(QID id, byte data)
{
    if (isFull(id)) 
        return false;

    if (queue[id].front == -1) 
    {
        queue[id].front = 0;
        queue[id].rear = 0;
    } 
    else if (queue[id].rear == QUEUE_SIZE - 1) 
    {
        queue[id].rear = 0;
    } 
    else 
    {
        queue[id].rear++;
    }
    queue[id].data[0].buffer[queue[id].rear] = data;
    return true;
}

bool enqueueTX(QID id, uint32_t dwDev_num, byte *item, uint16_t length)
{
    if (isFull(id)) 
        return false;

    if (queue[id].front == -1) 
    {
        queue[id].front = 0;
        queue[id].rear = 0;
    } 
    else if (queue[id].rear == QUEUE_SIZE - 1) 
    {
        queue[id].rear = 0;
    } 
    else 
    {
        queue[id].rear++;
    }
    //memset(queue.data.buffer[queue.rear], 0, TRANSMIT_SIZE);
    queue[id].data[queue[id].rear].dwDev_num = dwDev_num;
    queue[id].data[queue[id].rear].length = length;
    memcpy(queue[id].data[queue[id].rear].buffer, item, length);
    return true;
}

bool next(QID id)
{
    if (isEmpty(id))
        return false;


    if (queue[id].front == queue[id].rear) 
    {
        queue[id].front = -1;
        queue[id].rear = -1;
    } 
    else if (queue[id].front == QUEUE_SIZE - 1) 
    {
        queue[id].front = 0;
    } 
    else 
    {
        queue[id].front++;
    }
    return true;
}

bool dequeue(QID id, byte *data) 
{
    if (isEmpty(id)) 
        return false;


    *data = queue[id].data[0].buffer[queue[id].front];
    next(id);
    return true;
}

bool dequeueTX(QID id, QData *data) 
{
    if (isEmpty(id)) 
        return false;

    *data = queue[id].data[queue[id].front];
    next(id);
    return true;   
}

bool front(QID id, QData *data)
{
    if(queue[id].front == -1)
        return false;

    *data = queue[id].data[queue[id].front];
    return true;
}

bool back(QID id, QData *data)
{
    if(queue[id].rear == -1)
        return false;

    *data = queue[id].data[queue[id].rear];
    return true;
}

int getIndex(QID id)
{
    return queue[id].front;
}

size_t getQueueSize(QID id) 
{
    if (isEmpty(id)) 
        return 0;
    else if (queue[id].front <= queue[id].rear)
        return queue[id].rear - queue[id].front + 1;
    else 
        return QUEUE_SIZE - queue[id].front + queue[id].rear + 1;
}

void initAllQueue()
{
    for(int i = 0; i < QID_COUNT; i++)
    {
        initQueue(i);
    }
}

CQueue createQueue()
{
    CQueue q;
    q.init = initQueue;
    q.isFull = isFull;
    q.isEmpty = isEmpty;

    q.enqueue = enqueue;
    q.enqueueTX = enqueueTX;

    q.dequeue = dequeue;
    q.dequeueTX = dequeueTX;

    q.front = front;
    q.back = back;
    q.next = next;
    q.size = getQueueSize;
    q.getIndex = getIndex;

    initAllQueue();
    
    return q;
}

// int main()
// {
//     CQueue q = createQueue(QUSB_TX);
    
//     return 0;
// }