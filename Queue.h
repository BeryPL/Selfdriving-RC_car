#ifndef QUEUE_H
#define QUEUE_H

#define Q_SIZE 3

typedef struct {
	
	unsigned char Data[Q_SIZE];
	unsigned int Head; 		// points to oldest data element
	unsigned int Tail; 		// points to next free space
	unsigned int Size; 		// quantity of elements in queue
	
} Q_T;

void Q_Init(Q_T * q);

int Q_Empty(Q_T * q);

int Q_Full(Q_T * q);

int Q_Enqueue(Q_T * q, char d);

char Q_Dequeue(Q_T * q);

#endif //QUEUE_H
