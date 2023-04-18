#include "Queue.h"

void Q_Init(Q_T * q) {   //Inicjalizacja kolejki
	unsigned int i;
	for (i=0; i<Q_SIZE; i++){
		q->Data[i] = 0; 
	}
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int Q_Empty(Q_T * q) {
	return q->Size == 0;
}

int Q_Full(Q_T * q) {
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T * q, char d) { 	//Zapis do kolejki
	if (!Q_Full(q)) {
		q->Data[q->Tail++] = d;   		//postinkrementacja 
		q->Tail %= Q_SIZE;
		q->Size++;
			return 1; 									// success
	} 
	else
		return 0; 										// failure
}

char Q_Dequeue(Q_T * q) {         //Odczyt z kolejki
char t=0;
	if (!Q_Empty(q)) {
		t = q->Data[q->Head];
		q->Data[q->Head++] = 0;
		q->Head %= Q_SIZE;
		q->Size--;
	}
		return t;
}
