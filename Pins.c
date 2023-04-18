#include "Pins.h"
#include "MKL05Z4.h"

void Pin_Init(){
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; //wlaczenie portów A
	PORTA->PCR[10] |= PORT_PCR_MUX(1);  //rola portu jako GPIO do kierunku jazdy
	PORTA->PCR[11] |= PORT_PCR_MUX(1);  //rola portu jako GPIO do kierunku jazdy
	PTA->PDDR |= (1<<10) | (1<<11);     //ustawiamy wyjscie
}
