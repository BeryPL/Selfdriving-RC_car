

//Projekt pojazdu sterowanego Bluetooth z funkcja jazdy autonomicznej

#include "MKL05Z4.h"
#include "uart0.h"
#include "TPM.h"
//#include "lcd1602.h"
#include "Queue.h"
#include "Pins.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
 
#define MOD	52429  											// Stala do okreslania wypelnienie sygnal�w PWM


void Pomiar(void);											// Deklaracja uzywanych funkcji
void delay(void);
void Pomiar_delayed(void);

Q_T RxQ;  															//Utworzenie kolejki RxQ do odczytu danych z UART
uint8_t buff[3]={1,2,3};  							//Bufor do odczytu danych z kolejki

volatile float spofsound = 0.0001715; 	// Predkosc dzwieku w us dzielone na 2
volatile float PeriodofTPM1 = 0.76;   	// Okres licznika TPM1 w us
volatile float Dist = 1.0;							// Zmienne do wyliczenia odleglosci
volatile uint16_t tpmFirst;
volatile uint16_t tpmSecond;
volatile uint16_t tpmDiff;
volatile uint8_t InterFlag = 0; 				// Flaga do kontroli kolejnosci przerwan
volatile uint8_t PWMFLAG = 0;   				// Flaga do kontroli impuls�w
volatile uint8_t Measured = 0;					// Flaga do wykonywania pomiaru
volatile uint8_t SysTickControl = 1;		// Flaga do kontroli funkcjonalnosci SysTicka
volatile uint8_t Delay = 0;							// Flaga do kontroli op�znienia

uint8_t sekunda=0;											// Zmienna do liczenia czasu
uint8_t delayduration;									// Zmienna do okreslania czasu trwania op�znienia

void TPM1_IRQHandler(void){
	if((TPM1->STATUS | TPM_STATUS_CH0F_MASK)&&PWMFLAG){ 				// Pierwsze przerwanie od PWM z CH0; Zapewnia generacje pojedynczego impulsu 10us
		TPM1->CONTROLS[0].CnSC &= ~TPM_CnSC_CHIE_MASK;  					// Wylacz przerwania od TPM1 CH0
		TPM1->CONTROLS[0].CnV = 0; 																// Wylacza PWM na TPM1 CH0
		PWMFLAG = 0; 																							// Zapewnia generacje tylko jednego impulsu. Bez niej powstaje kilka impuls�w jeden po drugim
	}else if (TPM1->STATUS | TPM_STATUS_CH1F_MASK){							// Przerwanie z TPM1 CH1
		if(InterFlag==0){																					// Pomiar dla zbocza narastajacego
			tpmFirst = TPM1->CONTROLS[1].CnV & 0xFFFF;				
			InterFlag = 1;																					// Zmiana dzialania funkcji dla kolejnego przerwania
		} else{																										// Pomiar dla zbocza opadajacego i obliczenie odleglosci
			tpmSecond = TPM1->CONTROLS[1].CnV & 0xFFFF;
			InterFlag = 0;																					// Reset dzialania przerwania
			TPM1->SC &= ~TPM_SC_CMOD(1);														// Wylaczenie TPM1, zapewnia tylko jeden pomiar na przerwanie
			tpmDiff = tpmSecond - tpmFirst;
			Dist = (tpmDiff * PeriodofTPM1 * spofsound);						// Obliczenie odleglosci
			Measured = 1;																						// Potwierdzenie dokonania pomiaru
			if(SysTickControl){
				UART0->C2 |= UART0_C2_RIE_MASK;		 										// Wlacz przerwania od odbiornika
				UART0->C2 |= UART0_C2_RE_MASK;												// Wlacz nadajnik i odbiornik
				NVIC_ClearPendingIRQ(UART0_IRQn);
				NVIC_EnableIRQ(UART0_IRQn);
			}
		}
	}
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;          			// Kasowanie flag przerwan
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHF_MASK;
}

void UART0_IRQHandler(void) {  																// Odczyt danych z UART do kolejki RxQ
	if (UART0->S1 & UART0_S1_RDRF_MASK) {
		if (!Q_Full(&RxQ)) {
			Q_Enqueue(&RxQ, UART0->D);
		} 
		else{ 
			buff[0]=1;
			buff[1]=1;
			buff[2]=1;
		}
	}
}

void SysTick_Handler(void){  																	// Przerwania od SysTick do pomiaru odleglosci w r�wnych odstepach czasowych
	sekunda+=1;
	if((sekunda==5) && (SysTickControl)) {           						// Praca w trybie wyzwalania pomiaru odleglosci
		UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );			// Blokada nadajnika i o dbiornika
		NVIC_DisableIRQ(UART0_IRQn);
		sekunda=0;
		Pomiar();																									// Wyzwolenie pomiaru
	}
	if((!SysTickControl) && (sekunda==delayduration)){					// Praca w trybie sterowanego op�znienia
		SysTick_Config(1);
		Delay = 0;
		sekunda=0;
	}
}

void Pomiar(void){																						// Wywolanie pomiaru odleglosci
	TPM1->SC &= ~TPM_SC_CMOD(1);																// Upwenia sie ze licznik jest wylaczony
	PWMFLAG = 1;																								// Konfiguracja i reset parametr�w licznika przed pomiarem
	TPM1->CNT = 0;
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHIE_MASK;
	TPM1->CONTROLS[0].CnV = 0x000D;
	TPM1->SC |= TPM_SC_CMOD(1);																	// Uruchomienie TPM1 r�wnoznaczne z wyzwoleniem pomiaru odleglosci
}

void delay(void){																							// Funkcja op�zniajaca dzialanie programu z pomoca SysTicka
	Delay = 1;
	SysTick_Config(SystemCoreClock/10);
	while(Delay);
}

void Pomiar_delayed(void){																		// Funkcja dokonujaca pomiaru z wstrzymaniem dzialania 
	Measured = 0; 																							// programu do momentu potwierdzenia dokonania pomiaru
	Pomiar();
	while(!Measured);
}

int main (void) {
	
	UART0_Init();                        												// Inicjalizacja peryferi�w
	PWM_Init();
	Pin_Init();
	SysTickControl = 1;
	SysTick_Config(SystemCoreClock/10);  												// Uruchomienie SysTicka w trybie pomiarowym
	
	uint16_t Driving_Infill;																		//Zmienne do sterowania jazda
	float Infill_Coeff_Left;
	float Infill_Coeff_Right;
	float LeftDist;
	float RightDist;
	float temp;

	
	while(1){                             											// Petla wstepna, na czas ustanowienia polaczenia bluetooth
		for(int i=0; i<Q_SIZE ; i++){
			if (!Q_Empty(&RxQ)) { 
				buff[i]=(uint8_t)Q_Dequeue(&RxQ);
			}			
		}	
		if(buff[0] != 1){																					// Po odbiorze do bufora danych innych niz inicjalizujace bufor
			break;																									// przerywa petle i wchodzi do petli glownej
		}
	}
	
	while(1){																											// Petla gl�wna
		
		for(int i=0; i<Q_SIZE ; i++){             									// Odczyt danych z kolejki do bufora
			if (!Q_Empty(&RxQ)) {	
				buff[i]=(uint8_t)Q_Dequeue(&RxQ);
			}			
		}	

		if(!(buff[2] & 0x80)) {																			// Jazda sterowana z aparatury
			Infill_Coeff_Left = 1;																		// Reset wsp�lczynnik�w r�znicujacych wypelnienie sygnal�w PWM
			Infill_Coeff_Right = 1;
			if(buff[1] > 137){																				// Okreslenie kierunku skretu, ustawienie wsp�lczynnik�w do r�znicy w wypelnieniu PWM
				Infill_Coeff_Right = (float)(256 - buff[1])/118.0;
			}
			else if(buff[1] < 119){
				Infill_Coeff_Left = (float)(buff[1]/118.0);
				if(buff[1] == 0){
					Infill_Coeff_Left = 0;
				}
			}
			
			if(buff[0] >= 117 && buff[0] <= 137){ 										// Zatrzymaj pojazd
					PTA->PCOR |= (1<<10);
					PTA->PCOR |= (1<<11);
					TPM0->CONTROLS[2].CnV = 0x0000;
					TPM0->CONTROLS[3].CnV = 0x0000;	
			}
			else if(buff[0] > 137){ 																	// Jazda do przodu
					Driving_Infill=(int)MOD*(buff[0]-137)/118;  					// Okreslenie predkosci jazdy 
					PTA->PSOR |= (1<<10);																	// Polaryzacja silnik�w
					PTA->PCOR |= (1<<11);    
					TPM0->CONTROLS[2].CnV = (Driving_Infill * Infill_Coeff_Right);		// Ustawienie wypelnien z poprawka na kierunek skretu
					TPM0->CONTROLS[3].CnV = (Driving_Infill * Infill_Coeff_Left);
			}   
			else if(buff[0] < 119){ 																	// Jazda do tylu
					Driving_Infill=(int)MOD*(118-buff[0])/118;  					// Okreslenie predkosci jazdy 
					PTA->PCOR |= (1<<10);																	// Polaryzacja silnik�w
					PTA->PSOR |= (1<<11);     
					TPM0->CONTROLS[2].CnV = (Driving_Infill * Infill_Coeff_Right);		// Ustawienie wypelnien z poprawka na kierunek skretu
					TPM0->CONTROLS[3].CnV = (Driving_Infill * Infill_Coeff_Left);
			}
			
		}else { 																											// Jazda autonomiczna
			PTA->PSOR |= (1<<10);																				// Rozpoczecie jazdy na wprost
			PTA->PCOR |= (1<<11);
			TPM0->CONTROLS[2].CnV = 0xCCCC;
			TPM0->CONTROLS[3].CnV = 0xCCCC;
			
			if(Dist < 0.3){																							// Wykrycie przeszkody
				
				PTA->PCOR |= (1<<10);																			// Zatrzymanie pojazdu
				PTA->PCOR |= (1<<11);
				TPM0->CONTROLS[2].CnV = 0x0000;														
				TPM0->CONTROLS[3].CnV = 0x0000;
				
				UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );			// Blokada nadajnika i o dbiornika
				NVIC_DisableIRQ(UART0_IRQn);
				
				SysTick_Config(1); 																				// Zatrzymanie SysTicka
				SysTickControl = 0;																				// Zmiana trybu pracy SysTicka na sterowane op�znienie
				
				while(1){
					
					delayduration = 10;																			// Op�znienie dla ustabilizowania warunk�w pomiarowych
					delay();
					
					TPM0->CONTROLS[5].CnV = 1400;  													// Obr�t serwa w prawo
					delayduration = 20;
					delay();
					Pomiar_delayed();             													// Pomiary z prawej strony
					temp = Dist;
					Pomiar_delayed();
					RightDist = (Dist + temp) * 0.5; 												// Usrednianie

					TPM0->CONTROLS[5].CnV = 6400;  													// Obr�t serwa w lewo
					delayduration = 20;
					delay();
					Pomiar_delayed();																				// Pomiary z lewej strony
					temp = Dist;
					Pomiar_delayed();
					LeftDist = (Dist + temp) * 0.5; 												// Usrednianie

					TPM0->CONTROLS[5].CnV = 3550;   												// Obr�t serwa na wprost
					delayduration = 20;
					delay();

					if((LeftDist < 0.3) && (RightDist < 0.3)){  						// Decyzja o cofnieciu i ponowieniu pomiaru
						PTA->PCOR |= (1<<10);																	// Dokonuje jazdy do tylu przez 1.1s, nastepnie ponawia pomiary po obu stronach
						PTA->PSOR |= (1<<11); 
						TPM0->CONTROLS[2].CnV = 0xCCCC;
						TPM0->CONTROLS[3].CnV = 0xCCCC;
						delayduration = 11;
						delay();
						TPM0->CONTROLS[2].CnV = 0;
						TPM0->CONTROLS[3].CnV = 0;
						SysTick_Config(1);
						continue;
					}
					
					if((LeftDist > 2.0) || (RightDist > 2.0)){  						// Jazda w prawo, gdy wolne z kazdej strony (zabezpieczenie na wypadek przekroczenia maksimum pomiaru)
						PTA->PCOR |= (1<<10);																	// W przypadku wykrycia bardzo duzych odleglosci po obu stronach domyslnie kontunuuje jazde w lewo
						PTA->PSOR |= (1<<11);
						TPM0->CONTROLS[2].CnV = 0xCCCC;
						TPM0->CONTROLS[3].CnV = 0;
						delayduration = 12;
						delay();
						TPM0->CONTROLS[2].CnV = 0;
						TPM0->CONTROLS[3].CnV = 0;
						Dist = 2.0;
						break;
					}
					
					if(LeftDist >= RightDist){  														// Jazda w lewo
						PTA->PCOR |= (1<<10);																	// Dokonuje obrotu w lewo i kontunuuje jazde
						PTA->PSOR |= (1<<11);
						TPM0->CONTROLS[2].CnV = 0;
						TPM0->CONTROLS[3].CnV = 0xCCCC;
						delayduration = 12;
						delay();
						TPM0->CONTROLS[2].CnV = 0;
						TPM0->CONTROLS[3].CnV = 0;
						Dist = 2.0;
						break;
					}
					
					if(LeftDist < RightDist){  															// Jazda w prawo
						PTA->PCOR |= (1<<10);																	// Dokonuje obrotu w prawo i kontunuuje jazde
						PTA->PSOR |= (1<<11);
						TPM0->CONTROLS[2].CnV = 0xCCCC;
						TPM0->CONTROLS[3].CnV = 0;
						delayduration = 12;
						delay();
						TPM0->CONTROLS[2].CnV = 0;
						TPM0->CONTROLS[3].CnV = 0;
						Dist = 2.0;
						break;
					}
				}
				
				delayduration = 10;
				delay();
				
				PTA->PSOR |= (1<<10);
				PTA->PCOR |= (1<<11);
				TPM0->CONTROLS[2].CnV = 0xCCCC;
				TPM0->CONTROLS[3].CnV = 0xCCCC;
				
				UART0->C2 |= UART0_C2_RIE_MASK;									// Wlacz przerwania od odbiornika
				UART0->C2 |= UART0_C2_RE_MASK;									// Wlacz nadajnik i odbiornik
				NVIC_ClearPendingIRQ(UART0_IRQn);
				NVIC_EnableIRQ(UART0_IRQn);
				
				SysTickControl = 1;															// Uruchom SysTick w trybie pomiarowym
				sekunda = 0;
				SysTick_Config(SystemCoreClock/10);
			}
		}
		
		
		for(uint32_t i=0;i<(500000);i++)
		__nop();
	}	
}	
