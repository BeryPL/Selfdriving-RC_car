#include "TPM.h"
#include "MKL05Z4.h"
void PWM_Init()
{
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;			// Dołączenie sygnału zegara do portu B
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;			// Dołączenie sygnału zegara do portu A

	//Podłączenie pinów do kanałów liczników
	PORTB->PCR[6] = PORT_PCR_MUX(2);				// PTB6 TPM0 CH3 Lewa strona
	PORTB->PCR[7] = PORT_PCR_MUX(2);				// PTB7 TPM0 CH2 Prawa strona
	PORTA->PCR[5] = PORT_PCR_MUX(2);				// PTA5 TPM0 CH5 Serwo
	
	PORTA->PCR[0]  = PORT_PCR_MUX(2); 			// PTA0 TMP1 CH0 Trigger 
	PORTB->PCR[13] = PORT_PCR_MUX(2); 			// PTB13 TMP1 CH1 Echo
	
	//Dołączenie zegara do liczników
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;			// Dołączenie sygnału zegara do TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; 			// Dołączenie sygnału zegara do TPM1
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);			// Źródło taktowania TPMx MCGFLLCLK=41943040Hz 1

	//Główne ustawienia liczników
	TPM1->SC &= ~TPM_SC_CPWMS_MASK; 				// TPM1 w trybie zliczania w przód
	TPM1->SC |= TPM_SC_PS(5); 							// TPM1 Prescaler = 32; F = 1310720 Hz
	TPM1->MOD = 0xFFFF;											// MOD = 65535 (0xFFFF), w celu dokonania pomiaru odległości
	
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;					// TPM0 w trybie zliczanie w przód
	TPM0->SC |= TPM_SC_PS(4);								// TPM0 Prescaler = 16; F = 2621440Hz
	TPM0->MOD = 0xCCCC;											// MOD = 52428 (0xCCCC); F = 50Hz, dla kontroli jazdy i serwa 
	
	//PWM na TPM1 CH0 do wyzwolenia pomiaru
	TPM1->CONTROLS[0].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK; 			// TPM1 CH0 PWM HIGH True
	TPM1->CONTROLS[0].CnV=0x000D;  																		// Wypełnienie dla PWM o długości trwania stanu wysokiego ~10us
	
	//InputCapture na TPM1 CH1 do pomiaru odległości
	TPM1->CONTROLS[1].CnSC &= ~ (TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
	TPM1->CONTROLS[1].CnSC |= (TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK); 
	TPM1->CONTROLS[1].CnV=0;																					// Wyzerowanie CnV dla CH1

	//PWM TPM0 CH2, CH3 do kontroli prędkości i kierunku jazdy.
	TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK;		// TPM0 CH2 PWM HIGH True
	TPM0->CONTROLS[2].CnV = 0x0000;																		// CnV = 0, PWM wyłączony
	TPM0->CONTROLS[3].CnSC = TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK;		// TPM0 CH3 PWM HIGH True
	TPM0->CONTROLS[3].CnV = 0x0000;																		// CnV = 0, PWM wyłączony

	//PWM TPM0 CH5 do kontroli serwa
	TPM0->CONTROLS[5].CnSC = TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK;		// TPM0 CH5 PWM HIGH True
	TPM0->CONTROLS[5].CnV = 3550;																		  // CnV = 4100, Serwo na wprost
	
	TPM0->SC |= TPM_SC_CMOD(1);																				// Włącz licznik TPM0
	
	// Ustawienie przerwań dla licznika TPM1 i kanałów CH0, CH1
	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHIE_MASK; 										// Włączenie przerwań TPM1 CH0
	TPM1->CONTROLS[1].CnSC |= TPM_CnSC_CHIE_MASK; 										// Włączenie przerwań TPM1 CH1
	NVIC_SetPriority(TPM1_IRQn, 0);
	NVIC_ClearPendingIRQ(TPM1_IRQn);
	NVIC_EnableIRQ(TPM1_IRQn);
	
}
