/************************************************************************************************
 This file expects a voltage between 0 and 1.20V to be connected differentially to AIN3/AIN2.
				 This file will measure the voltage between AIN3 (pin 8) as - and AIN2 (pin 9) as +
				 and send the measured voltage to the UART (115200 baud).
*************************************************************************************************/
// Bit Definitions
#define BIT0  0x01
#define BIT1  0x02
#define BIT2  0x04
#define BIT3  0x08
#define BIT4  0x10
#define BIT5  0x20
#define BIT6  0x40
#define BIT7  0x80
#define BIT8  0x100
#define BIT9  0x200
#define BIT10 0x400
#define BIT11 0x800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000
#define BIT16 0x10000
#define BIT22 0x400000
#define BIT24 0x1000000
#define BIT30 0x40000000

#include<aduc7060.h>
# include "stdio.h"
# include "string.h"

void ADC0Init(void);						
void UARTInit(void);
void SendString(void);									// Used to send strings to the UART
void delay(int);												// Simple delay
float CalculateRTDTemp (void);					// returns Thermistor Temperature reading
void SystemZeroCalibration(void);				// Calibrate using external inputs
void SystemFullCalibration(void);
void fillBuf(void);  // send a byte if there's one to send
void sendChar(char toSend);  // put a character into send buffer


// global variable declarations....
volatile unsigned char ucComRx = 0;				// variable that ComRx is read into in UART IRQ
unsigned uADC0CONThermocouple;	 				// used to set ADC0CON which sets channel to thermocouple
unsigned uADC0CONRtd;			 				// used to set ADC0CON which sets channel to RTD
unsigned char ucRTDGain;						// PGA gain used on RTD
unsigned char ucThermocoupleGain;				// PGA gain used on	thermocouple		    
volatile unsigned char bSendResultToUART = 0;	// Flag used to indicate ADC0 resutl ready to send to UART	
unsigned char szTemp[64] = "";					// Used to store ADC0 result before printing to UART
unsigned char ucTxBufferEmpty  = 0;				// Used to indicate that the UART Tx buffer is empty
unsigned char newADCdata;          				// Used to indicate that new ADC data is available
unsigned char ucWaitForUart = 0;				// Used to check that user has indicated that correct voltage was set 
volatile long lADC0_Thermocouple = 0;			// Variable that ADC0DAT is read into, this is the ADC0 reading of the thermocouple
volatile unsigned long ulADC0_RTD = 0;			// Variable that ADC0DAT is read into, this is the ADC0 reading of the RTD
unsigned char ucADCInput;						// Used to indicate what channel the ADC0 is sampling

float Rrtd = 0.0;								// Resistance of RTD
float fVRTD, fVThermocouple;					// fVRTD = RTD voltage, fVThermocouple = Thermocouple voltage
float fTThermocouple, fTRTD;					// fTThermocouple = thermocouple temperature, fTRTD = RTD(cold junction) temperature
float fFinalTemp;								// fFinalTemp = final temperature taking cold junction into account
float fVoltsBi, fVoltsUni;
unsigned char i = 0;
unsigned char nLen = 0;
signed int j = 0;
unsigned char sendBuf[512];  // serial output buffer
const int sendBufSize = 512; // serial output buffer size (for wrapping)
int sendBufIndex = 0;				 // serial output buffer index used by sendChar
int sendBufUartIndex = 0;	   // serial output buffer index used by UART IRQ handler

int main(void)
{		
	POWKEY1 = 0x1;
	POWCON0 = 0x78;		   				// Set core to max CPU speed of 10.24Mhz
	POWKEY2 = 0xF4;
//	IEXCON = 0x42; 						// Enable Excitation Current Source 0 - 200uA
//	DACCON = 0x10;						// Enable DAC with internal reference
//	DACDAT = ((0xB55)<< 16);			// Set DAC to 850 mV which is used bias negative input of thermocouple
	UARTInit();							// Init UART
	ADC0Init();							// Init ADC0
	IRQEN = BIT11 + BIT10;						// Enable UART interrupt (BIT11) and ADC0 interrupt (BIT10)
	
	// uADC0CONRtd = 0x8415;				// Gain = 32, Unipolar, enable ADC0, Ext ref, ADC0/1 differential
	// uADC0CONThermocouple = 0x8145;		// Gain = 32, Bipolar, enable ADC0, Int ref, ADC2/3 differential
	// bit 15 = ADC ON, 14:13 current source 00, 12 HIGHEXTREF, 11 AMP_CM, 10 unipolar, 9:6 input select,
	// bit 5:4 reference select, 3:0 PGA gain select 0000=gain of 1.  page 45 of PDF
	// ADC0CON = 0x8540 ;	//  ADC on, ADC2/ADC3 (differential mode), Int ref, gain = 1
	ADC0CON = BIT15 + BIT8 + BIT6 + BIT4 + BIT5;	//  ADC on, ADC2/ADC3 (differential mode), Vdd/2 ref, gain = 1
	ADCMDE  = 0x81;								// ADCMDE bit 7 = fullpower, bits 2:0 = 001 continous conversion mode
	
	ucThermocoupleGain = 32;			// Need to change these values according to the PGA gain set in ADC0CON
	ucRTDGain = 32;						 
	fVoltsUni = 1.2 / 16777216;		// Volts per ADC unit in Unipolar mode
	fVoltsBi = 2.4 / 16777216;		// Volts per ADC unit in Biipolar mode

	while (1)
	{
	   	if (newADCdata == 1) 			// if new ADC data is available
	   	{
				fVThermocouple = lADC0_Thermocouple * fVoltsUni;
				newADCdata = 0;								// Indicate that data has been read
//			sprintf ( (char*)szTemp, "Voltage : \t%+8.6ffV \r\n",fVThermocouple );// Send the ADC0 Result to the UART                          
//				sprintf ( (char*)szTemp, "%+8.6fV \r\n",fVThermocouple );
				sprintf((char*)szTemp, "%07.7LX\r\n",lADC0_Thermocouple );  // pad left with zeroes, 6 width, 6 precision, Long Double, HEX
				nLen = strlen((char*)szTemp);
     		if (nLen <64)	SendString();
//				sprintf((char*)szTemp, "123456r\n");
//				nLen = strlen((char*)szTemp);
//     		if (nLen <64)	SendString();
				delay(100000);
   		}
	}
}

void ADC0Init()
{
	ADCMSKI = BIT0;						// Enable ADC0 result ready interrupt source
  // ADCFLT = 0xFF1F;					// Chop on, Averaging, AF=63, SF=31, 4Hz					
//	ADCFLT = BIT14;  // Bit 14 = RAVG2 running average /2, sample rate = 8kHz
	ADCFLT = 7;  //		Sinc3 factor of 64, chop off, ravg2 off
	
	ADCORCR = 3;
 	ADCCFG = 0;
	//ADC0CON = 0x8145;					// For system calibration set the gain that will be used
										// for measurements to ensure the best calibration is achieved,
										// In this case gain is 32 therefore Full Scale is 0.0375v
	//SystemZeroCalibration();			// For best results use a system zero scale calibration 
	//SystemFullCalibration();			// and a system full calibration instead of the self calibrations
										// AIN -ne has to be biased using the DAC
	ADC0CON = BIT10 + BIT15;	    	// Gain = 1, Unipolar, enable ADC0, Int Reference
	ADCMDE  = 0x94;						// Offest Self Calibration
  	while((ADCSTA & BIT15) == BIT15){}	// Wait for Calibration routine to complete
	ADCMDE  = 0x95;						// Gain Self Calibration
  	while((ADCSTA & BIT15) == BIT15){}	// Wait for Calibration routine to complete
}
void UARTInit()
{
// Initialize the UART for 9600-8-N
	GP1CON = 0x11;  			// Select UART functionality for P1.0/P1.1
	COMCON0 = 0x80;				// Enable access to COMDIV registers
	// baud rate = 320000 / ( DL * (M + (N / 2048)))
	// 9600 baud => DL = 0x21, M = 1, N = 21
	// 19200 baud => DL = 0x10, M = 1, N = 85
	// 115200 baud => DL = 0x02, M = 1, N = 796
	COMDIV0 = 0x02;				// DL low byte (Divisor Latch)
	COMDIV1 = 0x00;				// DL high byte (Divisor Latch)
	// COMDIV2 Bit 15 = Fractional Baud enable, bits 12:11 is M (00=4), bits 10:0 = N (page 80)
	// COMDIV2 = BIT15 + BIT11 + 21;	  // 9600 baud if COMDIV = 0x21
	// COMDIV2 = BIT15 + BIT11 + 85;	  // 19200 baud if COMDIV = 0x10
	COMDIV2 = BIT15 + BIT11 + 796;	  // 115200 baud if COMDIV = 0x02
	
	COMCON0 = BIT0 + BIT1 + BIT2;	// 8 data bits 2 stop bits
	COMIEN0 = BIT0 + BIT1;	 	// Enable UART interrupts when Rx full and Tx buffer empty.	
}
void SendString (void)
{
   	for ( i = 0 ; i < nLen ; i++ ) sendChar(szTemp[i]);
}
void sendChar(char toSend)
{
	if (((sendBufIndex + 1) % sendBufSize) == sendBufUartIndex) return;  // buffer is full
	sendBuf[sendBufIndex++] = toSend;  // put the char in the buffer and ++ the index
	sendBufIndex %= sendBufSize; // wrap buffer index if needed
	if (0x020==(COMSTA0 & 0x020)) fillBuf();  //if we can, send it now.
}
void fillBuf() {  // send a byte if there's one to send
			if (sendBufUartIndex == sendBufIndex) return;  // nothing to send
				COMTX = sendBuf[sendBufUartIndex++];  // send the next char and ++ the index
				sendBufUartIndex %= sendBufSize; // wrap buffer index if needed
}
void IRQ_Handler(void) __irq
{
	volatile unsigned long IRQSTATUS = IRQSTA;	   					// Read off IRQSTA register
	if ((IRQSTATUS & BIT11) == BIT11)		//UART interrupt source
	{
	  unsigned char ucCOMIID0 = COMIID0;  // read serial port status register
		//if ((ucCOMIID0 & 0x2) == 0x2)	  	// Transmit buffer is empty
		if (0x020==(COMSTA0 & 0x020))
		fillBuf();  // send a byte if there's one to send
		if ((ucCOMIID0 & 0x4) == 0x4)	  			// A byte has been received
		{
			ucComRx	= COMRX;
			ucWaitForUart = 0;
		} 	
	}

	if ((IRQSTATUS & BIT10) == BIT10)		//If ADC0 interrupt source
		{
		lADC0_Thermocouple = ADC0DAT;	// Read ADC0 conversion result
		newADCdata = 1;
  	}
	}

// Simple Delay routine
void delay (int length)
{
	while (length >0)
    	length--;
}

void SystemZeroCalibration(void)
{
	
	ucWaitForUart = 1;
	sprintf ( (char*)szTemp, "Set Zero Scale Voltage - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	while (ucWaitForUart == 1)
	{}
	ADCMDE	= 0x96;							// ADC System Zero scale calibration
	while ((ADCSTA & BIT15) != BIT15)		// bit 15 set by adc when calibration is complete
	{}
}
void SystemFullCalibration(void)
{
	ucWaitForUart = 1;
	sprintf ( (char*)szTemp, "Set Full Scale Voltage (0.0375) - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	while (ucWaitForUart == 1)
	{}
	ADCMDE	= 0x97;							// ADC System Full scale calibration
	while ((ADCSTA & BIT15) != BIT15)		// bit 15 set by adc when calibration is complete
	{}
}

float CalculateRTDTemp ()
{
  	float fresult;
  
  	// First calculate the resistance across the RTD
  	// Equation = ADC0Result * ((RREF/Gain)/# of bits)
  	Rrtd = (float)ulADC0_RTD * ((5600.0 /ucRTDGain) /0xFFFFFF);

  	// Next the temperature value is calculated using Rtd resistance
  	// Simple Linear equation:
  	// At 35 degree Celsius, the equivalent resistance of a PT100 is 113.607 Ohm.
  	fresult = (Rrtd - 100.0) * (35.0/13.607);
  	return fresult;
}

