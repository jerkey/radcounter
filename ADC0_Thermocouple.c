/************************************************************************************************

 Author        : ADI - Apps            www.analog.com/AduC7060

 Date          : November 2007

 File          : ADC_Cont.c

 Hardware      : ADuC706x

 Description   : This file expects a Thermocouple to be connected differentially to AIN2/AIN3.
				 The RTD connected to AIN0/AIN1 will be used for Cold Junction compensation.
				 This file will measure the thermocouple/RTD inputs and send the measured voltages and
				 temperature to the UART (9600 baud).
				 To measure the RTD, the following switch settings must be in place:
				 S4-1  			-	ON	
				 S4-2  			-	ON
				 S4-3  			-	ON
				 S4-4  			- 	ON
				 S4-5 to S4-8 	-	OFF
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

#define THERMOCOUPLE 	0						// Used for switching ADC0 to Thermocouple channel
#define RTD 			1						// Used for switching ADC0 to RTD channel
#define SAMPLENO		0x8					// Number of samples to be taken between channel switching

// Thermocouple temperature constants
#define TMINP 0  		// = minimum positive temperature in degC 
#define TMAXP 350  		// = maximum positive temperature in degC
#define VMINP 0  		// = input voltage in mV at 0 degC
#define VMAXP 17.819  	// = input voltage in mV at 350 degC
#define NSEGP 30  		// = number of sections in table
#define VSEGP 0.59397  	// = (VMAX-VMIN)/NSEG = Voltage VSEG in mV of each segment

#define TMINN 0  		// = minimum negative temperature in degC
#define TMAXN -200  	// = maximum negative temperature in degC
#define VMINN 0  		// = input voltage in mV at 0 degC
#define VMAXN -5.603  	// = input voltage in mV at -200 degC
#define NSEGN 20  		// = number of sections in table
#define VSEGN -0.28015  // = (VMAX-VMIN)/NSEG = Voltage VSEG in mV of each segment

// thermocouple lookup tables....
const float C_themocoupleP[NSEGP+1] = {0.0, 	15.1417, 	29.8016, 	44.0289, 	57.8675, 	71.3563, 
									84.5295, 	97.4175, 	110.047, 	122.441, 	134.62,		146.602, 
									158.402, 	170.034, 	181.51, 	192.841, 	204.035, 	215.101, 
									226.046, 	236.877, 	247.6,		258.221, 	268.745, 	279.177, 
									289.522, 	299.784, 	309.969, 	320.079, 	330.119, 	340.092, 
									350.001};
const float C_themocoupleN[NSEGN+1] = {0.0,		-7.30137,	-14.7101,	-22.2655, 
									-29.9855, 	-37.8791, 	-45.9548, 	-54.2258, 
									-62.7115, 	-71.4378, 	-80.4368,	-89.7453, 
									-99.4048, 	-109.463, 	-119.978, 	-131.025, 
									-142.707, 	-155.173, 	-168.641, 	-183.422, 
									-199.964};

void ADC0Init(void);						
void UARTInit(void);
void SendString(void);							// Used to send strings to the UART
void delay(int);								// Simple delay
float CalculateRTDTemp (void);					// returns Thermistor Temperature reading
float CalculateThermoCoupleTemp(void);			// returns Thermocouple Temperature reading
void SystemZeroCalibration(void);				// Calibrate using external inputs
void SystemFullCalibration(void);

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
unsigned char ucSaveADCResult = 0x0;			// Counter used to indicate when to save the ADC0 reading
unsigned char ucADCInput;						// Used to indicate what channel the ADC0 is sampling

float Rrtd = 0.0;								// Resistance of RTD
float fVRTD, fVThermocouple;					// fVRTD = RTD voltage, fVThermocouple = Thermocouple voltage
float fTThermocouple, fTRTD;					// fTThermocouple = thermocouple temperature, fTRTD = RTD(cold junction) temperature
float fFinalTemp;								// fFinalTemp = final temperature taking cold junction into account
float fVoltsBi, fVoltsUni;
unsigned char i = 0;
unsigned char nLen = 0;
signed int j = 0;
unsigned char sendBuf[512];

int main(void)
{		
	POWKEY1 = 0x1;
	POWCON0 = 0x78;		   				// Set core to max CPU speed of 10.24Mhz
	POWKEY2 = 0xF4;
	IEXCON = 0x42; 						// Enable Excitation Current Source 0 - 200uA
	DACCON = 0x10;						// Enable DAC with internal reference
	DACDAT = ((0xB55)<< 16);			// Set DAC to 850 mV which is used bias negative input of thermocouple
	UARTInit();							// Init UART
	IRQEN = BIT11;						// Enable UART interrupt
	ADC0Init();							// Init ADC0
	uADC0CONRtd = 0x8415;				// Gain = 32, Unipolar, enable ADC0, Ext ref, ADC0/1 differential
	uADC0CONThermocouple = 0x8145;		// Gain = 32, Bipolar, enable ADC0, Int ref, ADC2/3 differential
	ADC0CON = uADC0CONThermocouple ;	// Set ADC channel to Thermocouple				
	ucThermocoupleGain = 32;			// Need to change these values according to the PGA gain set in ADC0CON
	ucRTDGain = 32;						 
	ucADCInput = THERMOCOUPLE;			// Indicate that ADC0 is sampling thermocouple

	ADCMDE  = 0x81;					 	// Enable Continuous conversion mode
	IRQEN = BIT10 + BIT11; 				// Enable ADC0 and UART interrupts
	fVoltsUni = 1.2 / 16777216;			// Volts per ADC unit in Unipolar mode
	fVoltsBi = 2.4 / 16777216;			// Volts per ADC unit in Biipolar mode
	while (1)
	{
	   	if (newADCdata == 1) 			// if new ADC data is available
	   	{
				fVThermocouple = lADC0_Thermocouple * fVoltsBi / ucThermocoupleGain;
				newADCdata = 0;								// Indicate that data has been read
				bSendResultToUART = 1;						// Now that both channels are sampled send result to UART
			}
	   	//delay (0x1FFF);
		if (bSendResultToUART == 1) // Is there an ADC0 result ready for UART transmission?
	   	{
	   		bSendResultToUART = 0;
//			sprintf ( (char*)szTemp, "Voltage : \t%+8.6ffV \r\n",fVThermocouple );// Send the ADC0 Result to the UART                          
				sprintf ( (char*)szTemp, "%+8.6fV \r\n",fVThermocouple );
				nLen = strlen((char*)szTemp);
     		if (nLen <64)	SendString();
   		}
	}
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

float CalculateThermoCoupleTemp(void)
{	
	float fresult = 0;
	float fMVthermocouple = fVThermocouple*1000;			//thermocouple voltage in mV
	if (fMVthermocouple >= 0)
	{
  		j=(fMVthermocouple-VMINP)/VSEGP;       				// determine which coefficient to use
  		if (j>NSEGP-1)     									// if input is over-range..
    		j=NSEGP-1;            							// ..then use highest coefficients
		
		// Use the closest known temperature value and then use a linear approximation beetween the
		// enclosing data points
  		fresult = C_themocoupleP[j]+(fMVthermocouple-(VMINP+VSEGP*j))*(C_themocoupleP[j+1]-C_themocoupleP[j])/VSEGP;
	}
	else if (fMVthermocouple < 0)
	{
		j=(fMVthermocouple - VMINN) / VSEGN;       			// determine which coefficient to use
		if (j>NSEGN-1)     									// if input is over-range..
    		j=NSEGN-1;            							// ..then use highest coefficients
		fresult = C_themocoupleN[j]+(fMVthermocouple-(VMINN+VSEGN*j))*(C_themocoupleN[j+1]-C_themocoupleN[j])/VSEGN;
	} 
   	return  fresult;
}
void ADC0Init()
{
	ADCMSKI = BIT0;						// Enable ADC0 result ready interrupt source
  	ADCFLT = 0xFF1F;					// Chop on, Averaging, AF=63, SF=31, 4Hz							    
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
	COMDIV0 = 0x21;				// Set baud rate to 9600.
	COMDIV1 = 0x00;

	COMCON0 = BIT0 + BIT1 + BIT2;	// 8 data bits 2 stop bits
	COMIEN0 = BIT0 + BIT1;	 	// Enable UART interrupts when Rx full and Tx buffer empty.	
}

void SendString (void)
{
   	for ( i = 0 ; i < nLen ; i++ )	// loop to send ADC0 result
	{
 		 COMTX = szTemp[i];
  		 ucTxBufferEmpty = 0;
  		 while (ucTxBufferEmpty == 0)
  		 {
  		 }
	}
}
void IRQ_Handler(void) __irq
{
	volatile unsigned long IRQSTATUS = 0;
	unsigned char ucCOMIID0 = 0;
	
	IRQSTATUS = IRQSTA;	   					// Read off IRQSTA register
	if ((IRQSTATUS & BIT11) == BIT11)		//UART interrupt source
	{
		ucCOMIID0 = COMIID0;
		if ((ucCOMIID0 & 0x2) == 0x2)	  	// Transmit buffer empty
		{
		  ucTxBufferEmpty = 1;
		}
		if ((ucCOMIID0 & 0x4) == 0x4)	  			// Receive byte
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

 		ucSaveADCResult++;		
	}

// Simple Delay routine
void delay (int length)
{
	while (length >0)
    	length--;
}

