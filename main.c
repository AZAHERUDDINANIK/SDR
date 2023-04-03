



#include "RccConfig_F446.h"
#include "Delay_F446.h"


void ADC_Init (void)
{
	
	
// Enable ADC and GPIO clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock
	RCC->AHB1ENR |= (1<<0);  // enable GPIOA clock
	
// Set the prescalar in the Common Control Register (CCR)	
	ADC->CCR |= 1<<16;  		 // PCLK2 divide by 4
	
// Set the Scan Mode and Resolution in the Control Register 1 (CR1)	
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	
// Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 = (1<<1);     // enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOC after each conversion
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
	
// Set the Sampling Time for the channels	
	ADC1->SMPR2 &= ~((7<<3) | (7<<12));  // Sampling time of 3 cycles for channel 1 and channel 4

// Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (2<<20);   // SQR1_L =2 for 3 conversions
	
// Set the Respective GPIO PINs in the Analog Mode	
	GPIOA->MODER |= (3<<2);  // analog mode for PA 1
	GPIOA->MODER |= (3<<8);  // analog mode for PA 4
	
	
	
	// Sampling Freq for Temp Sensor 
	ADC1->SMPR1 |= (7<<24);  // Sampling time of 21 us
	
	// Set the TSVREFE Bit to wake the sensor
	ADC->CCR |= (1<<23);
	
	// Enable DMA for ADC
	ADC1->CR2 |= (1<<8);
	
	// Enable Continuous Request
	ADC1->CR2 |= (1<<9);
	
	// Channel Sequence
	ADC1->SQR3 |= (1<<0);  // SEQ1 for Channel 1
	ADC1->SQR3 |= (4<<5);  // SEQ2 for CHannel 4
	ADC1->SQR3 |= (18<<10);  // SEQ3 for CHannel 18
}

void ADC_Enable (void)
{
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000; // Wait for ADC to stabilize (approx 10us)
	while (delay--);
}

void ADC_Start (void)
{
	ADC1->SR = 0;        // Clear the status register
	
	ADC1->CR2 |= (1<<30);  // Start the conversion by Setting the SWSTART bit in CR2
}


void DMA_Init (void)
{
	// Enable the DMA2 Clock
	RCC->AHB1ENR |= (1<<22);  // DMA2EN = 1
	
	// Channel 0 and Stream 0 is selected for ADC
	
	// Select the Data Direction
	DMA2_Stream0->CR &= ~(3<<6);  // Peripheral to memory
	
	// Select Circular mode
	DMA2_Stream0->CR |= (1<<8);  // CIRC = 1
	
	// Enable Memory Address Increment
	DMA2_Stream0->CR |= (1<<10);  // MINC = 1;

	// Set the size for data 
	DMA2_Stream0->CR |= (1<<11)|(1<<13);  // PSIZE = 01, MSIZE = 01, 16 bit data
	
	// Select channel for the stream
	DMA2_Stream0->CR &= ~(7<<25);  // Channel 0 selected
}


void DMA_Config (uint32_t srcAdd, uint32_t destAdd, uint16_t size)
{
	
	
	DMA2_Stream0->NDTR = size;   // Set the size of the transfer
	
	DMA2_Stream0->PAR = srcAdd;  // Source address is peripheral address
	
	DMA2_Stream0->M0AR = destAdd;  // Destination Address is memory address
	
	// Enable the DMA Stream
	DMA2_Stream0->CR |= (1<<0);  // EN =1
}





uint16_t RxData[3];
float Temperature;


int main ()
{
	SysClockConfig ();
	TIM6Config ();
	
	ADC_Init ();
	ADC_Enable ();
	DMA_Init ();
	
	DMA_Config ((uint32_t ) &ADC1->DR, (uint32_t) RxData, 3);
	
	ADC_Start ();
	
	while (1)
	{
		
		Temperature = (((float)(3.3*RxData[2]/(float)4095) - 0.76) / 0.0025) + 25;
		

		
	  Delay_ms (1000);
	}
	
}
