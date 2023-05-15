



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
	//GPIOA->MODER |= (3<<8);  // analog mode for PA 2
	
	
	
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
	//ADC1->SQR3 |= (4<<5);  // SEQ2 for CHannel 2
	//ADC1->SQR3 |= (18<<10);  // SEQ3 for CHannel 18
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





uint16_t RxData[2048];
//float Temperature;






//uint16_t RxxData[2048];
// Code from Tusher Sir
/*
void DMAConfig(uint16_t * sindat){
	//Start DMA2 Streem5
	RCC->AHB1ENR |= (1 << 22);
	//DMA2_Stream5->CR = 0x00000000;
	DMA2_Stream5->CR &= ~(1 << 0);
	
	//DMA2_Stream1->PAR = (unsigned long)&(TIM1->DMAR);
	DMA2_Stream5->PAR = (uint32_t)&(TIM1->DMAR);
	DMA2_Stream5->M0AR =(uint32_t)&sindat;
	DMA2_Stream5->NDTR = 240;	//number of array element
	DMA2_Stream5->CR |= (1 << 13);	//Memory data length 16
	DMA2_Stream5->CR |= (1<< 11);	//pointer data length 16
	DMA2_Stream5->CR |= ( 1 << 6);	//data direction----->>>memory to peripheral
	DMA2_Stream5->CR |= (1 << 8);	//circular mode enable
	DMA2_Stream5->CR |= (1 << 10);	//Memory address increment
	DMA2_Stream5->CR |= (6 << 25);	//channeL6 selection
	DMA2_Stream5->CR |= (1 << 23);	//Memory burst increment 4
	DMA2_Stream5->CR |= (1 << 0);		//enable stream1
}	
*/

//DAC Code
  
	void DAC_Init (void)
{
	//// Enable peripherals: GPIOA, DMA, DAC, TIM6.
	
  RCC->AHB1ENR  |= ( RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_DMA1EN );
  RCC->APB1ENR  |= ( RCC_APB1ENR_DACEN |
                     RCC_APB1ENR_TIM6EN );
  // Pin A4 output type: Analog.
  GPIOA->MODER    &= ~( 0x3 << ( 4 * 2 ) );
  GPIOA->MODER    |=  ( 0x3 << ( 4 * 2 ) );
	
	//
	  
	// DMA configuration (channel 7 / stream 5).
  // SxCR register:
  // - Memory-to-peripheral
  // - Circular mode enabled.
  // - Increment memory ptr, don't increment periph ptr.
  // - 16-bit data size for both source and destination.
  // - High priority (2/3).
  DMA1_Stream5->CR &= ~( DMA_SxCR_CHSEL |
                         DMA_SxCR_PL |
                         DMA_SxCR_MSIZE |
                         DMA_SxCR_PSIZE |
                         DMA_SxCR_PINC |
                         DMA_SxCR_EN );
  DMA1_Stream5->CR |=  ( ( 0x2 << DMA_SxCR_PL_Pos ) |
                         ( 0x1 << DMA_SxCR_MSIZE_Pos ) |
                         ( 0x1 << DMA_SxCR_PSIZE_Pos ) |
                         ( 0x7 << DMA_SxCR_CHSEL_Pos ) |
                         DMA_SxCR_MINC |
                         DMA_SxCR_CIRC |
                         ( 0x1 << DMA_SxCR_DIR_Pos ) );
												 
											 }

 void DMA_Init_DAC (uint16_t srcAdd, uint16_t size)
 {	 
  // Set DMA source and destination addresses.
  // Source: Address of the sine wave buffer in memory.
  DMA1_Stream5->M0AR  = srcAdd;
  // Dest : Buffer
	uint32_t destAdd= ( uint16_t )&( DAC1->DHR12R1 );
  DMA1_Stream5->PAR   = destAdd;
  // Set DMA data transfer length (# of sine wave samples).
  DMA1_Stream5->NDTR  = size;
  // Enable DMA1 Stream 5.
  DMA1_Stream5->CR   |= ( DMA_SxCR_EN );
  // TIM6 configuration.
  // Set prescaler and autoreload for a 440Hz sine wave.
  TIM6->PSC  =  ( 0x0000 );
  TIM6->ARR  =  ( SystemCoreClock / ( 440 * size ) );
  // Enable trigger output on timer update events.
  TIM6->CR2 &= ~( TIM_CR2_MMS );
  TIM6->CR2 |=  ( 0x2 << TIM_CR2_MMS_Pos );
  // Start the timer.
  TIM6->CR1 |=  ( TIM_CR1_CEN );
  // DAC configuration.
  // Set trigger sources to TIM6 TRGO.
  DAC1->CR  &= ~( DAC_CR_TSEL1 );
  // Enable DAC DMA requests.
  DAC1->CR  |=  ( DAC_CR_DMAEN1 );
  // Enable DAC Channels.
  DAC1->CR  |=  ( DAC_CR_EN1 );
  // Delay briefly to allow sampling to stabilize
  Delay_us (uint16_t us)( 1000 );  
  // Enable DAC channel trigger.
  DAC1->CR  |=  ( DAC_CR_TEN1 );
}
 





int main ()
{
	SysClockConfig ();
	TIM6Config ();
	
	ADC_Init ();
	ADC_Enable ();
	DMA_Init ();
	
	//float val = 1.2;
	uint32_t var, i;\
	
	// DAC converstion
	
	for(i=0; i<2048; i++)
	{
	   if (RxData[i]>3)
				RxData = 3;
     RxData[i] = RxData[i]*(4096)/3.3;	// 4096 because of 12 bit	 
	}
	
	
	
	
	DAC_Init();
	
	
	DMA_Config ((uint32_t ) &ADC1->DR, (uint32_t) RxData, 2048);
	  
	DMA_Init_DAC((uint32_t) RxData, 2048);
	
	ADC_Start ();
	
	float val = 1.2;
	uint32_t var;
	
	//while (1)
	//{
		
		
		
		

		
	  
	//}
	
}
