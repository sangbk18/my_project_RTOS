#include "my_diver_stm32f1xx.h"

#define adress_vtor 0xE000ED08

/*=========================================FLASH_OPRATION=================================================*/
void Flash_write(volatile uint32_t adress_start,volatile uint16_t * data,volatile uint32_t size)
{
	if(FLASH->CR&(1U<<1))
	{
		FLASH->CR &= ~(1U<<1);
	}
	uint32_t dem = 0U;
	if(size % 2 != 0U)
	{
		 size += 1U; 
	}
	uint32_t length = (uint32_t)(size /2U);
	if(FLASH->CR&(1U<<7))
	{
		/*unclock sequence*/
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	FLASH->CR |= (1U<<0);
	for(dem = 0U ; dem < length ; dem++)
	{
		(SCB->CCR&(1U<<3)) ? (SCB->CCR &= ~(1U<<3)) : NULL; // clear bit unaligned trap
		*((volatile uint16_t*)adress_start + dem) = *((volatile uint16_t*)data + dem);
	}	
	while((FLASH->SR&(1U<<1)))
	{
		/*wait busy bit reset - finish opration*/
	}
	FLASH->CR &= ~(1U<<0);
}
void Flash_read(volatile uint32_t adress_start,volatile uint32_t * data,volatile uint32_t size)
{
	uint32_t dem = 0U;
	uint32_t length = (uint32_t)(size / 4U);
	while((FLASH->SR&(1U<<1)))
	{
		/*wait busy bit reset - finish opration*/
	}
	if(FLASH->CR&(1U<<7))
	{
		/*unclock sequence*/
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	for(dem = 0U ; dem < length; dem++)
	{
		*(data + dem) = *((volatile uint32_t*)adress_start + dem);
	}
}
void FLash_read_half_word(volatile uint32_t adress_start,volatile uint16_t * data,volatile uint32_t size)
{
	uint32_t dem = 0U;
	while((FLASH->SR&(1U<<1)))
	{
		/*wait busy bit reset - finish opration*/
	}
	if(FLASH->CR&(1U<<7))
	{
		/*unclock sequence*/
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	for(dem = 0U ; dem < size; dem++)
	{
		*((volatile uint32_t*)data + dem) = *((volatile uint16_t*)adress_start + dem);
	}
}
void FLash_erase(volatile uint32_t adress_start)
{
	if(FLASH->CR&(1U<<7))
	{
		/*unclock sequence*/
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	FLASH->CR |= (1U<<1);
	FLASH->AR = adress_start;
	FLASH->CR |= (1U<<6);
	while((FLASH->SR&(1U<<1)))
	{
		/*wait busy bit reset - finish opration*/
	}
	FLASH->CR &= ~(1U<<1);
	FLASH->CR &= ~(1U<<6);
}
/*==========================================FLASH_OPRATION================================================*/
/*========================================MY_CONFIG_INTERRUPT=============================================*/
/*========================================unit_test=======================================================*/
bool assert_paramater(bool condition)
{
	static bool check = true;
	if(condition == false)
	{
		check = false;
	}
	return check;
}
/*========================================unit_test=======================================================*/
void Change_function_interrupt(volatile uint32_t adress_vtor_mapped,volatile Offset_interrupt offset,volatile my_func adress_function)
{
	if(assert_paramater(IS_OFFSET_INTERRUPT(offset)))
	{*(volatile uint32_t *)adress_vtor = adress_vtor_mapped;
	*(volatile uint32_t *)((volatile uint32_t*)(*(volatile uint32_t *)adress_vtor) + offset) = (uint32_t)(void (*)(void))adress_function;
	}
	else return;
}
void Change_interrupt_function(volatile uint32_t adress_vtor_mapped,volatile Offset_interrupt *p_offset,volatile my_func *p_adress_fuction,volatile uint8_t size)
{
	uint8_t dem = 0U;
	*(volatile uint32_t*)adress_vtor = (uint32_t)((volatile uint32_t*)adress_vtor_mapped);
	for(dem = 0U; dem < size; dem++)
	{
		*(volatile uint32_t*)((volatile uint32_t*)(*(volatile uint32_t*)adress_vtor) + *(p_offset + dem)) = (uint32_t)((void (*)(void))*(p_adress_fuction + dem));
	}
}
/*========================================MY_CONFIG_INTERRUPT=============================================*/
/*========================================BOOT_LOADER=====================================================*/
void set_stack_pointer(uint32_t value_stack_pointer)
{
	__ASM(
	  "MSR MSP,R0 \n"
    "BX LR \n"	
	);
}
void bootloader_init(uint32_t _adress_flash_start)
{
	/*disable system tick*/
	SysTick->CTRL = 0U;
	SysTick->LOAD = 0U;
	/*disable all interrupts*/
	__ASM(
	  "LDR R0,=0x01 \n"
	  "MSR FAULTMASK,R0 \n"
	);
	/*stack pointer main stack pointer*/
	set_stack_pointer(*(volatile uint32_t*)_adress_flash_start);
	/*call reset fuction*/
	void (*my_reset_handler)(void) = (void (*)(void))(*(volatile uint32_t*)((volatile uint32_t*)_adress_flash_start + 1U));
	my_reset_handler();
}
/*========================================BOOT_LOADER=====================================================*/
/*===========================================WWDG_config==================================================*/
void Window_watchdog_init(void)
{
//	RCC->APB1ENR |= (1U<<11);
//	WWDG->CFR |= (3U<<7);
//	WWDG->CFR |= (1U<<9);
//	WWDG->CR |= (1U<<7);
//	WWDG->CR |= (126U<<0);
//	WWDG->SR &= ~(1U<<0);
	RCC->APB1ENR |= (1U<<11);//enable clock for watchdog 
//	WWDG->CFR |= (1U<<9);
	WWDG->CFR |= (3U<<7);
	WWDG->CR = 0xFE;
}
/*===========================================WWDG_config==================================================*/
/*=========================================Independent_config=============================================*/
void Independent_watchdog_init(void)
{
	/*enbale clock LSI*/
//	RCC->APB1ENR |= (3U<<27);
//	PWR->CR |= (1U<<8);
	RCC->CSR |= (1U<<0);
	while(!((RCC->CSR >>1)&0x01))
	{
		/*wait bit LSI on set, this indicate lsi clock is stable*/
	}
	IWDG->SR &= ~(3U<<0);
	IWDG->PR &= ~(7U<<0);
	IWDG->PR |= (4U<<0);
	IWDG->KR = 0x5555;
	IWDG->KR = 0xCCCC;//start idependent watchdog
}
/*=========================================Independent_config=============================================*/
/*==========================================EXT_CONFIGURE=================================================*/
void EXT_config(void)
{
	RCC->APB2ENR |= (1U<<0);// enable AFIO;
	AFIO->EXTICR[0] &= ~(15U<<0);
	AFIO->EXTICR[0] &= ~(15U<<4);
	AFIO->EXTICR[0] &= ~(15U<<12);
	AFIO->EXTICR[0] |= (1U<<0); // pin b0;
	AFIO->EXTICR[0] |= (1U<<4); // pin b1;
	AFIO->EXTICR[0] &= ~(15U<<8); // pin a2;
  AFIO->EXTICR[0] &= ~(15U<<12); // pin a3;
//	AFIO->EXTICR[0] |= (1U<<12); // pin b3;
	AFIO->EXTICR[1] &= ~(15U<<0);
	AFIO->EXTICR[1] |= (1U<<0); //pin b4;
	RCC->APB2ENR |= (1U<<3);//enable clock on port b
	RCC->APB2ENR |= (1U<<2);//enable clock on port a
	/*config exti on EXT0 - PIN B0*/
	GPIOB->CRL &= ~(15U<<0);
	GPIOB->CRL |= (8U<<0);
	GPIOB->BSRR |= (1U<<0); // input pull up
	EXTI->IMR |= (1U<<0); //NonMask on exti0
	EXTI->FTSR |= (1U<<0); //Falling trigger enabled
	EXTI->RTSR &= ~(1U<<0);
	/*config exti on EXT1 - PIN B1*/
	GPIOB->CRL &= ~(15U<<4);
	GPIOB->CRL |= (8U<<4);
	GPIOB->BSRR |= (1U<<1); // input pull up
	EXTI->IMR |= (1U<<1); //NonMask on exti1
	EXTI->FTSR |= (1U<<1); //Falling trigger enabled
	EXTI->RTSR &= ~(1U<<1);
	/*config exti on EXT1 - PIN A2*/
	GPIOA->CRL &= ~(15U<<8);
	GPIOA->CRL |= (8U<<8);
	GPIOA->BSRR |= (1U<<2); // input pull up
	EXTI->IMR |= (1U<<2); //NonMask on exti0
	EXTI->FTSR |= (1U<<2); //Falling trigger enabled
	EXTI->RTSR &= ~(1U<<2);
	/*config exti on EXT1 - PIN A3*/
	GPIOA->CRL &= ~(15U<<12);
	GPIOA->CRL |= (8U<<12);
	GPIOA->BSRR |= (1U<<3); // input pull up
	EXTI->IMR |= (1U<<3); //NonMask on exti0
	EXTI->FTSR |= (1U<<3); //Falling trigger enabled
	EXTI->RTSR &= ~(1U<<3);
//	/*config exti on EXT3 - PIN B3*/
//	GPIOB->CRL &= ~(15U<<12);
//	GPIOB->CRL |= (8U<<12);
//	GPIOB->BSRR |= (1U<<3); // input pull up
//	EXTI->IMR |= (1U<<3); //NonMask on exti0
//	EXTI->FTSR |= (1U<<3); //Falling trigger enabled
//	EXTI->RTSR &= ~(1U<<3);
	/*config exti on EXT4 - PIN B4*/
	GPIOB->CRL &= ~(15U<<16);
	GPIOB->CRL |= (8U<<16);
	GPIOB->BSRR |= (1U<<4); // input pull up
	EXTI->IMR |= (1U<<4); //NonMask on exti0
	EXTI->FTSR |= (1U<<4); //Falling trigger enabled
	EXTI->RTSR &= ~(1U<<4);
}
/*==========================================EXT_CONFIGURE=================================================*/
/*==========================================SYSTEM_CLOCK==================================================*/
/*default : system clock equal 72Mhz*/
void system_clock_init(void)
{
	/*setting flash latency*/
	FLASH->ACR &= ~(7U<<0);
	FLASH->ACR |= (2U<<0);
	/*using HSE clock for system clock*/
	/*HSE clock enable*/
  RCC->CR |= (1U<<16);
	/*wait HSE clock ready*/
	while(!(RCC->CR&(1U<<17)))
	{
		/*no opration - ide*/
	}
	/*HSE clock not divided*/
	RCC->CFGR &= ~(1U<<17);
	/*HSE oscillator clock selected as PLL input clock */
	RCC->CFGR |= (1U<<16);
	/*PLL multiplication factor*/
	/*caution : PLL output frequency must not exceed 72 Mhz*/
	/*choose PLL output frequency is 72Mhz*/
	RCC->CFGR &= ~(15U<<18);
	RCC->CFGR |= (7U<<18);
	/*enable PLL clock*/
	RCC->CR |= (1U<<24);
	/*wait PLL clock ready plag*/
	while(!(RCC->CR&(1U<<25)))
	{
		/*no opration - ide*/
		/*if bit 17 in RCC_CR register set 1 then PLL clock unlocked*/
	}
	/*PLL selected as system clock*/
	RCC->CFGR &= ~(3U<<0);
	RCC->CFGR |= (2U<<0);
	/*AHB prescaler*/
	/*AHB selected as PLL frequency output - 72Mhz*/
	RCC->CFGR &= ~(15U<<4);
	/*AHB1 prescaler - 36Mhz*/
	RCC->CFGR &= ~(7U<<8);
	RCC->CFGR |= (4U<<8);
	/*AHB2 prescaler - 72Mhz*/
	RCC->CFGR &= ~(7U<<11);
	/*ADC clock slected as 12Mhz - must not exceed 14Mhz*/
	RCC->CFGR &= ~(3U<<14);
	RCC->CFGR |= (2U<<14);
}
/*==========================================SYSTEM_CLOCK==================================================*/
/*===========================================I2C_CONFIG===================================================*/
void i2c_init(I2C_TypeDef *i2c)
{
	if(i2c == (I2C_TypeDef *)_I2C1_ADRESS)
	{
		RCC->APB2ENR |= (1U<<3);
		GPIOB->CRL |= (15U<<24);
		GPIOB->CRL |= (15U<<28);
		
	}
	RCC->APB1ENR |= (1U<<21);
	i2c->CR1 |= (1U<<15);
	i2c->CR1 &= ~(1U<<15);
	i2c->CR2 &= ~(63U<<0);
	i2c->CR2 |= (36U<<0);
	i2c->CCR = 180U;
	i2c->TRISE = 37U;
	i2c->CR1 |= (1U<<0);
}
void i2c_start(I2C_TypeDef *i2c)
{
	i2c->CR1 |= (1U<<8);
	while(!(i2c->SR1&(1U<<0)))
	{
		
	}
	uint16_t tmp = i2c->SR1;
	tmp = 0U;
	
}
void i2c_adress(I2C_TypeDef *i2c,uint8_t adress)
{
	i2c->DR = adress;
	while(!(i2c->SR1&(1U<<1)))
	{
		
	}
	uint16_t tmp = i2c->SR1 | i2c->SR2;
	tmp = 0U;
}
void i2c_data(I2C_TypeDef *i2c,uint8_t data)
{
	while(!(i2c->SR1&(1U<<7)))
	{
		/*data empty*/
	}
	i2c->DR = data;
	while(!(i2c->SR1&(1U<<2)))
	{
		
	}
}
void i2c_mutil_data(I2C_TypeDef *i2c,uint8_t adress,uint8_t *data,uint8_t size)
{
	uint8_t dem = 0U;
	i2c_start(i2c);
	i2c_adress(i2c,adress);
	for(dem = 0U; dem<size;dem++)
	{
			while(!(i2c->SR1&(1U<<7)))
			{
				/*data empty*/
			}
			i2c->DR = *(data + dem);
	}
	while(!(i2c->SR1&(1U<<2)))
	{
		
	}
}
void i2c_stop(I2C_TypeDef *i2c)
{
	i2c->CR1 |= (1U<<9);
}
void i2c_read(I2C_TypeDef *i2c,uint8_t adress,uint8_t *data_read,uint8_t size)
{
	uint8_t length = size;
	if(length == 1U)
	{
		i2c->DR = adress;
		while(!(i2c->SR1&(1U<<1)))
		{
			
		}
		i2c->CR1 &= ~(1U<<10);//ACK =0
		uint16_t tmp = i2c->SR1 | i2c->SR2;
		tmp = 0U;
		i2c->CR1 |= (1U<<9);//stop
		while(!(i2c->SR1&(1U<<6)))
		{
			
		}
		*(data_read + size - length) = (uint8_t)i2c->DR;
	}
	else
	{
		i2c->DR = adress;
		while(!(i2c->SR1&(1U<<1)))
		{
			
		}
		uint16_t tmp = i2c->SR1 | i2c->SR2;
		tmp = 0U;
		while(length > 2U)
		{
			while(!(i2c->SR1&(1U<<6)))
			{
				
			}
			*(data_read + size - length) = (uint8_t)i2c->DR;
			i2c->CR1 |= (1U<<10);
			length--;
		}
		while(!(i2c->SR1&(1U<<6)))
		{
			
		}
		*(data_read + size - length) = (uint8_t)i2c->DR;
		length--;
		i2c->CR1 &= ~(1U<<10);
		i2c->CR1 |= (1U<<9);
		while(!(i2c->SR1&(1U<<6)))
		{
			
		}
		*(data_read + size - length) = (uint8_t)i2c->DR;
	}
}
/*===========================================I2C_CONFIG===================================================*/
/*===========================================MLX9014_TEMPRATURE===========================================*/
void MLX9014_read(uint16_t *tem)
{
	uint8_t data_i2c[3] = {0U};
	i2c_init((I2C_TypeDef*)_I2C1_ADRESS);
	i2c_start((I2C_TypeDef*)_I2C1_ADRESS);
	i2c_adress((I2C_TypeDef*)_I2C1_ADRESS,0xB4);
	i2c_data((I2C_TypeDef*)_I2C1_ADRESS,0x07);
	i2c_start((I2C_TypeDef*)_I2C1_ADRESS);
	i2c_read((I2C_TypeDef*)_I2C1_ADRESS,0xB5,&data_i2c[0],3U);
	*tem = (uint16_t)(((data_i2c[1]<<8) | (data_i2c[0]))*0.02) - 273U;
}
/*===========================================MLX9014_TEMPRATURE===========================================*/
/*==========================================UART_typedef==================================================*/
void UART_init(USART_TypeDef *uart)
{
	RCC->APB2ENR |= (1U<<2);
	GPIOA->CRH &= ~(15U<<4);
	GPIOA->CRH &= ~(15U<<8);
	GPIOA->CRH |= (11U<<4); // TX-pina9
	GPIOA->CRH |= (8U<<8);// RX -PIN_A10
	GPIOA->ODR |= (1U<<10);
	
	/*UART configure*/
	RCC->APB2ENR |= (1U<<14);
	uart->CR1 |= (1U<<13);// enable usart
	uart->CR1 &= ~(1U<<12); // 1 bit start,8 bit data, 1 bit stop
	uart->CR2 &= ~(3U<<12);
	uart->BRR = (468U<<4) | (12U<<0);
  uart->CR1 |= (1U<<3); //enable tranmission
	uart->CR1 |= (1U<<2); // enable receive
	uart->CR1 |= (1U<<5); //enable interrupt receive+
}
void UART_data(USART_TypeDef *uart,uint8_t data)
{
	while(!(uart->SR&(1U<<7)))
	{
		/*wait txe bit set*/
	}
	uart->DR = data;
	while(!(uart->SR&(1U<<6)))
	{
		/*wait TC bit set*/
	}
	uart->SR &= ~(1U<<6);
}
void UART_string(USART_TypeDef *uart,char *s)
{
	while(*s != '\0')
	{
		while(!(uart->SR&(1U<<7)))
		{
			/*wait txe bit set*/
		}
		uart->DR = *s;
		s++;
	}
	while(!(uart->SR&(1U<<6)))
	{
		/*wait TC bit set*/
	}
	uart->SR &= ~(1U<<6);
}
/*==========================================UART_typedef==================================================*/
/*=======================================UART_TX_DMA_typedef==============================================*/
void UART_DMA_TX_init(USART_TypeDef *uart)
{
	RCC->APB2ENR |= (1U<<2);
	GPIOA->CRH &= ~(15U<<4);
	GPIOA->CRH &= ~(15U<<8);
	GPIOA->CRH |= (11U<<4); // TX-pina9
	GPIOA->CRH |= (8U<<8);// RX -PIN_A10
	GPIOA->ODR |= (1U<<10);
	
	/*UART configure*/
	RCC->APB2ENR |= (1U<<14);
	uart->CR1 |= (1U<<13);// enable usart
	uart->CR1 &= ~(1U<<12); // 1 bit start,8 bit data, 1 bit stop
	uart->CR2 &= ~(3U<<12);
	uart->BRR = (468U<<4) | (12U<<0);
  uart->CR1 |= (1U<<3); //enable tranmission
	uart->CR3 |= (1U<<7); //enable DMA TX;
	
	/*DMA config*/
	RCC->AHBENR |= (1U<<0);
	DMA1_Channel4->CCR |= (1U<<4); // read from memory
	DMA1_Channel4->CCR &= ~(1U<<5); // enable circular mode
	DMA1_Channel4->CCR &= ~(1U<<6); 
	DMA1_Channel4->CCR |= (1U<<7);
	DMA1_Channel4->CCR &= ~(3U<<8);
	DMA1_Channel4->CCR &= ~(3U<<10);
	DMA1_Channel4->CCR &= ~(3U<<12);
}
void DMA_UART_TX(volatile uint16_t *adress_memory,volatile uint16_t *adress_peripheral,volatile uint16_t size)
{
	DMA1_Channel4->CMAR = (uint32_t)adress_memory;
	DMA1_Channel4->CPAR = (uint32_t)adress_peripheral;
	DMA1_Channel4->CNDTR = (uint32_t)size;
	DMA1_Channel4->CCR |= (1U<<0);
}
/*=======================================UART_TX_DMA_typedef==============================================*/
/*=======================================UART_RX_DMA_typedef==============================================*/
void UART_DMA_RX_init(USART_TypeDef *uart)
{
	
	RCC->APB2ENR |= (1U<<2);
	GPIOA->CRH &= ~(15U<<4);
	GPIOA->CRH &= ~(15U<<8);
	GPIOA->CRH |= (11U<<4); // TX-pina9
	GPIOA->CRH |= (8U<<8);// RX -PIN_A10
	GPIOA->ODR |= (1U<<10);
	
	/*UART configure*/
	RCC->APB2ENR |= (1U<<14);
	uart->CR1 |= (1U<<13);// enable usart
	uart->CR1 &= ~(1U<<12); // 1 bit start,8 bit data, 1 bit stop
	uart->CR2 &= ~(3U<<12);
	uart->BRR = (468U<<4) | (12U<<0);
  uart->CR1 |= (1U<<2); //enable recevive
	uart->CR3 |= (1U<<6); //enable DMA RX;
	
	/*DMA config*/
	RCC->AHBENR |= (1U<<0);
	DMA1_Channel5->CCR &= ~(1U<<4); // read from peripheral
	DMA1_Channel5->CCR |= (1U<<5); // enable circular mode
	DMA1_Channel5->CCR &= ~(1U<<6); 
	DMA1_Channel5->CCR |= (1U<<7);
	DMA1_Channel5->CCR &= ~(3U<<8);
	DMA1_Channel5->CCR &= ~(3U<<10);
	DMA1_Channel5->CCR &= ~(3U<<12);
	DMA1_Channel5->CCR |= (1U<<1); // enable interrupt transfer data complete
}
void DMA_UART_RX(volatile uint16_t *adress_memory,volatile uint16_t *adress_peripheral,volatile uint16_t size)
{
	DMA1_Channel5->CMAR = (uint32_t)adress_memory;
	DMA1_Channel5->CPAR = (uint32_t)adress_peripheral;
	DMA1_Channel5->CNDTR = (uint32_t)size;
	DMA1_Channel5->CCR |= (1U<<0);
}
/*=======================================UART_RX_DMA_typedef==============================================*/
/*=======================================SPI_config=======================================================*/
void SPI_init(SPI_TypeDef *spi)
{
	if(spi == (SPI_TypeDef*)_SPI1_ADRESS)
	{
		RCC->APB2ENR |= (1U<<12);
		RCC->APB2ENR |= (1U<<2);
		GPIOA->CRL &= ~(15U<<16); //pin A4 - NSS - Not used. Can be used as a GPIO
		GPIOA->CRL &= ~(15U<<20); //pin A5 - SCK - Alternate function push-pull
    GPIOA->CRL &= ~(15U<<24); //pin A6 - MISO	- Input floating / Input pull-up 
		GPIOA->CRL &= ~(15U<<28); //pin A7 - MOSI - Alternate function push-pull
		GPIOA->CRL |= (3U<<16);
		GPIOA->ODR |= (1U<<4);
		GPIOA->CRL |= (11U<<20);
		GPIOA->CRL |= (11U<<28);
		GPIOA->CRL |= (8U<<24);
		GPIOA->ODR |= (1U<<6);
	}
  spi->CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1
  spi->CR1 |= (1<<2);  // Master Mode
  spi->CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
  spi->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
  spi->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
  spi->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
  spi->CR1 &= ~(1<<11);  // DFF=0, 8 bit data
  spi->CR2 = 0;
}
void SPI_send(SPI_TypeDef *spi,uint8_t *data,uint8_t size)
{
	uint8_t dem = 0U;
	for(dem = 0U;dem <size; dem++)
	{
		while(!(spi->SR&(1U<<1)))
		{
			/*wait TXE set*/
		}
		spi->DR = *(data + dem);
	}
	while(spi->SR&(1U<<7))
	{
		/*wait busy bit clear*/
	}
	if(spi->SR&(1U<<6))
	{
		uint8_t tmp = (uint8_t)spi->DR | (uint8_t)spi->SR;
		tmp = 0U;
	}
}
void SPI_receive(SPI_TypeDef *spi,uint8_t *data,uint8_t size)
{
	uint8_t dem = 0U;
	for(dem = 0U; dem < size; dem++)
	{
		while(spi->SR&(1U<<7))
		{
			/*wait busy bit clear*/
		}
		SPI1->DR = 0U;
		while(!(spi->SR&(1U<<0)))
		{
			/*wait RXE set*/
		}
		*(data + dem) = (uint8_t)spi->DR;
	}
}
/*=======================================SPI_config=======================================================*/
/*========================================ADC_config======================================================*/
void ADC_init(ADC_TypeDef *adc,ADC_Channel_Typedef *channel,uint8_t size)
{
	RCC->APB2ENR |= (1U<<2);//enable peripheral clock on port a
	uint8_t dem = 0U;
	for(dem = 0U; dem < size; dem++)
	{
		GPIOA->CRL &= ~(15U<<((*(channel + dem))*4));
	}
	if(adc == (ADC_TypeDef*)_ADC1_adress)
	{
		RCC->APB2ENR |= (1U<<9);
	}
	else if(adc == (ADC_TypeDef*)_ADC2_adress)
	{
		RCC->APB2ENR |= (1U<<10);
	}
	/*ADC_config*/
	/*eanble regular channel on all*/
	adc->CR1 &= ~(1U<<23);// diasble analogwatchdog
	adc->CR1 &= ~(15U<<16); // selection independent mode
	adc->CR1 &= ~(1U<<11); // Disable discontinuos moide on regular channels
  adc->CR1 |= (1U<<9);// anable analog watchdog on single channel
  adc->CR1 |= (1U<<8); // enable scan mode
  adc->CR1 |= (1U<<6);	// enable analog watchdog interrupt
	adc->CR2 &= ~(1U<<22); // reset bit start conversion of reguler channels
	adc->CR2 &= ~(1U<<20); //disable external trigger
	adc->CR2 |= (7U<<17); //select SWSTART
	adc->CR2 &= ~(1U<<11); // right alignment
	adc->CR2 |= (1U<<8); // enable DMA
	adc->CR2 |= (1U<<1); //continous conversion
	/*set sample time for channels*/
	dem = 0U;
	for(dem = 0U;dem <size ; dem++)
	{
		adc->SMPR2 &= ~(7U<<((*(channel + dem))*3U));
		adc->SMPR2 |= (5U<<((*(channel + dem))*3U));
	}
	/*set length sequence*/
	adc->SQR1 &= ~(15U<<20);
	adc->SQR1 |= ((size-1U)<<20);
	dem = 0U;
	for(dem = 0U;dem <size ; dem++)
	{
		adc->SQR3 &= ~(31U<<((*(channel + dem))*5U));
		adc->SQR3 |= ((uint32_t)(*(channel + dem))<<(dem*5U));
	}
	
	/*start conversion adc*/
	adc->CR2 |= (1U<<0); // adc converter on
	adc->CR2 |= (1U<<0); 
	adc->CR2 |= (1U<<22); 
}
void ADC_DMA_init(void)
{
	RCC->AHBENR |= (1U<<0);
	DMA1_Channel1->CCR &= ~(1U<<4); // read from peripheral
	DMA1_Channel1->CCR |= (1U<<5); // enable circular mode
	DMA1_Channel1->CCR &= ~(1U<<6); 
	DMA1_Channel1->CCR |= (1U<<7);
	DMA1_Channel1->CCR &= ~(3U<<8);
	DMA1_Channel1->CCR |= (1U<<8);
	DMA1_Channel1->CCR &= ~(3U<<10);
	DMA1_Channel1->CCR |= (1U<<10);
	DMA1_Channel1->CCR &= ~(3U<<12);
}
void ADC_DMA_start(volatile uint16_t *memory_adress,volatile uint16_t *peripheral_adress,volatile uint16_t size)
{	
	DMA1_Channel1->CMAR = (uint32_t)memory_adress;
	DMA1_Channel1->CPAR = (uint32_t)peripheral_adress;
	DMA1_Channel1->CNDTR = (uint32_t)size;
	DMA1_Channel1->CCR |= (1U<<0);
	
}
/*========================================ADC_config======================================================*/
/*========================================PWM_config======================================================*/
void PWM_init(TIM_TypeDef *tim,uint16_t * duty)
{
	if(tim == (TIM_TypeDef*)_TIM_1)
	{
		RCC->APB2ENR |= (1U<<11);
	}
	else if(tim == (TIM_TypeDef*)_TIM_2)
	{
		RCC->APB1ENR |= (1<<0);
	}
	tim->CR1 |= (1U<<7);
	tim->CR1 &= ~(3U<<0);
	tim->ARR = 1000U - 1U;
	if(tim == (TIM_TypeDef*)_TIM_1)
	{
		tim->PSC = 72U - 1U;
	}
	else if(tim == (TIM_TypeDef*)_TIM_2)
	{
		tim->PSC = 36U - 1U;
	}
	tim->CNT = 0U;
	tim->RCR = 0U;
	
	/*default using timer2 modulation pulse width - channel PA3*/
	RCC->APB2ENR |= (1U<<2);
	GPIOA->CRL &= ~(15U<<12);
	GPIOA->CRL |= (11U<<12);
	tim->CCMR2 &= ~(7U<<12);
	tim->CCMR2 |= (6U<<12);
	tim->CCMR2 &= ~(3U<<8);
	tim->CCER &= ~(1U<<13);
	tim->CCER |= (1U<<12);
	tim->CCR4 = *duty;
	
	tim->EGR |= (1U<<0);
	tim->CR1 |= (1U<<0);
}
/*========================================PWM_config======================================================*/
