/*=======================STM32F103_LIB======================*/
/*===== creator : THAI VAN SANG ============================*/
/*===== DATE    : 27/2/2022 ================================*/

#ifndef _MY_LIB_STM32F103_
#define _MY_LIB_STM32F103_

#include "stm32f10x.h"
#include "stdint.h"
#include "stddef.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"

#define adress_VTOR_remap  0x20000000 
#define adress_flash_start 0x08007C00 
#define adress_flash       0x08000000
/*=========================================FLASH_OPRATION=================================================*/
void Flash_write(volatile uint32_t adress_start,volatile uint16_t * data,volatile uint32_t size);
void Flash_read(volatile uint32_t adress_start,volatile uint32_t * data,volatile uint32_t size);
void FLash_erase(volatile uint32_t adress_start);
void FLash_read_half_word(volatile uint32_t adress_start,volatile uint16_t * data,volatile uint32_t size);
/*==========================================FLASH_OPRATION================================================*/
/*========================================MY_CONFIG_INTERRUPT=============================================*/
typedef void (*my_func)(void);
typedef enum
{
	_WWDG_IRQ = 0U,
	_PVD_IRQ,
	_TAMPER_IRQ,
	_RTC_IRQ,
	_FLASH,
	_RCC,
	_EXTI0_IQR = 22U,
	_EXTI1_IQR = 23U,
	_EXTI2_IQR = 24U,
	_EXTI3_IQR = 25U,
	_EXTI4_IQR = 26U,
  _DMA1_CH1_IQR,
	_DMA1_CH2_IQR,
	_DMA1_CH3_IQR,
	_DMA1_CH4_IQR,
	_DMA1_CH5_IQR,
	_DMA1_CH6_IQR,
	_DMA1_CH7_IQR,
	_ADC_1_2_IQR,
}Offset_interrupt;
#define IS_OFFSET_INTERRUPT(offset) (((offset) == _WWDG_IRQ) ||\
                                      ((offset) == _PVD_IRQ) ||\
																			((offset) == _TAMPER_IRQ) ||\
																			((offset) == _RTC_IRQ) ||\
																			((offset) == _FLASH) ||\
																			((offset) == _RCC) ||\
																			((offset) == _EXTI0_IQR) ||\
																			((offset) == _EXTI1_IQR) ||\
																			((offset) == _EXTI2_IQR) ||\
																			((offset) == _EXTI3_IQR) ||\
																			((offset) == _EXTI4_IQR) ||\
																			((offset) == _DMA1_CH1_IQR) ||\
																			((offset) == _DMA1_CH2_IQR) ||\
																			((offset) == _DMA1_CH3_IQR) ||\
																			((offset) == _DMA1_CH4_IQR) ||\
																			((offset) == _DMA1_CH5_IQR) ||\
																			((offset) == _DMA1_CH6_IQR) ||\
																			((offset) == _DMA1_CH7_IQR) ||\
                                      ((offset) == _ADC_1_2_IQR))
bool assert_paramater(bool condition);//check offset avaiable
/*external interrupt*/
__attribute__((weak)) void ngat_ext_0(void);
__attribute__((weak)) void ngat_ext_1(void);
__attribute__((weak)) void ngat_ext_2(void);
__attribute__((weak)) void ngat_ext_3(void);
__attribute__((weak)) void ngat_ext_4(void);
/*external interrupt*/
void Change_function_interrupt(volatile uint32_t adress_vtor_mapped,volatile Offset_interrupt offset,volatile my_func adress_function);
void Change_interrupt_function(volatile uint32_t adress_vtor_mapped,volatile Offset_interrupt *p_offset,volatile my_func *p_adress_fuction,volatile uint8_t size);
/*========================================MY_CONFIG_INTERRUPT=============================================*/
/*========================================BOOT_LOADER=====================================================*/
void set_stack_pointer(uint32_t value_stack_pointer);
void bootloader_init(uint32_t _adress_flash_start);
/*========================================BOOT_LOADER=====================================================*/
/*==========================================SYSTEM_CLOCK==================================================*/
/*default : system clock equal 72Mhz*/
void system_clock_init(void);
/*==========================================SYSTEM_CLOCK==================================================*/
/*==========================================EXT_CONFIGURE=================================================*/
void EXT_config(void);
/*==========================================EXT_CONFIGURE=================================================*/
/*===========================================WWDG_config==================================================*/
void Window_watchdog_init(void);
/*===========================================WWDG_config==================================================*/
/*=========================================Independent_config=============================================*/
void Independent_watchdog_init(void);
/*=========================================Independent_config=============================================*/
/*===========================================I2C_CONFIG===================================================*/
typedef enum
{
	_I2C1_ADRESS = 0x40005400UL,
	_I2C2_ADRESS = 0x40005800UL,
}I2C_active;
void i2c_init(I2C_TypeDef *i2c);
void i2c_start(I2C_TypeDef *i2c);
void i2c_adress(I2C_TypeDef *i2c,uint8_t adress);
void i2c_data(I2C_TypeDef *i2c,uint8_t data);
void i2c_mutil_data(I2C_TypeDef *i2c,uint8_t adress,uint8_t *data,uint8_t size);
void i2c_stop(I2C_TypeDef *i2c);
void i2c_read(I2C_TypeDef *i2c,uint8_t adress,uint8_t *data_read,uint8_t size);
/*===========================================I2C_CONFIG===================================================*/
/*===========================================MLX9014_TEMPRATURE===========================================*/
void MLX9014_read(uint16_t *tem);
/*===========================================MLX9014_TEMPRATURE===========================================*/
/*==========================================UART_typedef==================================================*/
typedef enum
{
	_USART1 = 0x40013800,
}UART_Adress_Typedef;
void UART_init(USART_TypeDef *uart);
void UART_data(USART_TypeDef *uart,uint8_t data);
void UART_string(USART_TypeDef *uart,char *s);
void UART_received(USART_TypeDef *uart,uint8_t *data);
/*==========================================UART_typedef==================================================*/
/*=======================================UART_TX_DMA_typedef==============================================*/
void UART_DMA_TX_init(USART_TypeDef *uart);
void DMA_UART_TX(volatile uint16_t *adress_memory,volatile uint16_t *adress_peripheral,volatile uint16_t size);
/*=======================================UART_TX_DMA_typedef==============================================*/
/*=======================================UART_RX_DMA_typedef==============================================*/
void UART_DMA_RX_init(USART_TypeDef *uart);
void DMA_UART_RX(volatile uint16_t *adress_memory,volatile uint16_t *adress_peripheral,volatile uint16_t size);
/*=======================================UART_RX_DMA_typedef==============================================*/
/*=======================================SPI_config=======================================================*/
typedef enum
{
	_SPI1_ADRESS = 0x40013000,
	_SPI2_ADRESS = 0x01,
}SPI_Adress_Typedef;
void SPI_init(SPI_TypeDef *spi);
void SPI_send(SPI_TypeDef *spi,uint8_t *data,uint8_t size);
void SPI_receive(SPI_TypeDef *spi,uint8_t *data,uint8_t size);
/*=======================================SPI_config=======================================================*/
/*========================================ADC_config======================================================*/
typedef enum
{
	_ADC1_adress = 0x40012400,
	_ADC2_adress = 0x40012800,
}ADC_Adress_Typedef;
typedef enum
{
	Channel_0 = 0U,
	Channel_1,
	Channel_2,
	Channel_3,
	Channel_4,
	Channel_5,
	Channel_6,
	Channel_7,
	Channel_8,
	Channel_9,
	Channel_10
}ADC_Channel_Typedef;
void ADC_init(ADC_TypeDef *adc,ADC_Channel_Typedef *channel,uint8_t size);
void ADC_DMA_init(void);
void ADC_DMA_start(volatile uint16_t *memory_adress,volatile uint16_t *peripheral_adress,volatile uint16_t size);
/*========================================ADC_config======================================================*/
/*========================================PWM_config======================================================*/
typedef enum
{
	_TIM_1 = 0x40012C00,
	_TIM_2 = 0x40000000,
	_TIM_3 = 0x00,
}_TIM_Adress_Typedef;
void PWM_init(TIM_TypeDef *tim,uint16_t * duty);// default channel 4 - timer2
/*========================================PWM_config======================================================*/
#endif
