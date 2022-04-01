#ifndef _MY_SENSORS_LIB
#define _MY_SENSORS_LIB

#include "my_diver_stm32f1xx.h"

/*===================================================DHT_11========================================================*/
union _byte
{  volatile uint8_t data;
	 struct
	 {
		 volatile uint8_t bit0:1;
		 volatile uint8_t bit1:1;
		 volatile uint8_t bit2:1;
		 volatile uint8_t bit3:1;
		 volatile uint8_t bit4:1;
		 volatile uint8_t bit5:1;
		 volatile uint8_t bit6:1;
		 volatile uint8_t bit7:1;
	 }_data;
};
typedef union _byte BYTE_Typedef;
extern BYTE_Typedef _byte;
typedef enum
{
	GPIO_0 = 0U,
	GPIO_1,
	GPIO_2,
	GPIO_3,
	GPIO_4 =4U,
	GPIO_5,
	GPIO_6,
	GPIO_7,
	GPIO_8,
	GPIO_9,
}DHT11_PIN_Typedef;
typedef enum
{
	TIM1_ADRESS = 0x40012C00,
	TIM2_ADRESS = 0x40000000,
}TIM_ADRESS_Typedef;
typedef enum
{
	GPIOA_ADRESS = 0x40010800,
	GPIOB_ADRESS = 0x40010C00,
  GPIOC_ADRESS = 0x40011000,
}GPIO_ADRESS_Typedef;
struct _my_data_dht11
{
	volatile uint8_t do_am_in;
	volatile uint8_t do_am_fl;
	volatile uint8_t nhiet_do_in;
	volatile uint8_t nhiet_do_fl;
	volatile uint8_t check_sum;
};
typedef struct _my_data_dht11 DATA_DHT11_Typedef;
extern DATA_DHT11_Typedef DATA_DHT11;
#define IS_GPIO_ADRESS(gpio)  (((gpio) == (GPIO_TypeDef*)GPIOA_ADRESS) ||\
                               ((gpio) == (GPIO_TypeDef*)GPIOB_ADRESS) ||\
                               ((gpio) == (GPIO_TypeDef*)GPIOC_ADRESS))
#define IS_TIM_ADRESS(adress) (((adress) == (TIM_TypeDef*)TIM1_ADRESS) ||\
                               ((adress) == (TIM_TypeDef*)TIM2_ADRESS))
#define IS_DHT11_PIN(pin)  (((pin) == GPIO_0) ||\
														((pin) == GPIO_1) ||\
														((pin) == GPIO_2) ||\
														((pin) == GPIO_3) ||\
														((pin) == GPIO_4) ||\
														((pin) == GPIO_5) ||\
														((pin) == GPIO_6) ||\
														((pin) == GPIO_7))
void DHT11_init(TIM_TypeDef *tim,GPIO_TypeDef *GPIO,DHT11_PIN_Typedef PIN);
uint8_t read_bit(TIM_TypeDef *tim,GPIO_TypeDef *GPIO,DHT11_PIN_Typedef PIN);
void read_5_byte(TIM_TypeDef *tim,GPIO_TypeDef *GPIO,DHT11_PIN_Typedef PIN,DATA_DHT11_Typedef *P_data);
/*===================================================DHT_11========================================================*/
/*===================================================LCD_CONFIG====================================================*/
void lcd_init(I2C_TypeDef *i2c);
void lcd_data(I2C_TypeDef *i2c,uint8_t data);
void lcd_lenh(I2C_TypeDef *i2c,uint8_t lenh);
void lcd_string(I2C_TypeDef *i2c,const char* s);
void lcd_gotoxy(I2C_TypeDef *i2c,uint8_t hang,uint8_t cot);

char* covert_string(uint16_t data,char *s);
/*===================================================LCD_CONFIG====================================================*/
/*===================================================STEP_MOTOR====================================================*/
typedef enum
{
	_1_div_8_circle = 256, //45 degree
	_1_div_4_circle = 512, //90 degree
	_1_div_2_circle = 1024,//180 degree
	_1_circle = 2048, // 360 degree
	_2_circle = 4096, //720 degree
}Revolution_Typedef;
#define IS_REVOLUTION_TYPEDEF(value) (((value) == _1_div_8_circle) ||\
                                      ((value) == _1_div_4_circle) ||\
																			((value) == _1_div_2_circle) ||\
																			((value) == _1_circle)       ||\
																			((value) == _2_circle))
typedef enum
{
	clockwise = 0U,
	unclockwise,
}Direction_Typedef;
#define IS_DIRECTION_TYPEDEF(value) (((value) == clockwise) ||\
                                     ((value) == unclockwise))
void STEP_MOTOR_INIT(void);
void MOTOR_DIRECTION(Direction_Typedef direct);
void START_ROTATION(Revolution_Typedef Revolution,Direction_Typedef direct);
/*===================================================STEP_MOTOR====================================================*/
/*===================================================RTC_1307======================================================*/
union _my_data
{
   uint8_t data;
   struct 
   {
      uint8_t bit0:1;
      uint8_t bit1:1;
      uint8_t bit2:1;
      uint8_t bit3:1;
      uint8_t bit4:1;
      uint8_t bit5:1;
      uint8_t bit6:1;
      uint8_t bit7:1;
   }_data;
};
typedef union _my_data DATA_RTC_Typedef;
extern DATA_RTC_Typedef data_RTC_initial;
uint8_t bin_dec(uint8_t data);
uint8_t dec_bin(uint8_t data);
struct _RTC_CLOCK
{
	volatile uint8_t second;
	volatile uint8_t minutes;
	volatile uint8_t hours;
	volatile uint8_t day;
	volatile uint8_t date;
	volatile uint8_t month;
	volatile uint8_t year;
};
typedef struct _RTC_CLOCK MY_RTC_Typedef;
extern MY_RTC_Typedef RTC_current;
void RTC_set(I2C_TypeDef *I2C,MY_RTC_Typedef *p_RTC);
void RTC_get(I2C_TypeDef *I2C,MY_RTC_Typedef *p_RTC);
/*===================================================RTC_1307======================================================*/
#endif
