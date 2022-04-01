#include "MY_RTOS.h"
/*==============================================================delare_glocal_variables===============================================================*/
TASK_Typedef TASKS[MAX_TASKS];
volatile uint32_t g_count = 0U;
volatile uint8_t current_task = 1U;
/*==============================================================delare_glocal_variables===============================================================*/
/*===============================================================declare_fucntions====================================================================*/
void enable_faults_exceptions(void)
{
	SCB->SHCSR |= (1U<<18);
	SCB->SHCSR |= (1U<<17);
	SCB->SHCSR |= (1U<<16);
}
void initial_stacks(void)
{
	TASKS[0].PSP_VALUE = STACK_IDE;
	TASKS[1].PSP_VALUE = STACK_TASK_1;
	TASKS[2].PSP_VALUE = STACK_TASK_2;
	TASKS[3].PSP_VALUE = STACK_TASK_3;
	
	TASKS[0].STATUS = STATE_READY;
	TASKS[1].STATUS = STATE_READY;
	TASKS[2].STATUS = STATE_READY;
	TASKS[3].STATUS = STATE_READY;
	TASKS[0].STATUS = STATE_READY;
	TASKS[1].STATUS = STATE_READY;
	TASKS[2].STATUS = STATE_READY;
	TASKS[3].STATUS = STATE_READY;
	
	TASKS[0].MY_FUCTION = (void (*)(void))(*(ARR_MY_FUNCTION + 0U));
	TASKS[1].MY_FUCTION = (void (*)(void))(*(ARR_MY_FUNCTION + 1U));
	TASKS[2].MY_FUCTION = (void (*)(void))(*(ARR_MY_FUNCTION + 2U));
	TASKS[3].MY_FUCTION = (void (*)(void))(*(ARR_MY_FUNCTION + 3U));
	
	uint32_t* p_PSP;
	uint8_t dem = 0U;
	for(dem = 0U; dem < MAX_TASKS; dem++)
	{
		p_PSP = (uint32_t*)(TASKS[dem].PSP_VALUE);
		p_PSP--;//XPSR
		*p_PSP = SET_T_BIT;
		p_PSP--;//PC
		*p_PSP = (uint32_t)(TASKS[dem].MY_FUCTION);
		p_PSP--;//LR
		*p_PSP = 0xFFFFFFFD;
		
		uint32_t i = 0U;
		for(i = 0U; i < 13U; i++)
		{
			p_PSP--;
			*p_PSP = 0U;
		}
		TASKS[dem].PSP_VALUE = (uint32_t)p_PSP;
	}
}
uint32_t get_stack_pointer_PSP(void)
{
	return TASKS[current_task].PSP_VALUE;
}
void save_stack_pointer(uint32_t value_stack)
{
	TASKS[current_task].PSP_VALUE = value_stack;
}
void switch_tasks(void)
{
	uint8_t dem = 0U;
	STATE_Typedef original_state = STATE_BLOCK;
	for(dem = 0U; dem < MAX_TASKS; dem++)
	{
		current_task++;
		current_task %= MAX_TASKS;
		original_state =  TASKS[current_task].STATUS ;
		if(TASKS[current_task].STATUS == STATE_READY && current_task != 0U)
		{
			break;
		}
	}
	if(original_state != STATE_READY)
	{
		current_task = 0U;
	}
}
void task_delay(uint32_t time)
{
	__disable_irq();
	if(current_task)
	{
		TASKS[current_task].TIME_BLOCK = g_count + time;
		TASKS[current_task].STATUS = STATE_BLOCK;
		SCB->ICSR |= (1U<<28);
	}
	__enable_irq();
}
void systick_config(void)
{
	SysTick->LOAD = 72U*1000U -1U;
	SysTick->VAL = 0U;
	SysTick->CTRL = 7U;
}
/*===============================================================declare_fucntions====================================================================*/
