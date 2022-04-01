#ifndef _MY_RTOS_SANG_H_
#define _MY_RTOS_SANG_H_
#include "stm32f10x.h"
#include "stddef.h"
/*===============================================================define_size_tasks====================================================================*/
#define SET_T_BIT 0x01000000U
#define MAX_TASKS 4U
#define ADRESS_STACK_START 0x20000000U
#define SIZE_STACK  1024U
#define SIZE_STACKS (20U*(SIZE_STACK))
#define ADRESS_STACK_END ((ADRESS_STACK_START) + (SIZE_STACKS))

#define STACK_IDE       ADRESS_STACK_END
#define STACK_TASK_1   (ADRESS_STACK_END - (1U*SIZE_STACK))
#define STACK_TASK_2   (ADRESS_STACK_END - (2U*SIZE_STACK))
#define STACK_TASK_3   (ADRESS_STACK_END - (3U*SIZE_STACK))
#define MSP_STACK      (ADRESS_STACK_END - (4U*SIZE_STACK))
/*===============================================================define_size_tasks====================================================================*/
/*===============================================================declare_fucntions====================================================================*/
typedef enum
{
	STATE_READY = 0xFFU,
	STATE_BLOCK = 0x00U,
}STATE_Typedef;
typedef void (*my_func_tasks)(void);
extern my_func_tasks ARR_MY_FUNCTION[MAX_TASKS];
#pragma pack(1)
struct my_task
{
	volatile uint32_t PSP_VALUE;
	volatile uint32_t TIME_BLOCK;
	volatile STATE_Typedef STATUS;
	volatile my_func_tasks MY_FUCTION;
};
typedef volatile struct my_task TASK_Typedef;
#pragma pack()
/*==============================================================delare_glocal_variables===============================================================*/
extern TASK_Typedef TASKS[MAX_TASKS];
extern volatile uint32_t g_count;
extern volatile uint8_t current_task;
/*==============================================================delare_glocal_variables===============================================================*/
/*===============================================================declare_fucntions====================================================================*/
void enable_faults_exceptions(void);
void save_stack_pointer(uint32_t value_stack);
void switch_tasks(void);
void task_delay(uint32_t time);
uint32_t get_stack_pointer_PSP(void);
void initial_stack_handler_mode(uint32_t my_value_stack);
void thread_mode_using_psp(void);
void initial_stacks(void);
void systick_config(void);
/*===============================================================declare_fucntions====================================================================*/

#endif
