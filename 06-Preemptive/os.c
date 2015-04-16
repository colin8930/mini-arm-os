#include <stddef.h>
#include <stdint.h>
#include "reg.h"
#include "asm.h"
#include "host.h"

/* Size of our user task stacks in words */
#define STACK_SIZE	256

/* Number of user task */
#define TASK_LIMIT	5

/* USART TXE Flag
 * This flag is cleared when data is written to USARTx_DR and
 * set when that data is transferred to the TDR
 */
#define USART_FLAG_TXE	((uint16_t) 0x0080)

void usart_init(void)
{
	*(RCC_APB2ENR) |= (uint32_t)(0x00000001 | 0x00000004);
	*(RCC_APB1ENR) |= (uint32_t)(0x00020000);

	/* USART2 Configuration, Rx->PA3, Tx->PA2 */
	*(GPIOA_CRL) = 0x00004B00;
	*(GPIOA_CRH) = 0x44444444;
	*(GPIOA_ODR) = 0x00000000;
	*(GPIOA_BSRR) = 0x00000000;
	*(GPIOA_BRR) = 0x00000000;

	*(USART2_CR1) = 0x0000000C;
	*(USART2_CR2) = 0x00000000;
	*(USART2_CR3) = 0x00000000;
	*(USART2_CR1) |= 0x2000;
}

void print_str(const char *str)
{
	while (*str) {
		while (!(*(USART2_SR) & USART_FLAG_TXE));
		*(USART2_DR) = (*str & 0xFF);
		str++;
	}
}



/* Exception return behavior */
#define HANDLER_MSP	0xFFFFFFF1
#define THREAD_MSP	0xFFFFFFF9
#define THREAD_PSP	0xFFFFFFFD

/* Initilize user task stack and execute it one time */
/* XXX: Implementation of task creation is a little bit tricky. In fact,
 * after the second time we called `activate()` which is returning from
 * exception. But the first time we called `activate()` which is not returning
 * from exception. Thus, we have to set different `lr` value.
 * First time, we should set function address to `lr` directly. And after the
 * second time, we should set `THREAD_PSP` to `lr` so that exception return
 * works correctly.
 * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Babefdjc.html
 */

#define TASK_READY      0
#define TASK_WAIT_READ  1
#define TASK_WAIT_WRITE 2
#define TASK_WAIT_INTR  3
#define TASK_DELAY      4
#define TASK_EXIT       5

int task_count = 0;
int current_task = 0;
unsigned int user_stacks[TASK_LIMIT][STACK_SIZE];

struct task_TCB {
	unsigned int priority;	/*< The priority of the task where 0 is the lowest priority. */
	unsigned int *sp;	/*< Task stack pointer */
	unsigned int status;	/*< The status of task. */
	unsigned int delayTime;	/*< The tick to resume task */
};

struct task_TCB TCBs[TASK_LIMIT];

void init_TCB(struct task_TCB *TCBs)
{
	int i;
	for (i = 0; i < TASK_LIMIT; i++) {
		TCBs[i].delayTime = 0;
		TCBs[i].priority  = 0;
	}
}

void *create_task(void (*start)(void), int priority)
{
	static int first = 1;
	unsigned int *stack = (unsigned int *) user_stacks[task_count];

	stack += STACK_SIZE - 32; /* End of stack, minus what we are about to push */
	if (first) {
		stack[8] = (unsigned int) start;
		first = 0;
	} else {
		stack[8] = (unsigned int) THREAD_PSP;
		stack[15] = (unsigned int) start;
		stack[16] = (unsigned int) 0x01000000; /* PSR Thumb bit */
	}
	stack = activate(stack);
	TCBs[task_count].sp = stack;
	TCBs[task_count].priority = priority;
	TCBs[task_count].status = TASK_READY;

	task_count++;
	return 0;
}

void delay(int count)
{
	count *= 50000;
	while (count--);
}

void idle(void)
{
	print_str("idle: Created!\n\r");
	print_str("idle: Now, return to kernel mode\n\r");
	syscall();
	while (1);
}

void task1_func(void)
{
	print_str("task1: Created!\n\r");
	print_str("task1: Now, return to kernel mode\n\r");
	syscall();
	while (1) {
		print_str("task1: Running...\n\r");
		delay(1000);
	}
}

void task2_func(void)
{
	print_str("task2: Created!\n\r");
	print_str("task2: Now, return to kernel mode\n\r");
	syscall();
	while (1) {
		print_str("task2: Running...\n\r");
		delay(1000);
	}
}

void scheduler()
{
	while (1) {
		print_str("OS: Activate next task\n\r");
		int highest;
		int i;
		highest = 0;

		for (i = 0; i < task_count; i++) {
			if (TCBs[i].status == TASK_READY) {
				if (TCBs[i].priority > (unsigned int)highest) {
					highest = TCBs[i].priority;
					current_task = i;
				}
			}
		}
		TCBs[current_task].sp = activate(TCBs[current_task].sp);
		print_str("OS: Back to OS\n\r");
	}
}

int main(void)
{

	usart_init();

	init_TCB(TCBs);

	print_str("OS: Starting...\n\r");
	print_str("OS: First create task 1\n\r");
	create_task(&task1_func, 1);
	print_str("OS: Back to OS, create task 2\n\r");
	create_task(&task2_func, 2);

	print_str("OS: Back to OS, create idle task\n\r");
	create_task(&idle, 1);

	print_str("\nOS: Start scheduler!\n\r");

	/* SysTick configuration */
	*SYSTICK_LOAD = 7200000;
	*SYSTICK_VAL = 0;
	*SYSTICK_CTRL = 0x07;

	scheduler();
	return 0;
}