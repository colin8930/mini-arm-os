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
#define PIROIRTY_LIMIT       5

unsigned int task_count = 0;
unsigned int current_task = 0;
unsigned int user_stacks[TASK_LIMIT][STACK_SIZE];
unsigned int tick_count = 0;
int host_handle;
int prev_task = 0;
int start = 0;

void write(char* buf, int len)
{
	host_action(SYS_WRITE, host_handle, (void *)buf, len);
}
int _snprintf_int(int num, char * buf, int buf_size)
{
	int len = 1;
	char *p;
	int i = num < 0 ? -num : num;

	for (; i >= 10; i /= 10, len++);

	if (num < 0)
		len++;

	i = num;
	p = buf + len - 1;
	do {
		if (p < buf + buf_size)
			*p-- = '0' + i % 10;
		i /= 10;
	} while (i != 0);

	if (num < 0)
		*p = '-';

	return len < buf_size ? len : buf_size;
}

unsigned int get_reload()
{
	return *SYSTICK_LOAD;
}

unsigned int get_current()
{
	return *SYSTICK_VAL;
}

unsigned int get_time()
{
	static const unsigned int scale = 100;  /* microsecond */

	return tick_count * scale + (*SYSTICK_LOAD - *SYSTICK_VAL) / (*SYSTICK_LOAD / scale);
}


int snprintf(char *buf, size_t size, const char *format, ...)
{
	va_list ap;
	char *dest = buf;
	char *last = buf + size;
	char ch;

	va_start(ap, format);
	for (ch = *format++; dest < last && ch; ch = *format++) {
		if (ch == '%') {
			ch = *format++;
			switch (ch) {
			case 's' : {
					char *str = va_arg(ap, char*);
					/* strncpy */
					while (dest < last) {
						if ((*dest = *str++))
							dest++;
						else
							break;
					}
				}
				break;
			case 'd' : {
					int num = va_arg(ap, int);
					dest += _snprintf_int(num, dest, last - dest);
				}
				break;
			case '%' :
				*dest++ = ch;
				break;
			default :
				return -1;
			}
		} else {
			*dest++ = ch;
		}
	}
	va_end(ap);

	if (dest < last)
		*dest = 0;
	else
		*--dest = 0;

	return dest - buf;
}

void trace_task_switch()
{
	if(start){
		char  buf[128];
		int len = snprintf(buf, 128, "switch %d %d %d %d %d %d\n",
	                   prev_task, current_task,
	                   tick_count, 7200000,
	                   7200000, *SYSTICK_VAL);
		write(buf, len);
	}
}
void trace_task_create(int task, unsigned int  priority)
{
	char  buf[128];
	int len = snprintf(buf, 128, "task %d %d %d\n", task, priority, task);
	write(buf, len);
}

struct list {
	struct list *prev;
	struct list *next;
	unsigned int id;
};

struct list ready_queue[PIROIRTY_LIMIT + 1];

void queue_init()
{
	int i;
	for (i = 0; i < PIROIRTY_LIMIT + 1; i++) {
		ready_queue[i].prev = &ready_queue[i];
		ready_queue[i].next = &ready_queue[i];
	}
}

int queue_empty(struct list *list)
{
	if ((list -> next) == list)
		return 1;
	else return 0;
}

void queue_push(struct list *list, struct list *new)
{

	struct list *curr = list;
	while ((curr -> next) != list) curr = curr -> next;
	new -> prev = curr;
	new -> next = list;
	curr -> next = new;
	list -> prev = new;

}

void queue_shift(struct list *list)
{
	struct list *shift = list -> next;
	list -> next = (list -> next) -> next;
	(list -> next) -> prev = list;
	queue_push(list, shift);
}

struct task_TCB {
	unsigned int priority;	/*< The priority of the task where 0 is the lowest priority. */
	unsigned int *sp;	/*< Task stack pointer */
	unsigned int status;	/*< The status of task. */
	unsigned int delayTime;	/*< The tick to resume task */

	struct list list;
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
	(TCBs[task_count].list).id = task_count;
	(TCBs[task_count].list).next = &(TCBs[task_count].list);
	(TCBs[task_count].list).prev = &(TCBs[task_count].list);
	queue_push(&(ready_queue[priority]), &TCBs[task_count].list);
	trace_task_create(task_count, priority);
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
		delay(5000);
	}
}

void task2_func(void)
{
	print_str("task2: Created!\n\r");
	print_str("task2: Now, return to kernel mode\n\r");
	syscall();
	while (1) {
		print_str("task2: Running...\n\r");
		delay(5000);
	}
}

void task3_func(void)
{
	print_str("task3: Created!\n\r");
	print_str("task3: Now, return to kernel mode\n\r");
	syscall();
	while (1) {
		print_str("task3: Running...\n\r");
		delay(5000);
	}
}

void task4_func(void)
{
	print_str("task4: Created!\n\r");
	print_str("task4: Now, return to kernel mode\n\r");
	syscall();
	while (1) {
		print_str("task4: Running...\n\r");
		delay(5000);
	}
}

void task5_func(void)
{
	print_str("task5: Created!\n\r");
	print_str("task5: Now, return to kernel mode\n\r");
	syscall();
	while (1) {
		print_str("task5: Running...\n\r");
		delay(5000);
	}
}

void scheduler()
{
	start = 1;
	while (1) {
		int i;
		tick_count++;
		prev_task = current_task;
		for (i = PIROIRTY_LIMIT; i >= 0; i--) {
			if (!queue_empty(&ready_queue[i])) {
				current_task = (ready_queue[i].next) -> id;
				queue_shift(&(ready_queue[i]));
			}
		}
		TCBs[current_task].sp = activate(TCBs[current_task].sp);
	}
}

void interrupt_enable()
{
	*(USART2_CR1) |= 0x00000020;
	*((__REG)0xE000E104) = 1<<6;
}

int main(void)
{

	usart_init();

	host_handle = host_action(SYS_OPEN, "syslog", 6);

	init_TCB(TCBs);
	queue_init();
	print_str("OS: Starting...\n\r");
	print_str("OS: First create task 1\n\r");
	create_task(&task1_func, 2);
	print_str("OS: Back to OS, create task 2\n\r");
	create_task(&task2_func, 2);
	print_str("OS: Back to OS, create task 3\n\r");
	create_task(&task3_func, 2);
	print_str("OS: Back to OS, create task 4\n\r");
	create_task(&task4_func, 2);
	print_str("OS: Back to OS, create task 5\n\r");
	create_task(&task5_func, 2);


	print_str("\nOS: Start scheduler!\n\r");

	/* SysTick configuration */
	*SYSTICK_LOAD = 7200000;
	*SYSTICK_VAL = 0;
	*SYSTICK_CTRL = 0x07;
	interrupt_enable();
	scheduler();
	return 0;
}