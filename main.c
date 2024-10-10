/*
   FreeRTOS self made Scheduler that dinamically selects priority of tasks by knowing its deadline.
   Created for the embedded system TM4C129 https://www.ti.com/product/TM4C129ENCPDT
   Global variables were necessary for many of the variables when using the free version of Keil uVision.
   All global variables are Uppercase ex: Sum_number. Different from normal C convention ex: g_sum_number.

   NOTE: Definitions are normally put in a header file, but for easier undertanding I put them here.

   Author: Alexandre Felipe Jorge 
   Date: 2020/07/12
   ---------------------------------------------------------------------------------------------------
   EDITED: 2023/08/17
   Author: Alexandre Felipe Jorge
   Changelog: 
   - Translation of documentation.
   - Fixed variables to follow naming convention.
   - Optimized Factorial function.
   - Made some tests.
   - Created new definitions.
   - Created UART macro.
   - Removed many unecessary variables.
*/

#include "cmsis_os.h"
#include "TM4C129.h"
#include "math.h"

#define NUMBER_OF_THREADS 6
#define TASK_A_ITERATIONS 8
#define TASK_B_ITERATIONS 2
#define TASK_C_ITERATIONS 5
#define TASK_D_ITERATIONS 1
#define TASK_E_ITERATIONS 6
#define TASK_F_ITERATIONS 10

// Do not go over 256, the counting variable has 8 bits (uint8_t Timer).
#define RESET_TIMER 200
// Timer is in miliseconds
#define TIME_CALL_SCHEDULER 5
// true for debugging
#define DEBUG true
// For easier UART use, uart0 is in GPIO header, UART is only for DEBUG.
#define print_uart uart_sendstr(uart0, __VA_ARGS__);

//State of the thread, used my own instead of the header file because it caused a strage delay, I sent a report to FreeRTOS.
enum {READY = 1, RUNNING, WAITING};

//Contains required information from every created thread.

typedef struct
{
	uint32_t task_ticks;
	float task_deadline;
	int task_static_priority;
	int task_dinamic_priority = 0;
	osThreadId thread_id;
	uint8_t state = READY;
	//Secondary fault means the task has passed its dealine or had too much time left, and needs to change priority.
	uint16_t secondary_fault = 0;
	uint16_t progress_to_complete = 0;
	uint32_t tick_master = 0;
	uint8_t times_thread_run = 0;
}	Thread_info;

Thread_info Thread[6];

//Used for the thread timer
osTimerId Timer_id;

//Thread signals, has to be global for other tasks to access.
int32_t Signals_schedular, Signals_A, Signals_B, Signals_C, Signals_D, Signals_E, Signals_F;

//Thread ID
osThreadId Thread_id_scheduler;
				
//Resets system after a number of times run, so that it does everything again.
uint8_t Timer = 0;

//Used for tasks B and D. Calculates the factorial of a given integer number, since it is unsigned no need to check if negative.
float factorial(uint32_t n)
{
	if (n == 0 || n == 1) 
	{
        return 1;
    } 
	else 
	{
		//Recursion
        return n * factorial(n - 1);
    }
}

// Initializes each thread.
bool init_setup()
{
	// These variables cannot be random as an RTOS has to have a reasonable dealine for tasks. They were collected.
	Thread[0].task_ticks = 8618;
	Thread[0].task_deadline = 1.7;
	Thread[0].task_static_priority = 10;
	
	Thread[1].task_ticks = 72606;
	Thread[1].task_deadline = 1.5;
	Thread[1].task_static_priority = 0;
	
	Thread[2].task_ticks = 5793;
	Thread[2].task_deadline = 1.3;
	Thread[2].task_static_priority = -30;
	
	Thread[3].task_ticks = 578;
	Thread[3].task_deadline = 1.5;
	Thread[3].task_static_priority = 0;
	
	Thread[4].task_ticks = 25190;
	Thread[4].task_deadline = 1.3;
	Thread[4].task_static_priority = -30;
	
	Thread[5].task_ticks = 8877;
	Thread[5].task_deadline = 1.1;
	Thread[5].task_static_priority = -100;

	for(uint8_t i = 0; i < NUMBER_OF_THREADS; i++)
	{
		if(Thread[i])
		{
			//Do nothing, it exists and was properly created
		}
		else
		{
			//Not good, someone was not created properly, restart system at MAIN.
			if(DEBUG)
			print_uart("Thread[%d] was not created properly!", i);
			return false;
		}
	}
	return true;
}

//Checks if deadline was not met, if so increases priority of the task, if it had a lot of time left decrease priority.
void system_fault_checker(uint8_t process_id)
{
	if((osKernelSysTick() - Thread[process_id].tick_master) < Thread[process_id].task_ticks*(Thread[process_id].task_deadline - ((1 - Thread[process_id].task_deadline)/2)))
	{
		Thread[process_id].secondary_fault++;
		if(Thread[process_id].task_static_priority != 100 && Thread[process_id].task_static_priority != -100)
		{
			Thread[process_id].task_static_priority = Thread[process_id].task_static_priority + 10;
		}			
	}
	if((osKernelSysTick() - Thread[process_id].tick_master) > Thread[process_id].task_ticks * Thread[process_id].task_deadline)
	{
		Thread[process_id].secondary_fault++;
		if(Thread[process_id].task_static_priority == -100)
		{
			if(DEBUG)
			main_fault_check(process_id);
		}
		if(Thread[process_id].task_static_priority != -100)
		{
			Thread[process_id].task_static_priority = Thread[process_id].task_static_priority - 10;
		}		
	}
}

// FOR DEBUG. How much of the task was completed with time allocated.
void percentage_complete(int time_run, int time_to_complete, int thread_id)
{
	Thread[thread_id].progress_to_complete = (float)((float)time_run/(float)time_to_complete)*100;
}

/*----------------------------------------------------------------------------
 *      Threads
 *---------------------------------------------------------------------------*/
void threads_scheduler(void const *argument){
	uint8_t next_thread = 0xff, priority_scheduler = 0xff;
	while(1)
	{
		// Wait for timer
		osSignalWait(Signals_schedular, osWaitForever);
		
		//Change dinamic priority, using how long the task has to complete the deadline.
		for(uint8_t i; i < NUMBER_OF_THREADS; i++)
		{
			Thread[i].task_dinamic_priority = ((osKernelSysTick() - Thread[i].tick_master)/((Thread[i].task_ticks*Thread[i].task_deadline) - Thread[i].task_ticks))*100;
			//Always results in a negative number, change it here so it is easier to read.
			Thread[i].task_dinamic_priority = -Thread[i].task_dinamic_priority;
		}
		
		//Checks next thread to be called		
		for(uint8_t i_scheduler = 0; i_scheduler < 6; i_scheduler++)
		{
			if(Thread[i_scheduler].state != WAITING)
			{
				Thread[i_scheduler].state = READY;
				if(priority_scheduler > (Thread[i_scheduler].task_static_priority + Thread[i_scheduler].task_dinamic_priority))
				{
					next_thread = i_scheduler;
					priority_scheduler = Thread[i_scheduler].task_static_priority + Thread[i_scheduler].task_dinamic_priority;
				}
			}
		}
		// Reset variable.
		priority_scheduler = 0xff;

		// Selects which thread goes next.
		switch (next_thread) 
		{
		case 0:
			osSignalSet(Thread[0].thread_id, 0x01);
			break;
		case 1:
			osSignalSet(Thread[1].thread_id, 0x01);
			break;
		case 2:
			osSignalSet(Thread[2].thread_id, 0x01);
			break;
		case 3:
			osSignalSet(Thread[3].thread_id, 0x01);
			break;
		case 4:
			osSignalSet(Thread[4].thread_id, 0x01);
			break;
		case 5:
			osSignalSet(Thread[5].thread_id, 0x01);
			break;
		default:
			// Should never get here;
			if(DEBUG)
			non_existent_thread_called(next_thread)
        break;
		}
		
		Timer++;
		// Restart everyone.
		if(Timer >= RESET_TIMER)
		{
			Timer = 0;
			for(uint8_t i = 0; i < NUMBER_OF_THREADS; i++)
			{
				Thread[i].state = READY;
				Thread[i].times_thread_run = 0;
			}
			if(DEBUG)
			result_collection();
		}
	}
}
osThreadDef(threads_scheduler, osPriorityAboveNormal, 1, 0);

// Timer to call priority to the schedular after some time defined in TIME_CALL_SCHEDULER.
void timer_Callback(void const *argument)
{	
	osSignalSet(Thread_id_scheduler, 0x01);
}
osTimerDef (Timer, timer_Callback); 

void threadA(void const *argument){
	uint32_t sumA = 0;
	while(1)
	{
		// Wait for Scheduler
		osSignalWait(Signals_A, osWaitForever);
		// Collect time task started.
		Thread[0].tick_master = osKernelSysTick();
		Thread[0].state = RUNNING;
		
		// Some arithmetics to use CPU time.
		for(uint8_t xA = 0; xA <= 256; xA++)
		{
			sumA = sumA + (xA + (xA + 2));
			if(DEBUG)
			percentage_complete(xA, 256, 0);
		}
		// Reset variable
		sumA = 0;
		Thread[0].times_thread_run++;
		
		// Check if it met dealine.
		system_fault_checker(0);
		
		// Check if ran maximum amount of times in this iteration.
		if(Thread[0].times_thread_run >= TASK_A_ITERATIONS)
		{
			Thread[0].state = WAITING;
		}
		
	}
	
}
// Do not change priority! It is the schedulers job.
osThreadDef(threadA, osPriorityNormal, 1, 0);

void threadB(void const *argument){
	float sumB = 0, 

	while(1)
	{
		// Wait for Scheduler
		osSignalWait(Signals_B, osWaitForever);
		// Collect time task started.
		Thread[1].tick_master = osKernelSysTick();
		Thread[1].state = RUNNING;
		
		// Some arithmetics to use CPU time.
		for(uint8_t nB = 1; nB <= 16; nB++)
		{
			sumB = sumB + (pow(2, nB)/factorial(nB));
			if(DEBUG)
			percentage_complete(nB, 16, 1);
		}
		// Reset variable
		sumB = 0;
		Thread[1].times_thread_run++;
		
		// Check if it met dealine.
		system_fault_checker(1);
		
		// Check if ran maximum amount of times in this iteration.
		if(Thread[1].times_thread_run >= TASK_B_ITERATIONS)
		{
			Thread[1].state = WAITING;
		}
	}
}
// Do not change priority! It is the schedulers job.
osThreadDef(threadB, osPriorityNormal, 1, 0);

void threadC(void const *argument){
	float sumC = 0;
	while(1)
	{
		// Wait for Scheduler
		osSignalWait(Signals_C, osWaitForever);
		// Collect time task started.
		Thread[2].tick_master = osKernelSysTick();
		Thread[2].state = RUNNING;
		
		// Some arithmetics to use CPU time.
		for(uint8_t nC = 1; nC <= 72; nC++)
		{
			sumC = sumC + ((nC + 1)/nC);
			if(DEBUG)
			percentage_complete(nC, 72, 2);
		}
		// Reset variable
		sumC = 0;
		Thread[2].times_thread_run++;
		
		// Check if it met dealine.
		system_fault_checker(2);
		
		// Check if ran maximum amount of times in this iteration.
		if(Thread[2].times_thread_run >= TASK_C_ITERATIONS)
		{
			Thread[2].state = WAITING;
		}
	}
}
// Do not change priority! It is the schedulers job.
osThreadDef(threadC, osPriorityNormal, 1, 0);

void threadD(void const *argument){
	float sumD = 0;
	while(1)
	{
		// Wait for Scheduler
		osSignalWait(Signals_D, osWaitForever);
		// Collect time task started.
		Thread[3].tick_master = osKernelSysTick();
		Thread[3].state = RUNNING;
		
		// Some arithmetics to use CPU time
		sumD = 1 + (5/factorial(3)) + (5/factorial(5)) + (5/factorial(7)) + (5/factorial(9));
		Thread[3].times_thread_run++;
		
		// Check if it met dealine.
		system_fault_checker(3);
		
		// Check if ran maximum amount of times in this iteration.
		if(Thread[3].times_thread_run >= TASK_D_ITERATIONS)
		{
			Thread[3].state = WAITING;
		}
	}
}
// Do not change priority! It is the schedulers job.
osThreadDef(threadD, osPriorityNormal, 1, 0);

void threadE(void const *argument){
	float sumE = 0;
	while(1)
	{
		// Wait for Scheduler
		osSignalWait(Signals_E, osWaitForever);
		// Collect time task started.
		Thread[4].tick_master = osKernelSysTick();
		Thread[4].state = RUNNING;
		
		// Some arithmetics to use CPU time
		for(uint8_t xE = 1; xE <= 100; xE++)
		{
			sumE = sumE + (xE*(3.14159265359*3.14159265359));
			if(DEBUG)
			percentage_complete(xE, 100, 4);
		}
		// Reset variable
		sumE = 0;
		Thread[4].times_thread_run++;
		
		// Check if it met dealine.
		system_fault_checker(4);
		
		// Check if ran maximum amount of times in this iteration.
		if(Thread[4].times_thread_run >= TASK_E_ITERATIONS)
		{
			Thread[4].state = WAITING;
		}
	}
}
// Do not change priority! It is the schedulers job.
osThreadDef(threadE, osPriorityNormal, 1, 0);

void threadF(void const *argument){
	float sumF = 0;
	while(1)
	{
		// Wait for Scheduler
		osSignalWait(Signals_F, osWaitForever);
		// Collect time task started.
		Thread[5].tick_master = osKernelSysTick();
		Thread[5].state = RUNNING;
		
		// Some arithmetics to use CPU time
		for(uint8_t Yf = 1; Yf <= 128; Yf++)
		{
			sumF = sumF + ((Yf*Yf*Yf)/(Yf*Yf));
			if(DEBUG)
			percentage_complete(Yf, 128, 5);
		}
		// Reset variable
		sumF = 0;
		Thread[5].times_thread_run++;
		
		// Check if it met dealine.
		system_fault_checker(5);
		
		// Check if ran maximum amount of times in this iteration.
		if(Thread[5].times_thread_run >= TASK_F_ITERATIONS)
		{
			Thread[5].state = WAITING;
		}
	}
}
// Do not change priority! It is the schedulers job.
osThreadDef(threadF, osPriorityNormal, 1, 0);

/*----------------------------------------------------------------------------
 *      TESTS
 *---------------------------------------------------------------------------*/
// To turn on remember to put true in DEBUG definition.
// Do not force the CPU to stop here, you will activate the Watch Dog and it resets the system after a little while.

// If here, then system failed to create balance to meet dealines.
void main_fault_check(uint8_t process_id)
{
	print_uart("Process id Failed: %d\n", process_id);
	print_uart("Progress made: %d\n", Thread[process_id].progress_to_complete);
}

// Unexistent Thread Called.
void non_existent_thread_called(uint8_t next_thread)
{
	print_uart("Thread called: %d\n", next_thread);
	print_uart("Number of threads created: %d\n", NUMBER_OF_THREADS);
}

// Called after all iterations, gives the results.
void result_collection()
{
	for(uint8_t i = 0; i < NUMBER_OF_THREADS; i++)
	{
		print_uart("Thread[%d].secondary_fault: %d\n", i, Thread[i].secondary_fault);
		print_uart("Thread[%d].task_dinamic_priority: %d\n", i, Thread[i].task_dinamic_priority);
	}
}

/*----------------------------------------------------------------------------
 *      Main
 *---------------------------------------------------------------------------*/
int main (void) {
	
	if(!init_setup()){
		//System crash, it will force restart.
		return 0;
	}

	if(DEBUG)
	{
		uart_start();
	}
	
	osKernelInitialize();  // Initialize Kernel
 
	// Create the threads.
	Thread_id_scheduler = osThreadCreate(osThread(threads_scheduler), NULL);
	Thread[0].thread_id = osThreadCreate(osThread(threadA), NULL);
	Thread[1].thread_id = osThreadCreate(osThread(threadB), NULL);
	Thread[2].thread_id = osThreadCreate(osThread(threadC), NULL);
	Thread[3].thread_id = osThreadCreate(osThread(threadD), NULL);
	Thread[4].thread_id = osThreadCreate(osThread(threadE), NULL);
	Thread[5].thread_id = osThreadCreate(osThread(threadF), NULL);
	
	//Signals
	Signals_schedular = osSignalSet(Thread_id_scheduler, 0x00000003);
	Signals_A = osSignalSet(Thread[0].thread_id, 0x00000004);
	Signals_B = osSignalSet(Thread[1].thread_id, 0x00000005);
	Signals_C = osSignalSet(Thread[2].thread_id, 0x00000006);
	Signals_D = osSignalSet(Thread[3].thread_id, 0x00000007);
	Signals_E = osSignalSet(Thread[4].thread_id, 0x00000008);
	Signals_F = osSignalSet(Thread[5].thread_id, 0x00000009);
	
	//Timer
	Timer_id = osTimerCreate(osTimer(Timer), osTimerPeriodic, NULL);
	
	//Starts the timer
	osTimerStart(Timer_id, TIME_CALL_SCHEDULER);

	// Turns Kernel ON
	osKernelStart();
	
	// Put the main function on wait forever.
	osDelay(osWaitForever);  
}