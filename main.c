#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "tasks_func.h"

// Define the number of tasks to be executed
#define NUM_TASKS 5

// Define the starting address of the stack
#define STACK_START 0x20002000

// Define the size of the task stack
#define STACK_SIZE 128


// Define the task control block structure
typedef struct tcb {
    uint32_t *sp;   // Stack pointer
//    int next;   // Pointer to the next task
	 struct tcb *next;   // Pointer to the next task
	  int curr;
} tcb_t;

// Declare the task control blocks for each task
static tcb_t tcb[NUM_TASKS];

// Declare the stacks for each task
//static uint32_t task_stacks[NUM_TASKS][STACK_SIZE];

// Declare the current task and the next task to be executed
static tcb_t *current_task;
static tcb_t *next_task;

//count variable for different tasks
static int timecount,count15,timintc;

volatile int curr_task_no;


// Declare a variable to hold the stack pointer
void stack_init(int);

//System Clock configuration
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}


uint32_t select_task(int t)
{
	switch(t)
	{
		case 1: return (uint32_t)&task1;
						break;
		case 2: return (uint32_t)&task2;
				    break;
		case 3: return (uint32_t)&task3;
				    break;
		case 4: return (uint32_t)&task4;
				    break;
		case 5: return (uint32_t)&task5;
				    break;
	}
}



void Systick_initialize (uint32_t ticks)
{
SysTick ->CTRL = 0; 
SysTick->LOAD = ticks - 1; 
SCB->SHP[11]=0;
SysTick->VAL = 0; 
SysTick->CTRL |= 7;//enable clock source, interrupt, enable systick	
}


void SysTick_Handler(void)
{
	 if(timecount<500)
	 {
		 timecount++;
	 }
	 else
	 {
		 SCB->ICSR =(1<<28);
		 timecount=0;
		 timintc++;
	 }
	 
	 if(timintc==3)
	 {
		 timintc=0;
		 NVIC->ISPR[0]=(1<<29);
	 }

}
// Declare the OS initialization function
void SVC_Handler(void)
{
    // Initialize the task control blocks
		uint32_t svc_no;
		__asm volatile(	"TST LR,#4");													
		__asm volatile("ITE EQ	\n"				
							"MRSEQ R0,MSP	\n"			
							"MRSNE R0,PSP\n");			
		__asm volatile(	"LDR R0,[R0,#24]");			
		__asm volatile(	"LDRB R0,[R0,#-2]");			
		__asm volatile(	"MOV %[svc_no],R0":[svc_no]"=r"(svc_no));
			
		if(svc_no==0xF3)
		{
   // Initialize the task stack location
		tcb[0].sp = (uint32_t*)(STACK_START+ 1*STACK_SIZE);
    tcb[1].sp = (uint32_t*)(STACK_START+ 2*STACK_SIZE);
    tcb[2].sp = (uint32_t*)(STACK_START+ 3*STACK_SIZE);
    tcb[3].sp = (uint32_t*)(STACK_START+ 4*STACK_SIZE);
    tcb[4].sp = (uint32_t*)(STACK_START+ 5*STACK_SIZE);
		
//		tcb[0].next = 1;
//    tcb[1].next = 2;
//    tcb[2].next = 3;
//    tcb[3].next = 4;
//    tcb[4].next = 0;
		
		tcb[0].next = &tcb[1];
    tcb[1].next = &tcb[2];
    tcb[2].next = &tcb[3];
    tcb[3].next = &tcb[4];
    tcb[4].next = &tcb[0];
			
		tcb[0].curr = 0;
    tcb[1].curr = 1;
    tcb[2].curr = 2;
    tcb[3].curr = 3;
    tcb[4].curr = 4;
		
		//updating the task control block
		current_task=&tcb[0];
		curr_task_no= current_task->curr+1;
		next_task=current_task->next;

		//initisalisation of stacks
			stack_init(1);
			stack_init(2);
			stack_init(3);
			stack_init(4);
			stack_init(5);
			
		//tcb[0].sp +=4;
			
			//Loading PSP with task1
			__asm volatile("BL curr_psp");  //load task1 psp 
			__asm volatile("LDMFD R0!,{R4-R11}");
			__asm volatile("MSR PSP,r0");
			__asm volatile("LDR LR,=0xFFFFFFFD");	//direct branch using PSP 
			__asm	volatile("MOV R0,#3");	//SPSEL =1(PSP) nPRIV =1 
			__asm volatile("MSR CONTROL, R0");	
 		  __asm volatile("BX LR");
		}


}

uint32_t curr_psp(){
			return ((uint32_t) current_task->sp); //32bit data returned to R0
}
void stack_init(int t)
{
	uint32_t* stackp= tcb[t-1].sp;
	*(--stackp)= 0x01000000; 
	*(--stackp)= select_task(t);
	*(--stackp)= 0xFFFFFFD;
	for(int i=0;i<13;i++)
	*(--stackp)=0;
	
	tcb[t-1].sp=stackp;
}

void context_save(uint32_t* curr_stack)
{
		current_task->sp=  curr_stack;
}

void next_task_select()
{
			current_task=next_task;
			next_task=current_task->next;
	    curr_task_no= current_task->curr+1;
}
// Declare the PendSV handler function
void PendSV_Handler(void)
{
		  __asm volatile(	"PUSH {LR}	");//EXC_RETURN value pushed into stack
			__asm volatile(	"MRS R0, PSP");	
			__asm volatile(	"STMFD R0!, {R4-R11}");	
			__asm volatile("BL context_save				");	//Context save of current task
			__asm volatile(	"BL next_task_select"); //Select next task
			__asm volatile(	"BL curr_psp	");	//load context of next task
			__asm volatile(	"LDMFD R0!, {R4-R11}");	
			__asm volatile(	"MSR PSP, R0");	//Update PSP location to R0
			__asm volatile("POP {PC}");	//EXC_RETURN moved to PC
}

//Timer 3 IRQ handler
void TIM3_IRQHandler(void)
{
	count15++;
}

// Declare the main function
int main(void)
{
	
		SystemClock_Config();


	  NVIC_SetPriorityGrouping(3);
	  NVIC_SetPriority(TIM3_IRQn,3);
	  SCB->SHP[10] = 239;
		NVIC->ISER[0]= (1<<29);
	
		//Systick_initialize(1680000);
	  Systick_initialize(560000);
	__asm ("SVC 0xF3");
	while(1)
	{
		
	}
}
