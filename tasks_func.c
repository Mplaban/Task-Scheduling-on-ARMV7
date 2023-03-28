#include "tasks_func.h"
volatile int count1,count2,count3,count4,count5;
void task1(void)
{
	while(1)
		 count1++;
}

void task2(void)
{
	while(1)
    count2++;
}

void task3(void)
{
	while(1)
    count3++;
}

void task4(void)
{
	while(1)
    count4++;
}

void task5(void)
{
	while(1)
   count5++;
}