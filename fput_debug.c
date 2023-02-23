#include <stdio.h>
#include "stm32f10x.h"
int fputc(int c,FILE *stream)
{
	return(ITM_SendChar(c));
}

