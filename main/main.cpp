#include <stdio.h>
#include "VescUart.h"
#include <iostream>
#include <math.h>
using namespace std;

extern "C" {
void app_main();
}


void app_main()
{
	//test
	VescUart UART(UART_NUM_2,115200);
	while(1)
	{
		UART.setDuty(-0.03);
	}
}
