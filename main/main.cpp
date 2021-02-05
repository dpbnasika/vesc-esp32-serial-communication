#include <stdio.h>
#include <iostream>
#include <math.h>
#include "VescUart.h"
using namespace std; // @suppress("Symbol is not resolved")

extern "C" {
void app_main();
}

void app_main()
{
	VescUart UART(UART_NUM_2,115200); // @suppress("Invalid arguments")
	while(1)
	{
		UART.setDuty(-0.1);
		if(UART.getVescValues()){
			UART.printVescValues();
		}

	}
}
