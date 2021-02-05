#ifndef VESC

#include "VescUart.h"
#include <stdio.h>
#include <cstring>
#include <iostream>

using namespace std;

#define BUF_SIZE 1024

VescUart::VescUart(int uart_port, int baud_rate)
{

	uartPort = uart_port;
	baudRate = baud_rate;
	uart_config_t uart_config = {
			.baud_rate = baudRate,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(uartPort, &uart_config);
	uart_set_pin(uartPort, TX2, RX2, 18, 19);
	uart_driver_install(uartPort, BUF_SIZE * 2, 0, 0, NULL, 0);
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived)
{
	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	uint8_t data[256];
	int length = 0;

	uint32_t timeout = (esp_timer_get_time()/1000) + 100; // Defining the timestamp for timeout (100ms before timeout)

	while ( (esp_timer_get_time()/1000) < timeout && messageRead == false)
	{
		while (uart_is_driver_installed(uartPort))
		{
			uart_get_buffered_data_len(uartPort, (size_t*)&length);
			uart_read_bytes(uartPort, messageReceived, length, 100);

			if (messageReceived[0] == 2)
			{
				switch (messageReceived[0])
				{
				case 2:
					endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
					lenPayload = messageReceived[1];
					break;

				case 3:
					// ToDo: Add Message Handling > 255 (starting with 3)
					break;

				default:

					break;
				}
			}
			if (messageReceived[endMessage - 1] == 3)
			{
				messageReceived[endMessage] = 0;
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}

	bool unpacked = false;

	if (messageRead)
	{
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked)
	{
		// Message was read
		return lenPayload; 
	}
	else
	{
		// No Message Read
		return 0;
	}
}

bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload)
{
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if (crcPayload == crcMessage)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int VescUart::packSendPayload(uint8_t * payload, int lenPay)
{
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';

	// Sending package
	uart_write_bytes(uartPort, messageSend, count);

	// Returns number of send bytes
	return count;
}

bool VescUart::processReadPacket(uint8_t * message)
{
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
	case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

		data.tempFET            = buffer_get_float16(message, 10.0, &ind);
		data.tempMotor          = buffer_get_float16(message, 10.0, &ind);
		data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
		data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
		ind += 8; // Skip the next 8 bytes
		data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
		data.rpm 				= buffer_get_int32(message, &ind);
		data.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
		data.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
		data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
		ind += 8; // Skip the next 8 bytes
		data.tachometer 		= buffer_get_int32(message, &ind);
		data.tachometerAbs 		= buffer_get_int32(message, &ind);
		return true;

		break;

	default:
		return false;
		break;
	}
}

bool VescUart::getVescValues(void)
{
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];

	packSendPayload(command, 1);
	vTaskDelay(500 / portTICK_PERIOD_MS); //needed, otherwise data is not read // @suppress("Invalid arguments")
	int lenPayload = receiveUartMessage(payload);
	if (lenPayload > 55)
	{
		bool read = processReadPacket(payload); //returns true if successful
		return read;
	}
	else
	{
		return false;
	}
}

void VescUart::setCurrent(float current)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setBrakeCurrent(float brakeCurrent)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5);
}

void VescUart::setDuty(float duty)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5);
}

void VescUart::printVescValues()
{
	printf("--------------- Motor Readings -------------\n");
	printf("Avg Motor Current:     %lf\n",data.avgMotorCurrent);
	printf("Avg Input Current:     %lf\n",data.avgInputCurrent);
	printf("Duty Cycle Now:        %lf\n",data.dutyCycleNow);
	printf("Rpm:                   %ld\n",data.rpm);
	printf("Input Voltage:         %lf\n",data.inpVoltage);
	printf("AmpHours:              %lf\n",data.ampHours);
	printf("AmpHoursCharges:       %lf\n",data.ampHoursCharged);
	printf("Tachometer:            %ld\n",data.tachometer);
	printf("Tachometer Abs:        %ld\n",data.tachometerAbs);
	printf("Temperature of Fet's:  %f\n",data.tempFET);
	printf("Motor temperature:     %f\n",data.tempMotor);
}

#endif
