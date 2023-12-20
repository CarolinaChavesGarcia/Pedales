/* Copyright 2023, Carolina Chaves Garcia
 * carolina.chaves@uner.edu.co
 * All rights reserved.
 * RX Pedales
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @section wiring Wiring
 * ##Transmitter - Reciber NRF24L01 ##
 *
 * | NRF24L01 pins (PTX) | CIAA pins 	 |
 * |:-------------------:|:-------------:|
 * |         VCC         |    +3.3V      |
 * |         GND         |    GND        |
 * |         CSN         |    CIAA_GPIO0 |
 * |         CE	         |    CIAA_GPIO2 |
 * |         SCK         |    SPI_SCK    |
 * |         MOSI        |    SPI_MOSI   |
 * |         IRQ         |    			 |
 * |         MISO        |    SPI_MISO   |
 *
 */
/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
#include "chip.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SYSTICK_CALL_FREC	100  /*call SysTick every 10ms 1/100Hz*/
#define UART_BAUDRATE 115200

typedef struct {
	float force_node; /** <= force in [Kg]*/
	float battery_voltage; /** <= voltage in battery [Volts]*/
	bool data_ready; /** <= data ready flag*/
} nrf24l01p_pedal_data;

/*=====[Definitions of extern global variables]==============================*/
int decimalPlaces = 5;
float PI;
float PD;
/*=====[Definitions of public global variables]==============================*/

/* RX_data[0] Store Pedal Left data
 * RX_data[1] Store Pedal Rigth data
 */
nrf24l01p_pedal_data RX_data[2] = { 0 };

/** Flag for print data in serial port */
static volatile bool flag_serial_data_print = FALSE;
/*=====[Definitions of private global variables]=============================*/

/*==================[Init_Hardware]==========================================*/
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(UART_BAUDRATE);
	for (uint8_t var = 0; var < 8; var++) {
		GPIOInit(CIAA_DO0 + var, GPIO_OUTPUT);
		GPIOInit(CIAA_DI0 + var, GPIO_INPUT);
	}
	//GPIOInit(CIAA_GPIO0, GPIO_INPUT);
	//GPIOInit(CIAA_GPIO1, GPIO_INPUT);
	//GPIOInit(CIAA_GPIO3, GPIO_INPUT);

	//Chip_SCU_PinMuxSet(0X06, 0X01, 0); /* Asocia el pin fisico del micro a un gpio */
	//Chip_GPIO_SetPinDIR(3, gpio[pin].gpioPort, gpio[pin].gpioPin,io); /* Indica si el gpio sera de salida o entrada */
		//Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin,FALSE); /* Inicializa el gpio en 0 */
}

void updateNrfData(void) {
	float float_data = 0.0f;
	memcpy(&float_data, &rcv_fr_PTX[1], sizeof(float_data)); /*Convert array data to float data*/
	if (rcv_fr_PTX[0] == 0x01) { /* Pedal L */
		RX_data[0].data_ready = true;
		RX_data[0].force_node = float_data;
	}
	if (rcv_fr_PTX[0] == 0x02) { /* Pedal R */
		RX_data[1].data_ready = true;
		RX_data[1].force_node = float_data;
	}
}

void clear_array(void) {
	memset(rcv_fr_PTX, 0x00, 32); //Set array of data input whit zeros
}

/*void print_serial_data(nrf24l01p_pedal_data *rx_buffer) {
	// Protocol Serial:
	 // length: 1 + 4*N bytes
	 // |0xFF|4 x data_float|....|4 x data_float|

	Chip_UART_SendByte(USB_UART, 0xff);
	//Chip_UART_SendBlocking(USB_UART, &(rx_buffer)->force_node, sizeof(float));
	//Chip_UART_SendBlocking(USB_UART, &(rx_buffer + 1)->force_node,sizeof(float));
	// Convert to ASCII & send by UART
	//floatToASCIIAndSend((rx_buffer)->force_node);
	Chip_UART_SendByte(USB_UART, 0x44);
	Chip_UART_SendByte(USB_UART, 0x0A);
	floatToASCIIAndSend((rx_buffer + 1)->force_node);

}*/

/*
void uint32ToASCIIAndSend(uint32_t value) {
    char asciiRepresentation[11]; // Máximo 10 dígitos + carácter nulo

    // Convert uint32_t to ASCII
    int index = 0;

    if (value == 0) {
        // Special case value 0
        asciiRepresentation[index++] = '0';
    } else {
    	//Convert each digit to ASCII
        while (value > 0) {
            asciiRepresentation[index++] = '0' + (value % 10);
            value /= 10;
        }
    }

    // Correct order
    for (int i = 0; i < index / 2; ++i) {
        char temp = asciiRepresentation[i];
        asciiRepresentation[i] = asciiRepresentation[index - i - 1];
        asciiRepresentation[index - i - 1] = temp;
    }

    // Send each character to UART with delay
    for (int i = 0; i < 11; ++i) {
    	Chip_UART_SendByte(USB_UART, asciiRepresentation[i]);
    	DelayMs(50);
    }
    Chip_UART_SendByte(USB_UART, 0x0A);
    Chip_UART_SendByte(USB_UART, 0x0D);
}*/

void floatToASCIIAndSend(float value) {
    int32_t intValue = (int32_t)value; // Integer part
    float decimalPart = value - intValue; // Decimal part
    char asciiRepresentation[11]; // Máximo 10 dígitos + carácter nulo

    // Convert integer part to ASCII
    int32_t absIntValue = abs(intValue);
    int index = 0;

    if (absIntValue == 0) {
        // Special case value 0
    	Chip_UART_SendByte(USB_UART, 0x30);
        //uart_send_char('0');
    } else {
        // Convert each digit of integer part to ASCII
        while (absIntValue > 0) {
        	//Chip_UART_SendByte(USB_UART, 0x30 + (absIntValue % 10));		//Print integer part
            //uart_send_char('0' + (absIntValue % 10));
        	asciiRepresentation[index++] = '0' + (absIntValue % 10);
            absIntValue /= 10;
        }
    }

    // Correct order
    if (intValue != 0) {
       for (int i = 0; i < index / 2; ++i) {
            char temp = asciiRepresentation[i];
            asciiRepresentation[i] = asciiRepresentation[index - i - 1];
            asciiRepresentation[index - i - 1] = temp;
            for (int i = 0; i < index; ++i) {
            	Chip_UART_SendByte(USB_UART, asciiRepresentation[i]);
            	//Chip_UART_SendBlocking(USB_UART, &asciiRepresentation[i],sizeof(asciiRepresentation));
                DelayMs(100);
            }
    	}
    }

    // Enviar el separador decimal
    Chip_UART_SendByte(USB_UART, 0x2E);

    // Convertir la parte decimal a ASCII
    for (int i = 0; i < sizeof(decimalPart); ++i) {
        decimalPart *= 10;
        Chip_UART_SendByte(USB_UART, 0x30 + (int)decimalPart);
        //Chip_UART_SendBlocking(USB_UART, &("PD = "),sizeof("PD = "));
        DelayMs(100);
        decimalPart -= (int)decimalPart;
    }
    Chip_UART_SendByte(USB_UART, 0x0A);
    Chip_UART_SendByte(USB_UART, 0x0D);
}
/*==================[SystickHandler]=========================================*/
/** Variable used for SysTick Counter */
static volatile uint32_t cnt = 0;
void SysTick_Handler(void) {
	if ((cnt % 5) == 0) { 		//flag change every 50 ms
		flag_serial_data_print = TRUE;
	}
	if (cnt == 50) { //toggle every 500 ms
		GPIOToggle(CIAA_DO4);

		cnt = 0;
	}
	cnt++;
}

/*=====[Main function, program entry point after power on or reset]==========*/
int main(void)
{

	/* perform the needed initialization here */
	SystemClockInit();
	Init_Hardware();
	nrf24l01_t RX;
	RX.spi.cfg = nrf24l01_spi_default_cfg;
	RX.cs = CIAA_GPIO0;
	RX.ce = CIAA_GPIO2;
	RX.irq = CIAA_GPIO1;
	RX.mode = PRX;
	RX.en_ack_pay = FALSE;

	Nrf24Init(&RX);
	Nrf24SetRXPacketSize(&RX, 0x00, 32); // Set length of pipe 0 in 32 (used for the Pedal Left)
	Nrf24SetRXPacketSize(&RX, 0x01, 32); // Set length of pipe 1 in 32 (used for the Pedal Right)
	Nrf24EnableRxMode(&RX); /* Enable RX mode */
	Nrf24SecondaryDevISRConfig(&RX); /* Config ISR (only use one module in PRX mode on board)*/

	SysTick_Config(SystemCoreClock / SYSTICK_CALL_FREC);

	float float_data = 0.0f;

	float floatValue = 123.456;


	while (TRUE)
	{


		//floatToASCIIAndSend(floatValue, decimalPlaces);
		//floatToASCIIAndSend(floatValue);
		//floatToASCIIAndSend(PD);


		/******* Pedals *******/
		memcpy(&float_data, &rcv_fr_PTX[1], sizeof(float_data)); /*Convert array data to float data*/

		if (rcv_fr_PTX[0] == 0x01) {/*Pedal L*/
			RX_data[0].data_ready = true;
			RX_data[0].force_node = float_data;
			GPIOOn(CIAA_DO7);
			PI = float_data;
			if (float_data > 0.2) {
				GPIOOn(CIAA_DO3);
			} else {
				GPIOOff(CIAA_DO3);
			}
		}
		if (rcv_fr_PTX[0] == 0x02) {/*Pedal R*/
			RX_data[1].data_ready = true;
			RX_data[1].force_node = float_data;
			GPIOOn(CIAA_DO6);
			PD = float_data;
			if (float_data > 0.2) {
				GPIOOn(CIAA_DO2);
			} else {
				GPIOOff(CIAA_DO2);
			}
		}

		if (flag_serial_data_print) {
			//print_serial_data(&PI, &PD);
			/*Chip_UART_SendByte(USB_UART, 0x49);
			Chip_UART_SendByte(USB_UART, 0x0A);
			floatToASCIIAndSend(PI);
			Chip_UART_SendByte(USB_UART, 0x44);
			Chip_UART_SendByte(USB_UART, 0x0A);
			floatToASCIIAndSend(PD);*/
			Chip_UART_SendBlocking(USB_UART, &("PI ="),sizeof("PI ="));
			floatToASCIIAndSend((RX_data)->force_node);
			Chip_UART_SendBlocking(USB_UART, &("PD = "),sizeof("PD = "));
			floatToASCIIAndSend((RX_data + 1)->force_node);
			clear_array();
			flag_serial_data_print = FALSE;/*Clear the flag*/
		}


	};

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.

	return 0;

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

