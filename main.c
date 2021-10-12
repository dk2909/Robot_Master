/*
 * main.c
 *
 *  Created on: Oct 11, 2021
 *      Author: dking
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_timer.h"

#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include "driverlib/timer.h"
#include "driverlib/adc.h"

#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTTiva.h>

#include <time.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>
#include <xdc/cfg/global.h>

///////////////  Definitions ///////////////
// UART , change macros, ctrl+F and replace all occurrences
#define UARTGPIOPERI "SYSCTL_PERIPH_GPIOA"
#define UARTPERI "SYSCTL_PERIPH_UART0"
#define UARTRX "GPIO_PA0_U0RX"
#define UARTTX "GPIO_PA1_U0TX"
#define UARTGPIOPORT "GPIO_PORTA_BASE"
#define UARTGPIOPIN "GPIO_PIN_0 | GPIO_PIN_1"
#define UARTBASE "UART0_BASE"
#define UARTINT "INT_UART0"
#define UARTBAUDE 9600

///////////////  Global Variables ///////////////

int count = 0; // count entered characters in uart

///////////////  Function Prototypes ///////////////

// Hardware
void hw_init(void);
void uart_init(void);
void delay(void);
//UART
void uart_print(const char * , uint32_t);
void uart_cmd_start(void);
void uart_read(void);
void cmd_lookup(char *);

///////////////  Tasks ///////////////

void uart_int_handler(void){
	uint32_t statusInt;

	// get and clear interrupt status
    statusInt = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, statusInt);
    // read uart
    //uart_read();
}

///////////////  Main ///////////////
int main(void){
	hw_init();
	delay();

	uart_cmd_start();
	while(1){
		uart_read();
	}
}
///////////////  Hardware Initialization ///////////////
void hw_init(void)
{
    // set system clock (will be 40 MHz)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    uart_init();

}

void uart_init(void){
    // enable Port for GPIO and UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // configure receiver (Rx)
    GPIOPinConfigure(GPIO_PA0_U0RX);
    // configure PA1 as transmitter (Tx)
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // configure PB0 and PB1 for input
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // configure UART, 9600 8-n-1
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    // enable UART0
    UARTEnable(UART0_BASE);
    // enable interrupts on processor
    IntMasterEnable();
    // enable interrupts on UART0
    IntEnable(INT_UART0);
    // enable interrupts for UART0, Rx and Tx
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

///////////////  Utility Functions ///////////////

/// UART ///
// print string to UART
void uart_print(const char * str, uint32_t len){
	uint32_t i;
	for(i = 0; i < len; i++){
		UARTCharPut(UART0_BASE, str[i]);
	}
}

// displays prompt for user to enter command
void uart_cmd_start(void){
	uart_print("Enter Command: ", 15);
}

// read uart input and print output command
void uart_read(void) {
	char cmd_arr[2];
	char cmd_char;
    // read command
    while(UARTCharsAvail(UART0_BASE)){
    	// echo input into prompt, non-blocking here
    	cmd_char = UARTCharGetNonBlocking(UART0_BASE);
    	UARTCharPutNonBlocking(UART0_BASE, cmd_char);
    	// store character
    	cmd_arr[count] = cmd_char;
    	count++;
    	// return if we already have two characters
    	if(count == 2){
    		// carriage return and new line
    		UARTCharPut(UART0_BASE, '\r');
    		UARTCharPut(UART0_BASE, '\n');
    		// reset counter
    		count = 0;
    		// interpret command
    		// print interpretation with uart_print
    		cmd_lookup(cmd_arr);
    	}
    }
}

void cmd_lookup(char command[]) {
	if (command[0] == 'f' && command[1] == 'o'){
		uart_print ("command forward", 15);
	} else if (command[0] == 's' && command[1] == 't'){
		uart_print ("command stop", 12);
	} else if (command[0] == 'e' && command[1] == 'r'){
		uart_print ("command error", 13);
	} else {
		uart_print ("command not found", 17);
	}
	UARTCharPut(UART0_BASE, '\r');
	UARTCharPut(UART0_BASE, '\n');
	UARTCharPut(UART0_BASE, '\n');
	uart_cmd_start();
}

/// Delay ///

void delay(void)
{
	SysCtlDelay(6700000); // ~500ms delay
}
