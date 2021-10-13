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
#include "driverlib/pwm.h"

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
#define UARTPERI "SYSCTL_PERIPH_UART5"
#define UARTRX "GPIO_PA0_U0RX"
#define UARTTX "GPIO_PA1_U0TX"
#define UARTGPIOPORT "GPIO_PORTA_BASE"
#define UARTGPIOPIN "GPIO_PIN_0 | GPIO_PIN_1"
#define UARTBASE "UART5_BASE"
#define UARTINT "INT_UART5"
#define UARTBAUDE 9600

// PWM
#define PWMFREQ 55

///////////////  Global Variables ///////////////

int count = 0; // count entered characters in uart
uint32_t adcVals; // read adc value of side sensor
uint32_t adcValf; // read adc value of side sensor

///////////////  Function Prototypes ///////////////

// Hardware
void hw_init(void);
void uart_init(void);
void adc_init(void);
void delay(void);
//UART
void uart_print(const char * , uint32_t);
void uart_cmd_start(void);
void uart_read(void);
void cmd_lookup(char *);
// ADC
void side_sensor_read(void);
void front_sensor_read(void);
// PWM
void pwm_init(void);
void pwm_start_motor(void);
// Float to String
void ftoa(float, char *, int);
void reverse(char *, int);
int intToStr(int, char *, int);

///////////////  Tasks ///////////////

void uart_int_handler(void){
	uint32_t statusInt;

	// get and clear interrupt status
    statusInt = UARTIntStatus(UART5_BASE, true);
    UARTIntClear(UART5_BASE, statusInt);
    // read uart
    //uart_read();
}

///////////////  Main ///////////////
int main(void){
	hw_init();
	delay();
	uart_print("Current commands: fo (forward), st (stop), sp (set speed), er (error)", 7+1+9+1+2+1+10+1+2+7+1+2+12+1+2+7+4);
	UARTCharPut(UART5_BASE, '\r');
	UARTCharPut(UART5_BASE, '\n');
	uart_cmd_start();
	while(1){
		uart_read();
	}
}

///////////////  Utility Functions ///////////////

/// UART ///
// print string to UART
void uart_print(const char * str, uint32_t len){
	uint32_t i;
	for(i = 0; i < len; i++){
		UARTCharPut(UART5_BASE, str[i]);
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
    while(UARTCharsAvail(UART5_BASE)){
    	// echo input into prompt, non-blocking here
    	cmd_char = UARTCharGetNonBlocking(UART5_BASE);
    	UARTCharPutNonBlocking(UART5_BASE, cmd_char);
    	// store character
    	cmd_arr[count] = cmd_char;
    	count++;
    	// return if we already have two characters
    	if(count == 2){
    		// carriage return and new line
    		UARTCharPut(UART5_BASE, '\r');
    		UARTCharPut(UART5_BASE, '\n');
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
		UARTCharPut(UART5_BASE, '\r');
		UARTCharPut(UART5_BASE, '\n');
		while(1){
			front_sensor_read();
			side_sensor_read();
			UARTCharPut(UART5_BASE, '\r');
			UARTCharPut(UART5_BASE, '\n');
		}
	} else if (command[0] == 's' && command[1] == 't'){
		uart_print ("command stop", 12);
	} else if (command[0] == 's' && command[1] == 'p'){
		uart_print ("command set motor speed", 23);
	} else if (command[0] == 'e' && command[1] == 'r'){
		uart_print ("command error", 13);
	} else {
		uart_print ("command not found", 17);
	}
	UARTCharPut(UART5_BASE, '\r');
	UARTCharPut(UART5_BASE, '\n');
	UARTCharPut(UART5_BASE, '\n');
	uart_cmd_start();
}

/// ADC ///

void side_sensor_read(void){
	//char cmd_arr[2] = {'s','t'};
	float distance = 0;
	char uartOut[8];
	// clear ADC interrupt
	ADCIntClear(ADC0_BASE, 3);
	// trigger ADC sampling
	ADCProcessorTrigger(ADC0_BASE, 3);
	// read voltage
	ADCSequenceDataGet(ADC0_BASE, 3, &adcVals);
	// convert adc value into cm
	distance = 22700*pow(adcVals, -1.08476) + 0.8;

	// convert float to string
	ftoa(distance, uartOut, 4);
	// write output (UART)
	//UARTCharPut(UART5_BASE, '\n');
	uart_print("Side Distance : ", 17);
	uart_print(uartOut, 8);

	/*
	if(distance < 4){
		uart_print("Side Too close", 14);
		//cmd_lookup(cmd_arr);
	} */
	UARTCharPut(UART5_BASE, '\r');
	UARTCharPut(UART5_BASE, '\n');
	delay();
}

void front_sensor_read(void){
	//char cmd_arr[2] = {'s','t'};
	float distance = 0;
	char uartOut[8];
	// clear ADC interrupt
	ADCIntClear(ADC0_BASE, 1);
	// trigger ADC sampling
	ADCProcessorTrigger(ADC0_BASE, 1);
	// read voltage
	ADCSequenceDataGet(ADC0_BASE, 1, &adcValf);
	// convert adc value into cm
	distance = 22700*pow(adcValf, -1.08476) + 0.8;
	// convert float to string
	ftoa(distance, uartOut, 4);
	// write output (UART)
	//UARTCharPut(UART5_BASE, '\n');
	uart_print("Front Distance: ", 17);
	uart_print(uartOut, 8);
	/*
	if(distance < 4){
		uart_print("Front Too close", 14+1);
		//cmd_lookup(cmd_arr);
	}*/
	UARTCharPut(UART5_BASE, '\r');
	UARTCharPut(UART5_BASE, '\n');
	delay();
}

/// ADC ///

void pwm_start_motor(void){
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
}

/// Delay ///

void delay(void)
{
	SysCtlDelay(6700000); // ~500ms delay
}

/// Float to String conversion ///

// conversion taken from https://www.geeksforgeeks.org/convert-floating-point-number-string/
// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

///////////////  Hardware Initialization ///////////////
void hw_init(void)
{
    // set system clock (will be 40 MHz)
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    uart_init();
    adc_init();
    //pwm_init();

}

void uart_init(void){
    // enable Port for GPIO and UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    // configure receiver (Rx)
    GPIOPinConfigure(GPIO_PE4_URX);
    // configure PA1 as transmitter (Tx)
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // configure PB0 and PB1 for input
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // configure UART, 9600 8-n-1
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    // enable UART5
    UARTEnable(UART5_BASE);
    // enable interrupts on processor
    IntMasterEnable();
    // enable interrupts on UART5
    IntEnable(INT_UART5);
    // enable interrupts for UART5, Rx and Tx
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
}

void adc_init(void)
{
	// Enable ADC0 module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
	// // configure PE2 for input
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	// configure PE3 for input
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	// Configure sample sequencer (PE2)
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);
	// Configure sample sequencer (PE3)
	//ADCSequenceDisable(ADC0_BASE, 3);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
}

void pwm_init(void){
	// set PWM clock
	volatile uint32_t PWMclk = SysCtlClockGet() / 64;
	volatile uint32_t PWMload = (PWMclk / PWMFREQ) - 1;
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	//Enable PWM1 and Port D for GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralReset(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
	// Set PD2 for input
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	// Set PD3 for input
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
	//Configure PD0 - connection with PD2
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	//Configure PD1 - connection with PD3
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
	// set counter for PWM1
	// Configure Generator 0, counting down
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWMload);
	// Configure Generator 0, counting down
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMload);
	// Enable Generator 1
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 7000);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	// Enable Generator 2
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 7000);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	pwm_start_motor();
}
