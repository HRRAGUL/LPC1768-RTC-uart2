#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <RTL.h>
#include <Net_Config.h>
#include <LPC17xx.h>                    /* LPC17xx definitions               */

#include <core_cm3.h>
//#define  FOSC    12000000
//#define  FCCLK   (FOSC*8)
//#define  FPCLK   (FCCLK/4)
#include "User.h"
#include "HTTP.h"
#include "Client.h"
#include "Server.h"
#include "Announce.h"
#include "I2C_EEPROM.h"
#include "USER_EEPROM.h"
#include "HTTP_CGI.h"
#include "Iiot.h"
#include "RTC.h"
#include "Modbus_Server.h"
#include "UART.h"
#include "ADC.h"
#include "GPIO.h"
#include "SPI.h"
#include "TIMER.h" 
#include "SSP.h"
#include "TDSXGL.h"	
#include "TDSXGA.h"
uint8_t GC_RecieveBuff[GK_RECEIVE_LENGTH]={0},GC_RecieveBuff1[GK_RECEIVE_LENGTH]={0},GC_RX_Flag=0,GC_RX_Flag1=0;
//static uint8_t  GC_ArrayPutPtr=0;//GC_ArrayPutPtr1=0;
extern unsigned char GC_IECEnableFlag_GA,GC_IECEnableFlag_GL,GC_Ticker30Sec_Flag;
extern uint64_t GL_Ticker30Sec;
uint8_t G_TxBuff[40]={0}; 
uint8_t G_TxBuff1[40]={0}; 
uint16_t AXE410_ModRtuCRC(uint8_t *,uint16_t);
static uint32_t GLI_Pclk,GLI_Fdiv;
 uint8_t Byte=0;
char a[]="\n\rApplied Embedded";
uint16_t sec,min,hou,day,mon,year;
unsigned char str3[150];
char on1=1,off1=1, on2=1,off2=1;

	void AXE410_RTC(void)
{
  //Enable CLOCK into RTC 
  LPC_SC->PCONP |= (1 << 9);
  /*--- Initialize registers ---*/    
  LPC_RTC->CCR = 0x00000000;//Clock Control Register,stop RTC
//  LPC_RTC->AMR  = 0x00000000;//Alarm Mask Register 
  LPC_RTC->CIIR = 0x00000000;//Counter Increment Interrupt Register
  LPC_RTC->CCR =0x00000001 ; // enable counter
	
}
void  AXE410_RTC_read(void)
{
	sec=LPC_RTC->SEC;	// Second value
	min=LPC_RTC->MIN;	// Second value
	hou=LPC_RTC->HOUR;	// Hour value
	day=LPC_RTC->DOM;   // day of month
	mon=LPC_RTC->MONTH;	// Month value
	year=LPC_RTC->YEAR;	// Year value

}


void AXE410_UART2Init(void)
{
	
		LPC_SC->PCONP 		|= 1<<24;	// Set UART1 Power Control Bit
	  LPC_SC->PCLKSEL0 		|= (0x01<<16);	// Set Uart clock is Pclk/4
	//	LPC_PINCON->PINSEL1 &= ~(0xFF << 18); // Clear bits 18-25
   // LPC_PINCON->PINSEL1 |= (0x10 << 18); 
	  LPC_PINCON->PINSEL0 |= (0x00500000);	// Set P0.0 as a TXD3,P0.1 as RXD3
	  LPC_PINCON->PINMODE0|= (0x00<<11);	// P0.0,P0.1 pin pull up is enabled
		LPC_PINCON->PINMODE0|= (0x00<<10);
	  LPC_GPIO0->FIODIR 	|= (1<<10);	// Set P0.0 pin as output,P0.1 pin as input
		LPC_GPIO0->FIODIR 	|= (0<<11);
	//  LPC_GPIO0->FIOCLR    = (0xFFFFFFFF);	// Clear output states
		LPC_GPIO0->FIOCLR	= (0xFFFFFFFF);	// Clear output states
	// UART control registers
	//0  LPC_UART3->FCR = 0x07;	// Uart access enabled. Tx,Rx FIFO got reset.
	  LPC_UART2->LCR = 0x83;	// 8data bits,1stop bit,DLA enabled
	  GLI_Pclk = SystemCoreClock/4;	// SystemCoreClock division value depands on PCLKSEL register value
	  GLI_Fdiv = (GLI_Pclk/16)/9600;
	  LPC_UART2->DLM = GLI_Fdiv/256;
	  LPC_UART2->DLL = GLI_Fdiv%256;
	
	  LPC_UART2->LCR = 0x03;	// Disable DLA
	  LPC_UART2->THR = 0x00;
	  LPC_UART2->FCR = 0x07;
	  NVIC_EnableIRQ(UART2_IRQn);
	  NVIC_SetPriority(UART2_IRQn,1);
	  LPC_UART2->IER = 1;	// Set UART RX interrupt*/
		
}



void AXE410_Uart2Tx(uint8_t Value)
{
	while(!(LPC_UART2->LSR & 0x20));	// wait until TSR get empty
	LPC_UART2->THR = Value;	// Fill Transmit Holding Register with out data
}


//////////////////////////////////////////////////////////
void UART2_IRQHandler(void)
{ // uint32_t LLI_i;	
	while(LPC_UART2->LSR&0x01)	// Wait until data gets receive
	{
		Byte=LPC_UART2->RBR;
	 
		
}
}
int main(void)
{
	AXE410_UserIoInit();
	
	AXE410_UART2Init();

	AXE410_RTC();
	
	
  while (1) 
	{
		
	uint32_t LLI_i;	
	LPC_GPIO2->FIOSET = GK_USER_LED2_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
	LPC_GPIO2->FIOCLR = GK_USER_LED2_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
		
		if(LPC_GPIO4->FIOPIN&0X10000000)
	{
	
	if(on1==1)
	{
	AXE410_RTC_read();
	sprintf(str3,"SWITCH 1-ON-TIME : %d-%d-%d ,%d-%d-%d\n*",hou,min,sec,day,mon,year);
	for(int i=0;str3[i]!='*';i++)  //transmit a predefined string
   AXE410_Uart2Tx(str3[i]);
	on1=0;
	//G_TxBuff1[1]=1;
  }}
	else
	{
		if(on1==0)
	{
	AXE410_RTC_read();
	sprintf(str3,"SWITCH 1-OFF-TIME : %d-%d-%d ,%d-%d-%d\n*",hou,min,sec,day,mon,year);
	for(int i=0;str3[i]!='*';i++)  //transmit a predefined string
   AXE410_Uart2Tx(str3[i]);
	on1=1;//	G_TxBuff1[1]=0;
	}}
	
	
	if(LPC_GPIO4->FIOPIN&0X20000000)
	{
		if(on2==1)
	{
	AXE410_RTC_read();
	sprintf(str3,"SWITCH 2-ON-TIME : %d-%d-%d ,%d-%d-%d\n*",hou,min,sec,day,mon,year);
	for(int i=0;str3[i]!='*';i++)  //transmit a predefined string
   AXE410_Uart2Tx(str3[i]);
	on2=0;
	//G_TxBuff1[2]=1;
	}}
	else
	{
		
	if(on2==0)
	{	
	AXE410_RTC_read();
	sprintf(str3,"SWITCH 2-OFF-TIME : %d-%d-%d ,%d-%d-%d\n*",hou,min,sec,day,mon,year);
	for(int i=0;str3[i]!='*';i++)  //transmit a predefined string
   AXE410_Uart2Tx(str3[i]);
		on2=1;//G_TxBuff1[2]=0;
	}}
	
}
}
