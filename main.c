/*
 * 168_UART_SIM900.c
 *
 * Created: 30.01.2017 13:35:47
 * Author : Riks
 */ 


#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

//#define BAUD 9600UL							// ñêîðîñòü ïåðåäà÷è äàííûõ ïî UART
//#define LINE_SIZE 16U						// äëèíà ñòðîêè äèñïëåÿ 
//#define DYSPLAY_SIZE 2U						// êîëè÷åñòâî ñòðîê íà ýêðàíå 
#define BUF_SIZE 128						// ðàçìåð áóôåðà ïåðåäàò÷èêà
#define BUF_MASK (BUF_SIZE-1)
#define IN_BUF_SIZE 64					// ðàçìåð áóôåðà ïðèåìíèêà
#define IN_BUF_MASK (IN_BUF_SIZE-1)

#define CR 0x0D
#define SUB 0x0A

#define DELL _delay_ms (500)

#define SEND (UCSR0B |= (1<<UDRIE0)) // ðàçðåøàåì ïðåðûâàíèå ïî îïóñòîøåíèþ ðåãèñòðà ïåðåäàò÷èêà
#define RESEN (UCSR0B |= (1<<RXEN0)) // RX Enable


#define NUM0 "\"+380xxxxxxxxx\"\r"
#define REG_MESS "Hello. Button 1\r"
#define STR_MESS "Device ready to work\r"





#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTB0
#define EN eS_PORTB1

// === îáúÿâëåíèå ãëîáàëüíûõ ïåðåìåííûõ è ôóíêöèé ===
// ==================================================
//#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)    // ìàêðîñ ðàñ÷åòà ñêîðîñòè ïåðåäà÷è äëÿ UBRR
//#define A_SIZE(a)  (sizeof(a)/sizeof(*(a)))	// ìàêðîñ ðàñ÷åòà ÷èñëà ýëåìåíòîâ ìàññèâà



#include <util/delay.h>

#include <avr/interrupt.h>					// ïîäêëþ÷àåì áèáëèîòåêó äëÿ ðàáîòû ñ ïðåðûâàíèÿìè
#include <util/atomic.h>					// ïîäêëþ÷àåì áèáëèîòåêó äëÿ àòîìàðíûõ îïåðàöèé
#include <string.h>
#include "lcd.h"
#include <avr/io.h>
#include "stdint-gcc.h"


volatile char buffer[BUF_SIZE]="";
volatile char inbuf[IN_BUF_SIZE]="\0";			//inner buffer of USART
volatile uint8_t com_detect = 0;					//idex commands
volatile uint8_t ind_in=0, ind_out=0, rxind_out=0, rxind_in=0, mess = 0;

#define TIMEOUT 100

// èíèöèàëèçàöèÿ UART
void uartInit (void)
{
	//UBRR0H = (unsigned char)(baudrate>>8);	// ñäâèãàåì ÷èñëî âïðàâî íà 8 áèò
	//UBRR0L = (unsigned char)baudrate;		// óñòàíàâëèâàåì ñêîðîñòü ïåðåäà÷è
	
	#define baudrate 9600L
	#define bauddivider (F_CPU/(16*baudrate)-1)
	#define HI(x) ((x)>>8)
	#define LO(x) ((x)& 0xFF)
	
	UBRR0L = LO(bauddivider);
	UBRR0H = HI(bauddivider);
	UCSR0A = 0;
	UCSR0B|= (1<<RXCIE0)|(0<<TXCIE0);				// ðàçðåøàåì ïðåðûâàíèå ïî ïðèåìó
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);     // âêëþ÷àåì ïðèåìíèê è ïåðåäàò÷èê
	UCSR0C|= (0<<UPM00)|(3<<UCSZ00);    //  ôîðìàò äàííûõ 8 áèò
	UCSR0A &= ~(1<< UDRE0);
	sei ();
}


void SendByte(char byte)
{
		//while(!(UCSR0A & (1<<UDRE0))); //Stop interrupt for correct position
		//UDR0 = byte;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
	buffer[ind_in++] = byte; //èíêðåìåíò «ãîëîâû» 
	ind_in &= BUF_MASK;  // çàöèêëèâàíèå «ãîëîâû» (îðãàíèçàöèÿ êîëüöà)
	}
}

void SendStr(char *string)
{
	while (*string !='\r')  //check if End
	{
		SendByte(*string);
		string++;
	}
}

 void Ready_Snd (void)
 {
	 Lcd4_Clear();
	 Lcd4_Set_Cursor(0,1);
	 Lcd4_Write_String ("Ready");
 }
 
 void ErrMes (void)
 {
	 Lcd4_Clear();
	 Lcd4_Set_Cursor(0,1);
	 Lcd4_Write_String("Error");
 }
 
 void code_com (uint8_t count)
 {
	 switch (com_detect)
	 {
		 case (0x12): if (count == 4) com_detect = 2; break; //R^I^N^G
		 case (0x58): if (count == 5) com_detect = 3; break; //ERROR
		 case (0x04): if (count == 2) com_detect = 1; break; //OK
		 case (0x49): if (count == 10) com_detect = 4; break; //Call Ready
		 default: com_detect = 0;
	 }
 }
 
void rx_check_in (void)
{
	char *tmp = 0;
	int count = 0;
	int i = 0;
	com_detect = 0;  //zero command scaner
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,0);
	Lcd4_Write_String ("Mess in BUF:");
	Lcd4_Set_Cursor(1,13);
	Lcd4_Write_Char(mess+0x30);   //how many mess in buf (ASCII)
	
	while (1)
	{
			/*
			
			if(inbuf[rxind_out] == '+' ) 
			{
				while(inbuf[rxind_out] != '$')
				{
					rxind_out++;
					rxind_out &= IN_BUF_MASK;
				}
				
				Lcd4_Set_Cursor(2,10);
				Lcd4_Write_String ("S +");
			}
			if(inbuf[rxind_out] == '0' )
			{
				while(inbuf[rxind_out] != '$')
				{
					rxind_out++;
					rxind_out &= IN_BUF_MASK;
				}
				com_detect = 0x04;
				count = 2;
				code_com (count);
				count = 0;
				break;
			}
			if(inbuf[rxind_out] == '"')
			{
				while(inbuf[rxind_out] != '$')
				{
					rxind_out++;
					rxind_out &= IN_BUF_MASK;
				}
				
				Lcd4_Set_Cursor(2,10);
				Lcd4_Write_String ("S \"");
			}
			
			*/
			
		if (inbuf[rxind_out] != '\0') //if mess separator detected
		{
		
			*tmp = inbuf[rxind_out];
			ATOMIC_BLOCK(ATOMIC_FORCEON){
			com_detect ^= inbuf[rxind_out++];
			rxind_out &= IN_BUF_MASK;
			}
			Lcd4_Set_Cursor(2,i++);
			Lcd4_Write_Char (*tmp);
			count++;
			
		}
		else
		{
			ATOMIC_BLOCK(ATOMIC_FORCEON){
			rxind_out++;
			rxind_out &= IN_BUF_MASK;
			}
			code_com (count);
			break;
		}
	}
	if (com_detect != 0)
	{
		//Lcd4_Clear();
		Lcd4_Set_Cursor(2,12);
		Lcd4_Write_Char(com_detect + 0x30);
	}
}

int send_sms (int fun, char *number)
{
	int clock = 0;
	UCSR0B &= ~(1<<RXEN0);   //disable recieve
	SendStr("AT+CMGF=1\r");
	SendByte(CR);
	SEND;
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,0);
	Lcd4_Write_String("Sending to:");
	while (com_detect != 1)
	{
		if (++clock == TIMEOUT)	return 0;
		if (mess != 0) //if we have mess in buffer
		{
			// code
			mess--;   //minus one
			rx_check_in ();
		}
		DELL;
	}
	com_detect = 0;
	clock = 0;
	Lcd4_Set_Cursor(2,0);
	SendStr("AT+CMGS=\r");
	UCSR0B &= ~(1<<RXEN0);

	while (*number != '\r')
	{
		SendByte (*number);
		Lcd4_Write_Char (*number);
		number++;
	}
	SendByte(CR);  //send ENTER
	SEND;
	DELL;
	UCSR0B &= ~(1<<RXEN0);
	if (fun == 1) SendStr(STR_MESS);
	else SendStr(REG_MESS);
	SEND;
	_delay_ms(100);
	UCSR0B &= ~(1<<RXEN0);
	SendByte(0x1A);
	SEND;
	while (com_detect != 1)
	{
		if ((++clock == TIMEOUT) || (com_detect == 3))	return 0;
		if (mess != 0) //if we have mess in buffer
		{
			// code
			mess--;   //minus one
			rx_check_in ();
		}
		DELL;
	}
	SendByte(0x1B);
	UCSR0B &= ~(1<<RXEN0);
	SEND;
	Lcd4_Clear();
	Lcd4_Set_Cursor(2,0);
	Lcd4_Write_String ("Sended");
	_delay_ms(500);
	Ready_Snd();
	return 1;
}

//Sending data from buffer
ISR (USART_UDRE_vect)
{
	UDR0 = buffer[ind_out++];   //çàïèñü èç áóôåðà
	ind_out &= BUF_MASK;      //ïðîâåðêà ìàñêè êîëüöåâîãî áóôåðà
	if (ind_in == ind_out)  //if last byte //åñëè áóôåð óæå ïóñò
	{
		UCSR0B &= ~(1<<UDRIE0); //disable interrupt UDR empty
		RESEN;
	}
	sei ();
}


//recieving Data 
ISR (USART_RX_vect)
{
	uint8_t tmp;
	tmp = UDR0;
	if (tmp == 0x0d)
	{
		
		inbuf[rxind_in++] = '\0'; //set separate between mess
		rxind_in &= IN_BUF_MASK;
		mess++;     //one more message
		
		
	} else  {
				if (tmp == 0x00) {
					
				} else {
				if (tmp != 0x0a)					//clear bad simble
				{
				inbuf[rxind_in++] = tmp;
				rxind_in &= IN_BUF_MASK;
				}
				}
			}
	sei ();
}

ISR(USART_TX_vect)      // âåêòîð ïðåðûâàíèÿ UART - çàâåðøåíèå ïåðåäà÷è
{
	sei ();
}

 
int main(void)
{
	
// === èíèöèàëèçàöèÿ ===
// =====================

				
							
// èíèöèàëèçàöèÿ äèñïëåÿ
	
	DDRD = 0xFF;
	DDRB = 0xFF;
	Lcd4_Init();
	
	Lcd4_Clear();
	Lcd4_Set_Cursor(1,1);
	Lcd4_Write_String("Hello My Lord!");
	_delay_ms(2000);
	Lcd4_Clear();
	
// èíèöèàëèçàöèÿ UART
	uartInit();
	
	
		 // Ready_Snd ();
	Lcd4_Write_String("Wait");
	
	 	
	  while (1)
	  {
		  if (mess != 0) //if we have mess in buffer
		  {
			  // code
			  mess--;   //minus one
			  rx_check_in ();
			  if (com_detect == 2)
			  {
				  SendStr("ATH0\r");
				  SendByte(CR);
				  DELL;
				  if (!send_sms (1,NUM0)) ErrMes ();
				   //Ready_Snd ();
			  } else if (com_detect == 4)
			   {
				   if (!send_sms (1,NUM0)) ErrMes ();
				  // Ready_Snd ();
			   }
			  // com_detect = 0;
			   //Ready_Snd ();
		  }
		  //tmp = PINC & 1;  //If push button:
		  //if (tmp != 1)
		  //{
			  //send_sms(0,NUM0);
			  //Ready_Snd ();
		  //}
	  }
	  //return 0;
	
   }

