#define F_CPU 8000000UL // definire frecventă
#include <avr/io.h>      // contine toti registrii I/O
#include <util/delay.h>  // generare delay
#include <avr/interrupt.h> // contine functii pentru sistemul de întreruperi
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#define LCD_Dir  DDRD		// definire directie pini ca output pentru pinii folositi pentru LCD
#define LCD_Port PORTD		
#define RS PD3			// definire pin LCD RS 
#define EN PD2			// definire pin LCD en 
int runtime;			
#define DATA_REGISTER_EMPTY_INTERRUPT (1<<UDRIE0) // definire registru intreruperi Register Empty Interrupt Enable

volatile uint8_t UART_ReceiveBuffer; // buffer UART global

void UART_Init()
{
	UBRR0H = 0b00000000;  // Setare baud rate
	UBRR0L = 0b00110011;  // Setare baud rate
	UCSR0C |= (0 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01); // Setare format cadru
	UCSR0B = (1<<RXEN0); // activare receptie
	sei(); 
}

ISR(USART_RX_vect) 
{
	UART_ReceiveBuffer = UDR0;  // functie pentru salvare date 
								// cand se primesc date se livreaza  intrerupere si datele sunt salvate in UDR0
}

void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); // trimitere primii 4 biti(de la MSB spre LSB)
	LCD_Port &= ~ (1<<RS); //RS=0
	LCD_Port |= (1<<EN);   //EN=1
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); //EN=0
	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  // trimitere ultimii 4 biti(de la MSB spre LSB)
	LCD_Port |= (1<<EN); //EN=1
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); //EN=0
	_delay_ms(2);
}

void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); // trimitere primii 4 biti(de la MSB spre LSB)
	LCD_Port |= (1<<RS);	//RS=1
	LCD_Port|= (1<<EN);		//EN=1
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); 	//EN=0
	_delay_us(200);
	LCD_Port = (LCD_Port & 0x0F) | (data << 4);  //trimitere ultimii 4 biti(de la MSB spre LSB)
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void LCD_Init (void) // functia de initializare a LCD-ului
{
	LCD_Dir = 0xFF;    // setare directie porturi pentru pinii aferenti LCD-ului ca output
	_delay_ms(20);     // delay la pornirea LCD-ului, minim 15ms
	LCD_Command(0x02); // comandă pentru mod functionare pe 4 biti
	LCD_Command(0x28); //comandă pentru mod de functionare pe 2 linii în format 5*8 biti           	
	LCD_Command(0x0c); // LCD stare ON, cursor stare OFF
	LCD_Command(0x06); //mutare cursor spre dreapta
	LCD_Command(0x01); // curătare display
	_delay_ms(2);
}

void LCD_String (char *str) //trimitere string pentru afitare
{
	int i;
	for(i=0;str[i]!=0;i++)	//se trimite fiecare caracter al string-ului iar după ultimul aracter se iese din buclă
	{
		LCD_Char (str[i]);
	}
}

void LCD_Clear()
{
	LCD_Command (0x01);	//comanda pentru curătare display
	_delay_ms(2);
	LCD_Command (0x80);	// comandă pentru readucere cursor in pozitie initiala
}


void adc_init(void)
{
	ADCSRA = 0x00; //dezactivare ADC
	ADMUX = 0x00; // dezactivare Aref
	ACSR  = 0x80; // dezactivare comparator analogic
	ADCSRA = 0x86; // pornire ADC, cu valoare divizare prescaler 64
}

unsigned int ReadAdc(unsigned char Ch)
{
	ADMUX = 0xC0 | Ch; //selectare tensiune de referintă internă cu valoarea de 1.1V
	ADCSRA |= 0x40; // activare ADC ti pornire conversie
	while (!(ADCSRA & 0x10));  // asteptare până la finalul conversiei
	ADCSRA |= 0x10;  // marcare final conversie
	return ADC; // returnare valoare
}


int main(void)
{	
	UART_Init(); // initializare UART
	UCSR0B |= (1<<RXCIE0); // configurare registru UCSR0B

	DDRB = DDRB | ( 1<<3) ; //setare PB3 ca output pentru releu 1
	DDRB = DDRB | ( 1<<4) ; //setare PB4 ca output pentru releu 2
	DDRB = DDRB | ( 1<<5) ; //setare PB5 ca output pentru releu 3
	
	DDRB = DDRB & (~(1<<0));	//input sw0
	PORTB = PORTB | (1<<0);   // activare internal pull up res
	DDRB = DDRB & (~(1<<1));	//input sw1
	PORTB = PORTB | (1<<1);   // activare internal pull up res
	DDRB = DDRB & (~(1<<2));	//input sw2
	PORTB = PORTB | (1<<2);
	
	adc_init();  // initializare ADC
	LCD_Init();	 //initializare LCD		

	while (1)
	
	{LCD_String(" Mod AUTO"); // scriere string pe linia 1
		LCD_Command(0xC0);		 // trecere la linia 2
		LCD_String("        Activat"); // scriere string pe linia 2
		_delay_ms(75);
		uint8_t sw1=ReadAdc(3)/1023; // citire stare switch 1 pe ADC
		uint8_t sw2=ReadAdc(4)/1023; // citire stare switch 2 pe ADC
		uint8_t sw3=ReadAdc(5)/1023; // citire stare switch 3 pe ADC
		
		if (sw1==1) // daca switch ==1
		{
			
			LCD_String(" Mod AUTO"); // scriere string pe linia 1
			LCD_Command(0xC0);		 // trecere la linia 2 
			LCD_String("        Activat"); // scriere string pe linia 2
			_delay_ms(75);
			LCD_Clear(); // curatare lcd
			if (ReadAdc(0)>600) //daca valoare senzor >600
			{
				PORTB = PORTB | (1<<3); 
				LCD_String(" IRIGARE ZONA 1"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("       Activata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
				
			}
			else
			{
				PORTB = PORTB & (~(1<<3));
				LCD_String(" IRIGARE ZONA 1"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("    Dezactivata"); // scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
				
			}
			
			
			if (ReadAdc(1)>600) //senzorul 2 //daca valoare senzor >600
			{
				PORTB = PORTB | (1<<4); 
				LCD_String(" IRIGARE ZONA 2"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("       Activata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear();// curatare lcd
			}
			else
			{
				PORTB = PORTB & (~(1<<4));
				LCD_String(" IRIGARE ZONA 2"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("    Dezactivata"); // scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			
			
			if (ReadAdc(2)>600) //senzorul 3 //daca valoare senzor >600
			{
				PORTB = PORTB | (1<<5); 
				LCD_String(" IRIGARE ZONA 3"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("       Activata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			else
			{
				PORTB = PORTB & (~(1<<5));//  
				LCD_String(" IRIGARE ZONA 3"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("    Dezactivata"); // scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			
		}
		if (sw2==1)
		{
			LCD_String(" Mod MANUAL"); // scriere string pe linia 1
			LCD_Command(0xC0);		// trecere la linia 2 
			LCD_String("        Activat");	// scriere string pe linia 2
			_delay_ms(75);
			LCD_Clear(); // curatare lcd
			if((PINB & 0b00000001)==0)  //swich 1
			{PORTB &= ~(1<<PB3);
				
				LCD_String(" IRIGARE ZONA 1"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("       Activata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
				
			} 
			else{
				
				PORTB |= (1<<PB3);
				
				LCD_String(" IRIGARE ZONA 1"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("    Dezactivata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			if((PINB & 0b00000010)==0) //swich 2 
			{
				PORTB &= ~(1<<PB4); 
				LCD_String(" IRIGARE ZONA 2"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("       Activata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			else{
				
				PORTB |= (1<<PB4); 
				LCD_String(" IRIGARE ZONA 2"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("    Dezactivata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			if((PINB & 0b00000100)==0)//sw3
			{
				PORTB &= ~(1<<PB5); 
				LCD_String(" IRIGARE ZONA 3"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("       Activata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			else{
				
				PORTB |= (1<<PB5);
				LCD_String(" IRIGARE ZONA 3"); // scriere string pe linia 1
				LCD_Command(0xC0);		// trecere la linia 2 
				LCD_String("    Dezactivata");	// scriere string pe linia 2
				_delay_ms(75);
				LCD_Clear(); // curatare lcd
			}
			
		}
if (sw3==1) 
{
	LCD_String(" Mod BLUETOOTH"); // scriere string pe linia 1
	LCD_Command(0xC0);		// trecere la linia 2 
	LCD_String("        Activat");	// scriere string pe linia 2
	
	
	if (UART_ReceiveBuffer ==('Pornire irigatie') // atunci cand se trimite comanda
		PORTB |= (1<<PB3); // releu 1 ON
		PORTB |= (1<<PB4); // releu 2 ON
		PORTB |= (1<<PB5); // releu 3 ON
		
		_delay_ms(18000000);  // oprire după 30 minute
		PORTB &= ~(1<<PB3); // releu 1 OFF
		PORTB &= ~(1<<PB4); // releu 2 OFF
		PORTB &= ~(1<<PB5); // releu 3 OFF
	}
	else
	{
		
			}
		} 
	}
}
