#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "HD44780.h"

volatile int count;
volatile int petla;

#define thr 80

// obsluga przerwania timera0
// zeby co okreslony czas (100x/s) sprawdzac stan czujnikow i ustawiac predkosc silnikow
ISR(TIMER0_COMP_vect) {
	count++;
	if (count == 50) {
		petla = 1;
		count = 0;
	}
}

// obsluga przerwania INT1
// pochodzi z atmegi8, sluzy wlaczeniu/wylaczeniu robota

void init_ports() {
	// A0 - silniki
	// A1-A7 - SENS_14-SENS_8
	DDRA = 0x01;
	PORTA = 0x00;

	// B5-B7 - ISP
	// B3-B4 - LCD
	// B0-B2 - silniki
	DDRB = 0xFF;
	PORTB = 0x00;

	// C0 - LCD
	// C1-C7 - SENS_1-SENS_7
	DDRC = 0x01;
	PORTC = 0x00;

	// D6-D7 - LCD
	// D4-D5 - PWM
	// D3 - TSOP
	// D0-D2 - LCD
	DDRD = 0xF1;
	PORTD = 0x0E;
}

void init_timers() {
	// inicjalizacja timera 0 - opis bitow od str. 84 datasheeta
	// ponizej to co jest wlaczone
	// mode 2 - CTC (Clear Timer on Compare match)
	// normal port operacion, OC2 disconnected
	// zegar - CLK/1024 - taktowanie zegara 16MHz/1024 = 15625Hz
	TCCR0=(1<<WGM01)|(1<<CS02)|(1<<CS00);
	// wartosc przy ktorej zegar sie zeruje i generowane jest przerwanie
	OCR0 = 156; // dla 156 czestotliwosc przerwan to ok. 100Hz
	// odmaskowanie przerwania pochodzacego z timera 0
	TIMSK |= (1<<OCIE0);
}

void init_adc() {
	ADCSRA = (1<<ADEN) //ADEN=1 włączenie przetwornika ADC)
		|(1<<ADPS0) // ustawienie preskalera na 128  
		|(1<<ADPS1)  
		|(1<<ADPS2);

	ADMUX = (1<<REFS0)
		|(1<<ADLAR);
}

int sensors[14];

char buf[32];
int i;
float dif;
int on_track;

// procedura wypisujaca na ekran stan kazdego z czujnikow
// oraz predkosc silnikow
// ekran nalezy podlaczyc przed uruchomieniem robota!!!
void print_sensors() {
	LCD_Clear();

	if (PINA&(1<<PA1)) {
		sensors[0] = 1;
	}
	else {
		sensors[0] = 0;
	}
	if (PINA&(1<<PA2)) {
		sensors[1] = 1;
	}
	else {
		sensors[1] = 0;
	}
/*	if (PINA&(1<<PA3)) {
//		LCD_WriteText("1");
		sensors[2] = 1;
	}
	else {
//		LCD_WriteText("0");
		sensors[2] = 0;
	}
	if (PINA&(1<<PA4)) {
//		LCD_WriteText("1");
		sensors[3] = 1;
	}
	else {
//		LCD_WriteText("0");
		sensors[3] = 0;
	}
	if (PINA&(1<<PA5)) {
//		LCD_WriteText("1");
		sensors[4] = 1;
	}
	else {
//		LCD_WriteText("0");
		sensors[4] = 0;
	}
	if (PINA&(1<<PA6)) {
//		LCD_WriteText("1");
		sensors[5] = 1;
	}
	else {
//		LCD_WriteText("0");
		sensors[5] = 0;
	}*/
	if (PINA&(1<<PA7)) {
		sensors[6] = 1;
	}
	else {
		sensors[6] = 0;
	}
	if (PINC&(1<<PC7)) {
		sensors[7] = 1;
	}
	else {
		sensors[7] = 0;
	}
	if (PINC&(1<<PC6)) {
		sensors[8] = 1;
	}
	else {
		sensors[8] = 0;
	}
	if (PINC&(1<<PC5)) {
		sensors[9] = 1;
	}
	else {
		sensors[9] = 0;
	}
	if (PINC&(1<<PC4)) {
		sensors[10] = 1;
	}
	else {
		sensors[10] = 0;
	}
	if (PINC&(1<<PC3)) {
		sensors[11] = 1;
	}
	else {
		sensors[11] = 0;
	}
	if (PINC&(1<<PC2)) {
		sensors[12] = 1;
	}
	else {
		sensors[12] = 0;
	}
	if (PINC&(1<<PC1)) {
		sensors[13] = 1;
	}
	else {
		sensors[13] = 0;
	}

	for (i=3; i<=6; i++) {
		ADMUX &= 0b11100000;            // zerujemy bity MUX odpowiedzialne za wybór kanału (s. 255 w DS) 
		ADMUX |= i;                      // wybieramy kanał przetwornika 
		ADCSRA |= (1<<ADSC);            // uruchamiamy pomiar 
		while(ADCSRA & (1<<ADSC)) {};

		sensors[i-1] = (ADCH>thr) ? (1) : (0);
	}

	for (i=0; i<14; i++) {
		if (sensors[i]) LCD_WriteText("1");
		else LCD_WriteText("0");
	}
	
	dif=0;
	on_track=0;
	for (i=0; i<14; i++) {
                if (sensors[i]) {
                        dif += (float)i - 6.5;
                        on_track++;
                }
        }
	
	if (on_track) dif /= (float)on_track;

	LCD_GoTo(0,1);
	itoa(10*dif, buf, 10);
	LCD_WriteText(buf);
}

int main(void) {
	init_ports();
	init_timers();
	LCD_Initalize();

	init_adc();

	count = 0;
	petla = 0;

	// lewe do przodu
//	PORTB |= (1<<PB0);
//	PORTA &= ~(1<<PA0);

	// lewe do tylu
//	PORTA |= (1<<PA0);
//	PORTB &= ~(1<<PB0);

	// prawe do przodu
//	PORTB |= (1<<PB2);
//	PORTB &= ~(1<<PB1);

	// prawe do tylu
//	PORTB |= (1<<PB1);
//	PORTB &= ~(1<<PB2);

	LCD_Clear();
	LCD_WriteText("Line Follower");

	// wlaczenie obslugi przerwan
	sei();

	while (1) {
		if (petla) {
			print_sensors();

			LCD_GoTo(0,1);
/*
			for (i=3; i<=6; i++) {
			
				ADMUX &= 0b11100000;            // zerujemy bity MUX odpowiedzialne za wybór kanału (s. 255 w DS) 
				ADMUX |= i;                      // wybieramy kanał przetwornika 
				ADCSRA |= (1<<ADSC);            // uruchamiamy pomiar 
				while(ADCSRA & (1<<ADSC)) {};

				itoa(ADCH, buf, 10);

				LCD_WriteText(buf);
				LCD_WriteText(" ");
			}
*/
			petla = 0;
		}
	}

	return 0;
}
