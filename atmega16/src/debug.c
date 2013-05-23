#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "HD44780.h"

volatile int count;
volatile int petla;

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

// procedura wypisujaca na ekran stan kazdego z czujnikow
// oraz predkosc silnikow
// ekran nalezy podlaczyc przed uruchomieniem robota!!!
void print_sensors() {
	LCD_Clear();

	if (PINA&(1<<PA1)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINA&(1<<PA2)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINA&(1<<PA3)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINA&(1<<PA4)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINA&(1<<PA5)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINA&(1<<PA6)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINA&(1<<PA7)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC7)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC6)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC5)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC4)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC3)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC2)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}
	if (PINC&(1<<PC1)) {
		LCD_WriteText("1");
	}
	else {
		LCD_WriteText("0");
	}

}

int main(void) {
	init_ports();
	init_timers();
	LCD_Initalize();

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
			
			petla = 0;
		}
	}

	return 0;
}
