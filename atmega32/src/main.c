#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "HD44780.h"

int count;
volatile int petla;

float value;
int l_speed, r_speed;


ISR(TIMER0_COMP_vect) {
	count++;
	if (count == 100) {
		petla = 1;
		count = 0;
	}
}

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

	// inicjalizacja timera 1 - opis bitow od str. 112 datasheeta
	// Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM, (non-inverting mode)
 	// Fast PWM, 8-bit
	// zegar bez skalowania, co daje czestotliwosc 16MHz/1/256=62.5kHz
	TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);
	TCCR1B=(1<WGM12)|(1<<CS10);
}

void sensors_value() {
	value = 0;

	if (PINC & 0x02) { // SENS_1
		value -= 6.5;
	}
	if (PINC & 0x04) { // SENS_2
		value -= 5.5;
	}
	if (PINC & 0x08) { // SENS_3
		value -= 4.5;
	}
	if (PINC & 0x10) { // SENS_4
		value -= 3.5;
	}
	if (PINC & 0x20) { // SENS_5
		value -= 2.5;
	}
	if (PINC & 0x40) { // SENS_6
		value -= 1.5;
	}
	if (PINC & 0x80) { // SENS_7
		value -= 0.5;
	}
	if (PINA & 0x80) { // SENS_8
		value += 0.5;
	}
	if (PINA & 0x40) { // SENS_9
		value += 1.5;
	}
	if (PINA & 0x20) { // SENS_10
		value += 2.5;
	}
	if (PINA & 0x10) { // SENS_11
		value += 3.5;
	}
	if (PINA & 0x08) { // SENS_12
		value += 4.5;
	}
	if (PINA & 0x04) { // SENS_13
		value += 5.5;
	}
	if (PINA & 0x02) { // SENS_14
		value += 6.5;
	}
}

void set_motors() {
		
	sensors_value();

	l_speed = 50;
	r_speed = 50;

	OCR1A = l_speed;
	OCR1B = r_speed;

	if (l_speed > 0) {
		PORTA |= 0x01;
		PORTB &= 0xFE;
	}
	else {
		PORTA &= 0xFE;
		PORTB |= 0x01;
	}
	if (r_speed > 0) {
		PORTB |= 0x02;
		PORTB &= 0xFB;
	}
	else {
		PORTB &= 0xFD;
		PORTB |= 0x04;
	}

}


int main(void) {
	init_ports();
	init_timers();
	LCD_Initalize();
	int i;
	char c[2];
	c[1] = '\0';
	for (i=3; i>=0; i--) {
		c[0] = (char)(i+'0');
		LCD_WriteText(c);
		_delay_ms(1000);
	}
	LCD_Home();
	LCD_WriteText("te");
	
	count = 0;
	petla = 0;

	OCR1A = 45;
	OCR1B = 45;

	LCD_Clear();
	LCD_WriteText("!!!");


	// wlaczenie obslugi przerwan
	sei();

	char tab[5] = "0000";
	int clk = 0;

	while (1) {
		if (petla) {
			// set_motors();

			LCD_Clear();
			LCD_WriteText(tab);
			clk++;
			tab[3] = (char)(clk%10+'0');
			tab[2] = (char)((clk/10)%10+'0');
			tab[1] = (char)((clk/100)%10+'0');
			tab[0] = (char)((clk/1000)%10+'0');

			petla = 0;
		}
	}

	return 0;
}
