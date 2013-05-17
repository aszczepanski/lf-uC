#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "HD44780.h"

#define kp 13
#define ki 0
#define kd 0
#define Tp 100

int count;
volatile int petla;

float value;
short active[14], on_track;
int l_speed, r_speed;

int i;

ISR(TIMER0_COMP_vect) {
	count++;
	if (count == 50) {
		petla = 1;
		count = 0;
	}
}

ISR(INT1_vect) {

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

void init_interrupts() {
	// TO DO
	//MCUCR |= 
	//
	GICR |= (1<<INT1);
}

void sensors_value() {
	value = 0;
	on_track = 0;

	if ((PINC & 0x02) && (active[0] || active[1])) { // SENS_1
		value -= 6.5;
		active[0] = 1;
		on_track = 1;
	}
	else {
		active[0] = 0;
	}
	if ((PINC & 0x04) && (active[0] || active[1] || active[2])) { // SENS_2
		value -= 5.5;
		active[1] = 1;
		on_track = 1;
	}
	else {
		active[1] = 0;
	}
	if ((PINC & 0x08) && (active[1] || active[2] || active[3])) { // SENS_3
		value -= 4.5;
		active[2] = 1;
		on_track = 1;
	}
	else {
		active[2] = 0;
	}
	if ((PINC & 0x10) && (active[2] || active[3] || active[4])) { // SENS_4
		value -= 3.5;
		active[3] = 1;
		on_track = 1;
	}
	else {
		active[3] = 0;
	}
	if ((PINC & 0x20) && (active[3] || active[4] || active[5])) { // SENS_5
		value -= 2.5;
		active[4] = 1;
		on_track = 1;
	}
	else {
		active[4] = 0;
	}
	if ((PINC & 0x40) && (active[4] || active[5] || active[6])) { // SENS_6
		value -= 1.5;
		active[5] = 1;
		on_track = 1;
	}
	else {
		active[5] = 0;
	}
	if ((PINC & 0x80) && (active[5] || active[6] || active[7])) { // SENS_7
		value -= 0.5;
		active[6] = 1;
		on_track = 1;
	}
	else {
		active[6] = 0;
	}
	if ((PINA & 0x80) && (active[6] || active[7] || active[8])) { // SENS_8
		value += 0.5;
		active[7] = 1;
		on_track = 1;
	}
	else {
		active[7] = 0;
	}
	if ((PINA & 0x40) && (active[7] || active[8] || active[9])) { // SENS_9
		value += 1.5;
		active[8] = 1;
		on_track = 1;
	}
	else {
		active[8] = 0;
	}
	if ((PINA & 0x20) && (active[8] || active[9] || active[10])) { // SENS_10
		value += 2.5;
		active[9] = 1;
		on_track = 1;
	}
	else {
		active[9] = 0;
	}
	if ((PINA & 0x10) && (active[9] || active[10] || active[11])) { // SENS_11
		value += 3.5;
		active[10] = 1;
		on_track = 1;
	}
	else {
		active[10] = 0;
	}
	if ((PINA & 0x08) && (active[10] || active[11] || active[12])) { // SENS_12
		value += 4.5;
		active[11] = 1;
		on_track = 1;
	}
	else {
		active[11] = 0;
	}
	if ((PINA & 0x04) && (active[11] || active[12] || active[13])) { // SENS_13
		value += 5.5;
		active[12] = 1;
		on_track = 1;
	}
	else {
		active[12] = 0;
	}
	if ((PINA & 0x02) && (active[12] || active[13])) { // SENS_14
		value += 6.5;
		active[13] = 1;
		on_track = 1;
	}
	else {
		active[13] = 0;
	}

	if (!on_track) {
		for (i=0; i<14; i++) {
			active[i] = 1;
		}
	}
}

void set_motors() {
		
	sensors_value();

	l_speed = 50;
	r_speed = 50;

	if (on_track) {
		

	}	

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

void print_sensors() {
	LCD_Clear();
	LCD_WriteText("Line Follower");
	
	LCD_GoTo(0,1);

	if (PINC&(1<<PC1)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINC&(1<<PC2)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINC&(1<<PC3)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINC&(1<<PC4)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINC&(1<<PC5)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINC&(1<<PC6)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINC&(1<<PC7)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA7)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA6)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA5)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA4)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA3)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA2)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	if (PINA&(1<<PA1)) {
		LCD_WriteText("0");
	}
	else {
		LCD_WriteText("1");
	}
	LCD_WriteText(" =");


}

int main(void) {
	init_ports();
	init_timers();
	init_interrupts();
	LCD_Initalize();
	
	count = 0;
	petla = 0;

	PORTB |= (1<<PB0);
	
	PORTB |= (1<<PB2);

	OCR1A = 0;
	OCR1B = 0;

	LCD_Clear();
	LCD_WriteText("Line Follower");

	// wlaczenie obslugi przerwan
	sei();

	char tab[5] = "0000";
	int clk = 0;

	while (1) {
		if (petla) {
			// set_motors();

			print_sensors();

/*
			LCD_Clear();
			LCD_WriteText(tab);
			clk++;
			tab[3] = (char)(clk%10+'0');
			tab[2] = (char)((clk/10)%10+'0');
			tab[1] = (char)((clk/100)%10+'0');
			tab[0] = (char)((clk/1000)%10+'0');
*/
			petla = 0;
		}
	}

	return 0;
}
