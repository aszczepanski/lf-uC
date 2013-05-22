#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "HD44780.h"

// zmienne wykorzystywane w algorytmie PID
#define kp 7.0
#define ki 0.0
#define kd 0.0
#define Tp 48.0
float P, I, D, dif, pdif, rate, turn;

volatile int count;
volatile int petla;

short active[14], on_track;
int l_speed, r_speed;

int i;
char tab[33];

volatile short tsop;

// obsluga przerwania timera0
// zeby co okreslony czas (100x/s) sprawdzac stan czujnikow i ustawiac predkosc silnikow
ISR(TIMER0_COMP_vect) {
//	count++;
//	if (count == 50) {
		petla = 1;
//		count = 0;
//	}
}

// obsluga przerwania INT1
// pochodzi z atmegi8, sluzy wlaczeniu/wylaczeniu robota
ISR(INT1_vect) {
	tsop ^= 1;
//	tsop = 0;
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
	// inicjalizacja przerwan zewnetrznych - od str. 67 datasheeta
	// MCU Control Register
	// The rising edge of INT1 generates an interrupt request
	MCUCR |= ((1<<ISC11)|(1<<ISC10)); // mozna zmienic na cos innego, zobaczymy
	// When the INT1 bit is set (one) and the I-bit in the Status Register (SREG) is set (one), the external pin interrupt is enabled 
	GICR |= (1<<INT1);
}

void set_motors() {
	
	pdif = dif;

	dif = 0;
	on_track = 0;

	// odczyt stanu sensorow

	if ((PINA&(1<<PA1)) && (active[0] || active[1])) { // SENS_1
		dif -= 6.5;
		active[0] = 1;
		on_track++;
	}
	else {
		active[0] = 0;
	}
	if ((PINA&(1<<PA2)) && (active[0] || active[1] || active[2])) { // SENS_2
		dif -= 5.5;
		active[1] = 1;
		on_track++;
	}
	else {
		active[1] = 0;
	}
	if ((PINA&(1<<PA3)) && (active[1] || active[2] || active[3])) { // SENS_3
		dif -= 4.5;
		active[2] = 1;
		on_track++;
	}
	else {
		active[2] = 0;
	}
	if ((PINA&(1<<PA4)) && (active[2] || active[3] || active[4])) { // SENS_4
		dif -= 3.5;
		active[3] = 1;
		on_track++;
	}
	else {
		active[3] = 0;
	}
	if ((PINA&(1<<PA5)) && (active[3] || active[4] || active[5])) { // SENS_5
		dif -= 2.5;
		active[4] = 1;
		on_track++;
	}
	else {
		active[4] = 0;
	}
	if ((PINA&(1<<PA6)) && (active[4] || active[5] || active[6])) { // SENS_6
		dif -= 1.5;
		active[5] = 1;
		on_track++;
	}
	else {
		active[5] = 0;
	}
	if ((PINA&(1<<PA7)) && (active[5] || active[6] || active[7])) { // SENS_7
		dif -= 0.5;
		active[6] = 1;
		on_track++;
	}
	else {
		active[6] = 0;
	}
	if ((PINC&(1<<PC7)) && (active[6] || active[7] || active[8])) { // SENS_8
		dif += 0.5;
		active[7] = 1;
		on_track++;
	}
	else {
		active[7] = 0;
	}
	if ((PINC&(1<<PC6)) && (active[7] || active[8] || active[9])) { // SENS_9
		dif += 1.5;
		active[8] = 1;
		on_track++;
	}
	else {
		active[8] = 0;
	}
	if ((PINC&(1<<PC5)) && (active[8] || active[9] || active[10])) { // SENS_10
		dif += 2.5;
		active[9] = 1;
		on_track++;
	}
	else {
		active[9] = 0;
	}
	if ((PINC&(1<<PC4)) && (active[9] || active[10] || active[11])) { // SENS_11
		dif += 3.5;
		active[10] = 1;
		on_track++;
	}
	else {
		active[10] = 0;
	}
	if ((PINC&(1<<PC3)) && (active[10] || active[11] || active[12])) { // SENS_12
		dif += 4.5;
		active[11] = 1;
		on_track++;
	}
	else {
		active[11] = 0;
	}
	if ((PINC&(1<<PC2)) && (active[11] || active[12] || active[13])) { // SENS_13
		dif += 5.5;
		active[12] = 1;
		on_track++;
	}
	else {
		active[12] = 0;
	}
	if ((PINC&(1<<PC1)) && (active[12] || active[13])) { // SENS_14
		dif += 6.5;
		active[13] = 1;
		on_track++;
	}
	else {
		active[13] = 0;
	}

	if (!on_track) {
		for (i=0; i<14; i++) {
			active[i] = 1;
		}
		dif = 0;
	}

	
	if (on_track) {
		dif = dif/(float)on_track;

	}
	
	P = dif*kp;
	
	I = I + dif;
	I = I * ki;

	rate = dif - pdif;
	D = rate * kd;

	turn = P + I + D;
	
	if (tsop) {
		l_speed = round(Tp + turn);
		r_speed = round(Tp - turn);
	}
	else {
		l_speed = 0;
		r_speed = 0;
	}	
	
	if (l_speed > 0) {
		PORTB |= (1<<PB0);
		PORTA &= ~(1<<PA0);
	}
	else {
		PORTA |= (1<<PA0);
		PORTB &= ~(1<<PB0);
	}

	if (r_speed > 0) {
		PORTB |= (1<<PB2);
		PORTB &= ~(1<<PB1);
	}
	else {
		PORTB |= (1<<PB1);
		PORTB &= ~(1<<PB2);
	}
	
	OCR1A = abs(l_speed);
	OCR1B = abs(r_speed);
	
//	OCR1A = 0;
//	OCR1B = 0;

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

	LCD_GoTo(0,1);
	LCD_WriteText("l=");
	itoa(l_speed,tab,10);
	LCD_WriteText(tab);
	
	LCD_WriteText("   r=");
	itoa(r_speed,tab,10);
	LCD_WriteText(tab);


}

int main(void) {
	init_ports();
	init_timers();
	init_interrupts();
	LCD_Initalize();

	dif = I = 0;
	
//	tsop = 1;
	tsop=0;
		
	count = 0;
	petla = 0;

	OCR1A = 0;
	OCR1B = 0;

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
			set_motors();

//			print_sensors();
			
			petla = 0;
		}
	}

	return 0;
}
