#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define thr 80

// zmienne wykorzystywane w algorytmie PID
#define kp 13
#define ki 0.52
#define kd 8
//#define Tp 50//46.0//36
#define dead 22
float P, I, D, dif, pdif, rate, turn, Tp;

volatile int petla;

short active[14], on_track;
int l_speed, r_speed;

int i;

volatile short tsop;

// obsluga przerwania timera0
// zeby co okreslony czas (100x/s) sprawdzac stan czujnikow i ustawiac predkosc silnikow
ISR(TIMER0_COMP_vect) {
	// podczas obslugi przerwania nalezy wykonywac mozliwie najmniej operacji
	// dlatego ustawiana jest jedynie zmienna petla
	petla = 1;
}

// obsluga przerwania INT1
// pochodzi z atmegi8, sluzy wlaczeniu/wylaczeniu robota
ISR(INT1_vect) {
	tsop ^= 1;
}

void init_adc() {
        ADCSRA = (1<<ADEN) //ADEN=1 włączenie przetwornika ADC)
                |(1<<ADPS0) // ustawienie preskalera na 128  
                |(1<<ADPS1)
                |(1<<ADPS2);

        ADMUX = (1<<REFS0)
                |(1<<ADLAR);
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
	// normalna operacja portowa, OC2 rozloczone
	// zegar - CLK/1024 - taktowanie zegara 16MHz/1024 = 15625Hz
	TCCR0=(1<<WGM01)|(1<<CS02)|(1<<CS00);
	// wartosc przy ktorej zegar sie zeruje i generowane jest przerwanie
	OCR0 = 100; // dla 156 czestotliwosc przerwan to ok. 100Hz
	// odmaskowanie przerwania pochodzacego z timera 0
	TIMSK |= (1<<OCIE0);

	// inicjalizacja timera 1 - opis bitow od str. 112 datasheeta
	// wyczysc OC1A/OC1B przy porownaniu, ustaw OC1A/OC1B na dole, (tryb nie odwracajacy)
 	// Fast PWM, 8-bit
	// zegar bez skalowania, co daje czestotliwosc 16MHz/1/256=62.5kHz
	TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);
	TCCR1B=(1<WGM12)|(1<<CS10);
}

void init_interrupts() {
	// inicjalizacja przerwan zewnetrznych - od str. 67 datasheeta
	// MCU Control Register
	// jakakolwiek zmiana logiczna na wejsciu INT1 generuje przerwanie
	MCUCR |= (1<<ISC10);
	// Kiedy bit INT1 jest ustawiony i I-bit w Status Register (SREG) jest ustawiony
	// przerwanie poprzez wejscie zewnetrzne jest wlaczone
	GICR |= (1<<INT1);
}

int sensors[14];

void set_motors() {
	
	pdif = dif;

	dif = 0;
	on_track = 0;

	// odczyt stanu sensorow

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
	// PA3-PA6 odczytywane z ADC
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
		if (sensors[i]) {
			dif += ((float)i - 6.5);
			on_track++;
		}
	}
  if(sensors[0]) dif -= 5.0;
  if(sensors[1]) dif -= 3.5;
  if(sensors[13]) dif += 5.0;
  if(sensors[12]) dif += 3.5;
  //Tp = 60;
  if(on_track >= 5) {
    Tp = 48;    
  } else {
    Tp = 60;
  }

	if (!on_track) {
		for (i=0; i<14; i++) {
			active[i] = 1;
		}
		dif = pdif;
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
	
	//if (tsop) {
		l_speed = round(Tp + turn);
		r_speed = round(Tp - turn);
    if(l_speed < dead) l_speed -= 2*dead;
    if(r_speed < dead) r_speed -= 2*dead;
/*		if (on_track >= 10) {
			l_speed = Tp;
			r_speed = Tp;
		}
*/
  /*
	}
	else {
		l_speed = 0;
		r_speed = 0;
	}	
  */

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
	
}

int main(void) {
	// inicjalizacja
	init_ports();
	init_timers();
	init_interrupts();

	init_adc();

	dif = I = 0;
	
	// po wlaczeniu zasilania robot nie jedzie, dopiero po otrzymaniu sygnalu z pilota
	tsop = 0;
		
	petla = 0;

	// poczatkowo predkosci obu silnikow sa zerowane
	OCR1A = 0;
	OCR1B = 0;

	// wlaczenie obslugi przerwan
	sei();

	while (1) {
		if (petla) {
			set_motors();

			petla = 0;
		}
	}

	return 0;
}
