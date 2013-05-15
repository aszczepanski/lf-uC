#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void) {

	// LED1
	DDRC |= (1<<PC0);
//	PORTC |= (1<<PC0);

	// LED2
	DDRC |= (1<<PC1);
//	PORTC |= (1<<PC1);

	// LED3
	DDRB |= (1<<PB2);
//	PORTB |= (1<<PB2);

	// LED4
	DDRD |= (1<<PD6);
//	PORTD |= (1<<PD6);

	int counter = (1<<3);

	while (1) {
		counter<<=1;
		if (counter == (1<<4)) counter=1;
		PORTC &= !(1<<PC0);
		PORTC &= !(1<<PC1);
		PORTD &= !(1<<PD6);
		PORTB &= !(1<<PB2);
		if (counter&1) PORTC |= (1<<PC0);
		else if (counter&(1<<1)) PORTC |= (1<<PC1);
		else if (counter&(1<<2)) PORTB |= (1<<PB2);
		else PORTD |= (1<<PD6);		

		_delay_ms(500);
	}

	return 0;
}