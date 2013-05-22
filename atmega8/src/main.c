#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Odbiornik podczerwieni SFH5110  przyłączona do portu  PB0 
#define RC5_IN   (PINB & (1<<PB1))
#define POWER_OFF 12

//
typedef unsigned char u8;
typedef unsigned int  uint;

// Zmienne globalne pełnią rolę  programowych zegarów
// napędzanych przerwaniem TIMER0_OVF
volatile u8 timerL; 
volatile u8 timerH; 

//---------------------------------------------------------------
// Funkcja konfiguruje i uruchamia Timer0 
// oraz włącza przerwanie od przepełnienia timera,
// przerwanie powinno występować co 32us.  
//---------------------------------------------------------------
void init_rc5()
{
  //atmega8
  TCCR0 = (1<<CS00);  // włącza Timer0  
  TIMSK = (1<<TOIE0); // włącza przerwanie "Timer0 Overflow"

/*
  //atmega88
  TCCR0B = (1<<CS00);
  TIMSK0 = (1<<TOIE0);
*/
  // Zezwala na przerwania 
  sei();
}

//---------------------------------------------------------------
// Procedura obsługi przerwania  Timer0 Overflow"
//---------------------------------------------------------------
ISR(TIMER0_OVF_vect)
{
   volatile  static u8 inttemp;

   // zmienna timerL zwiększa się co 32us
   timerL++;

   // zmienna timerH  zwiększa się co 8.192ms (32us*256) 
   inttemp++;
   if(!inttemp ) timerH++;
}

//---------------------------------------------------------------
// Funkcja wykrywa i dekoduje  komendę pilota RC5                                             
//---------------------------------------------------------------
 uint detect()
 {
    u8 temp;
    u8 ref1;
    u8 ref2;
    u8 bitcnt;
    uint command;

    timerH  = 0;
    timerL  = 0;

    // Czeka na okres ciszy na linii wejścia uC trwający  3.5ms
    // Jeśli nie wykryje takiego okresu ciszy w ciągu 131ms,
    // to kończy działanie funkcji z błędem
    while( timerL<110)
    {
       if(timerH>=16)  return  command = -1;

       if(!RC5_IN) timerL = 0;
    }

    // Czeka na  pierwszy bit startowy. 
    // Jeśli nie wykryje bitu startowego w ciągu 131ms,
    // to kończy działanie funkcji z błędem
    while(RC5_IN)  
         if(timerH>=16)  return command = -1 ;

    // Pomiar czasu trwani niskiego poziom syganłu 
    // w pierwszym bicie startowym.
    // Jeśli nie wykryje rosnącego zbocza sygnału w ciągu  
    // 1ms, to kończy działanie funkcji z błędem 
    timerL = 0;
    while(!RC5_IN)
         if(timerL>34) return command = -1;

    //
    temp = timerL;
    timerL = 0;

    // ref1 - oblicza  3/4 czasu trwania bitu
    ref1 =temp+(temp>>1);

    // ref2 - oblicza 5/4 czasu trwania bitu
    ref2 =(temp<<1)+(temp>>1);

 
    // Oczekuje na zbocze opadające sygnału w środku drugiego
    // bitu startowego.
    // Jeśli nie wykryje zbocza w ciągu 3/4 czasu trwania 
    // bitu, to kończy działanie funkcji z błędem 
    while(RC5_IN)
         if(timerL > ref1) return command = -1;

    // W momencie wykrycia zbocza sygnału, synchronizuje
    // zmieną timerL dla próbkowania  bitu toggle
    timerL = 0;

    // Odczytuje dekoduje pozostałe 12 bitów polecenia rc5
    for(bitcnt=0, command = 0; bitcnt <12; bitcnt++)
    {
       // Czeka 3/4 czasu trwania bitu od momentu wykrycia
       // zbocza sygnału w połowie poprzedniego bitu 
       while(timerL < ref1) {};
 
       // Próbkuje - odczytuje port we  uC
       if(!RC5_IN)
       {
          // Jeśli odczytano 0, zapamiętuje w zmiennej 
          // "command" bit o wartości 0         
          command <<= 1 ;

          // Oczekuje na zbocze rosnące sygnału w środku bitu.
          // Jeśli nie wykryje zbocza w ciągu 5/4 czasu trwania 
          // bitu, to kończy działanie funkcji z błędem    
          while(!RC5_IN)
             if(timerL > ref2) return command = -1;
       }
       else
       {
          // Jeśli odczytano 1, zapamiętuje w zmiennej 
          // "command" bit o wartości 1  
          command = (command <<1 ) | 0x01;

          // Oczekuje na zbocze opadające sygnału w środku bitu.
          // Jeśli nie wykryje zbocza w ciągu 5/4 czasu trwania 
          // bitu, to kończy działanie funkcji z błędem 
          while(RC5_IN)
             if(timerL > ref2) return command = -1;
       }

       // W momencie wykrycia zbocza sygnału, synchronizuje
       // zmieną timerL dla próbkowania kolejnego bitu
       timerL = 0;
   }

   // Zwraca kod polecenia rc5
   // bity 0..5 numer przycisku
   // bity 6..10  kod systemu(urządzenia)
   // bit 11 toggle bit
   return command;
 }


int main(void) {

	short tsop = 0;

	// inicjalizacja odbiornika podczerwieni TSOP
	init_rc5();
	
	// wlaczenie obslugi przerwan
	sei();

	// ustawienie portow I/O

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

	// TSOP
	DDRB |= (1<<PB0);
	PORTB &= ~(1<<PB0);

	int counter = (1<<3);
	short running = 1;
  uint cmd, toggle;
  u8 out;
  int on_state = 0, toggle_state = -1;

	while (1) {
    cmd = detect();
    if(cmd != -1) {
      out    = cmd & 0b0000000000111111;
      toggle = (cmd & 0b0000100000000000) >> 11;
      if(out == POWER_OFF) {
        if(-1 == toggle_state) toggle_state = toggle;
        if(toggle_state == toggle) {
          on_state ^= 1;
          toggle_state ^= 1;
          if(on_state) {
            PORTC |= (1<<PC0);
            PORTC |= (1<<PC1);
            PORTD |= (1<<PD6);
            PORTB |= (1<<PB2);
            PORTB |= (1<<PB0);
          }
          else {
            PORTC &= !(1<<PC0);
            PORTC &= !(1<<PC1);
            PORTD &= !(1<<PD6);
            PORTB &= !(1<<PB2);
            PORTB &= ~(1<<PB0);
          }
        }
      }
    }
	}

	return 0;
}
