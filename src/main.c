#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

int main() {
   set_sleep_mode(SLEEP_MODE_IDLE);
   // this will be a simple LED test
   DDRB = (1<<DDB3);
   PORTB |= (1<<DDB3);

   while (1) {
           asm __volatile__ ("nop");
   }
}
