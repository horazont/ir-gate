#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>



int main() {
    // disable all interrupts until we're done with the setup
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    // this will be a simple LED test
    DDRB = ( 0
             | (1<<DDB3)  // LED output test on PB3
             | (1<<DDB2)  // PWM output
             );
    PORTB |= (1<<DDB3);

    TCCR0A |= ( 0
                /* COM0A := */ | (1<<COM0A0)   // toggle OC0A on compare match (iff WGM02 set)
                /* COM0B := */ | 0             // OC0B disconnected
                /* WGM0  := */ | (1<<WGM00)    // phase correct PCM mode
                );
    TCCR0B |= ( 0
                /* FOC0A := */ | 0           // force output disabled
                /* FOC0B := */ | 0           // force output disabled
                /* WGM02 := */ | (1<<WGM02)  // count up to OCR0A
                /* CS0   := */ | (1<<CS00)   // enabled, no prescaler
                );
    OCR0A = 54;  // this gives roughly 38 kHz on my thing, when running with 8 MHz clkio (from internal RC). might need tuning.

    sei();  // setup done, we can now safely engage in doing things
    while (1) {
        __asm__ __volatile__ ("nop");
    }
}
