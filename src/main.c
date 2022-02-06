#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// NOTE: we are using a bitmask for really fast and simple conflict checks
#define OWNER_NONE (0)
#define OWNER_SIN (0x01)
#define OWNER_UART (0x02)
#define OWNER_RES0 (0x04)
#define OWNER_RES1 (0x08)
#define OWNER_PMASK (0x0f)
#define OWNER_PSHIFT (0)

#define OWNER_FIXED (0x00)
#define OWNER_EXPIRING (0x10)
#define OWNER_ESHIFT (4)
#define OWNER_EMASK (0x10)

#define SOUT_EN_BIT(v) ((v)<<COM0A0) // toggle OC0A on compare match (iff WGM02 set)
#define SOUT_EN_REG (TCCR0A)

#define DELAY_EN_MASK ((1<<CS10) | (1<<CS11) | (1<<CS12))
#define DELAY_EN_FLAG ((1<<CS11) | (1<<CS10)) // enabled, 64 prescaler (that way, a single iteration should be close to half a 38 kHz cycle)

// this is tricky! the amount of lost cycles because of the TSOP1738 depends on the signal quality (obviously)
// 20 might be too much when signal conditions are excellent, but when signal conditions are terrible, 10 may be not enough
#define SOUT_DELAY_H (0)
#define SOUT_DELAY_L (20)

#define EXPIRE_DELAY_H (1)
#define EXPIRE_DELAY_L (0)

// TODO: this turns out to require a lot of instructions to access... might want to optimize this by abusing some register(s), which should be faster to access
volatile uint8_t ownership = OWNER_NONE | OWNER_FIXED;

static inline void set_timer_delay(uint8_t lo, uint8_t hi) {
    OCR1AH = hi;
    OCR1AL = lo;
    TCNT1H = 0;
    TCNT1L = 0;
}

static inline void disable_timer() {
    TCCR1B |= DELAY_EN_FLAG;
}

static inline void enable_timer() {
    TCCR1B |= DELAY_EN_FLAG;
}

static inline void program_delay(uint8_t lo, uint8_t hi) {
    disable_timer();
    set_timer_delay(lo, hi);
    enable_timer();
}

ISR(PCINT0_vect) {
    const uint8_t curr_owner = ownership & OWNER_PMASK;
    // if someone who is not us is owning the output, we have to step back
    if (curr_owner & ~OWNER_SIN) {
        return;
    }
    const uint8_t nen = (PINB & (1<<PINB0)) >> PINB0;
    if (!nen) {
        // enable the output immediately
        TCNT0 = 0;
        SOUT_EN_REG |= SOUT_EN_BIT(1);
        // halt the delay timer, because we do not need it right now
        disable_timer();
        ownership = OWNER_SIN;
    } else {
        // program the delay to keep the output going for a few more cycles
        program_delay(SOUT_DELAY_L, SOUT_DELAY_H);
        ownership = OWNER_SIN;
    }
}

ISR(TIMER1_COMPA_vect) {
    disable_timer();
    const uint8_t curr_owner = ownership;
    const uint8_t sin_owned = curr_owner & OWNER_SIN;
    const uint8_t expiring = curr_owner & OWNER_EXPIRING;
    if (sin_owned && !expiring) {
        // special case: this is the delay slot of the SIN-based output, because we have to compensate for the TS1738 having a delay
        // disable the output
        SOUT_EN_REG &= ~SOUT_EN_BIT(1);
        // prepare ownership expiry
        set_timer_delay(EXPIRE_DELAY_L, EXPIRE_DELAY_H);
        enable_timer();
        ownership = OWNER_SIN | OWNER_EXPIRING;
        // here, we keep the timer enabled to be notified when the expiry is over
        return;
    }
    if (expiring) {
        // release ownership now
        ownership = OWNER_NONE;
    }
}

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
                /* COM0A := */ | 0             // OC0A disconnected by default
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

    PCMSK0 = ( 0
               | (1<<PCINT0)  // enable pin change interrupt for PB0
               );
    GIMSK = ( 0
              | (1<<PCIF0)  // enable pin change interrupts for 0..7
              );

    TCCR1A |= 0;
    TCCR1C |= 0;
    TCCR1B |= ( 0
                /* WGM1[3:2] := */ | (1<<WGM12)  // count up to OCR1A
                /* CS1       := */ | 0
                );
    TIMSK |= ( 0
               | (1<<OCIE1A)  // effectiely interrupt on reset
               );

    sei();  // setup done, we can now safely engage in doing things
    while (1) {
        __asm__ __volatile__ ("nop");
    }
}
