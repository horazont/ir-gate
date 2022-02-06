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

#define BUFFER_SIZE (0x08)
#define BUFFER_MASK (0x07)

#define TIMER_MODE_DISABLED (0)
#define DELAY_EN_MASK ((1<<CS10) | (1<<CS11) | (1<<CS12))
#define DELAY_EN_FLAG ((1<<CS11)) // enabled, div-8 prescaler (that way, we get a 1us timer)
#define TIMER_DELAY_FLAG (1<<WGM12)
#define TIMER_MODE_COUNTER (DELAY_EN_FLAG)
#define TIMER_MODE_DELAY (DELAY_EN_FLAG | TIMER_DELAY_FLAG)

// STATE MACHINE
//                   COM0A0  TIMER1_EN  TIMER1_CTC
// idle              0       0          x
// sin-repeat-hi     1       1          0
// sin-repeat-delay  1       1          1
// sin-hold          0       1          1

// TRANSITIONS
// idle [PCINT0 w/ !SIN] -> sin-repeat-hi
// idle [PCINT0 w/ SIN] -> idle
// sin-repeat-hi [PCINT0 w/ SIN] -> sin-repeat-delay
// sin-repeat-hi [PCINT0 w/ !SIN] -> sin-repeat-hi
// sin-repeat-delay [PCINT0 w/ !SIN] -> sin-repeat-hi
// sin-repeat-delay [PCINT0 w/ SIN] -> sin-repeat-delay
// sin-repeat-delay [TIM1] -> sin-hold
// sin-hold [PCINT0 w/ !SIN] -> sin-repeat-hi
// sin-hold [PCINT0 w/ SIN] -> sin-hold
// sin-hold [TIM1] -> idle

// INTERRUPTS
// PCINT0:
//   [idle: !COM0A0 && !TIMER1_EN] && SIN -> idle (noop)
//   [idle: !COM0A0 && !TIMER1_EN] && !SIN -> sin-repeat-hi: COM0A0 := 1, TIMER1_EN := 1, TIMER1_CTC := 0
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && !TIMER1_CTC] && SIN -> sin-repeat-delay: COM0A0 := 1, TIMER1_EN := 1, TIMER1_CTC := 1, emit duration, reset timer, set deadline
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && !TIMER1_CTC] && !SIN -> sin-repeat-hi (noop)
//   [sin-repeat-delay: COM0A0 && TIMER1_EN && TIMER1_CTC] && SIN -> sin-repeat-delay (noop)
// ! [sin-repeat-delay: COM0A0 && TIMER1_EN && TIMER1_CTC] && !SIN -> sin-repeat-hi: TIMER1_CTC := 0, reset timer, emit zero
//   [sin-hold: !COM0A0 && TIMER1_EN && TIMER_CTC] && SIN -> sin-hold (noop)
//   [sin-hold: !COM0A0 && TIMER1_EN && TIMER_CTC] && !SIN -> sin-repeat-hi: COM0A0 := 1, TIMER1_CTC := 0, emit duration, reset timer
//
// TIMER1_COMPA
//   !TIMER1_EN -> noop
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && !TIMER1_CTC] -> emit 0xfe (overflow)
//   [sin-repeat-delay: COM0A0 && TIMER1_EN && TIMER1_CTC] -> COM0A0 := 0, reset timer
//   [sin-hold: !COM0A0 && TIMER1_EN && TIMER1_CTC] -> TIMER1_EN := 0, emit 0xff (end of symbol)

// this is tricky! the amount of lost cycles because of the TSOP1738 depends on the signal quality (obviously)
// 20 might be too much when signal conditions are excellent, but when signal conditions are terrible, 10 may be not enough
#define SOUT_DELAY_H (0)
#define SOUT_DELAY_L (160)

// 250 cycles of the 38 kHz carrier
#define EXPIRE_DELAY_H (25)
#define EXPIRE_DELAY_L (202)

// 2.5ms should be plenty to detect end-of-symbol, while allowing for proper repeats
#define HOLD_DELAY_H (9)
#define HOLD_DELAY_L (205)

static inline __attribute__((always_inline)) void set_timer_delay(uint8_t lo, uint8_t hi) {
    OCR1AH = hi;
    OCR1AL = lo;
    TCNT1H = 0;
    TCNT1L = 0;
}

static inline __attribute__((always_inline)) void disable_timer() {
    TCCR1B = TIMER_MODE_DISABLED;
}

static inline __attribute__((always_inline)) void program_delay(uint8_t lo, uint8_t hi) {
    disable_timer();
    set_timer_delay(lo, hi);
    TCCR1B = TIMER_MODE_DELAY;
}

static inline __attribute__((always_inline)) void program_counter() {
    disable_timer();
    // counter shall notify us of overflow
    set_timer_delay(EXPIRE_DELAY_L, EXPIRE_DELAY_H);
    TCCR1B = TIMER_MODE_COUNTER;
}

static inline __attribute__((always_inline)) uint16_t read_timer() {
    const uint8_t lo = TCNT1L;
    const uint8_t hi = TCNT1H;
    return (lo | (hi << 8));
}

#define MSG_END_OF_SYMBOL (0xff)
#define MSG_OVERFLOW (0xfe)
#define MSG_UNDERFLOW (0x01)
#define MSG_FLAG_PAUSE (0x01)

ISR(PCINT0_vect) {
    // read input as soon as possible
    const uint8_t nen = (PINB & (1<<PINB0)) >> PINB0;

    const uint8_t output_enabled = SOUT_EN_REG & SOUT_EN_BIT(1);
    if (output_enabled) {
        if (TCCR1B & TIMER_DELAY_FLAG) {
            // sin-repeat-delay state
            if (nen) {
                // signal is low, so nothing to do (noise?)
                return;
            }
            // XXX: signal is high while we were still emitting the "delay slot"
            // while we can treat the lo-ness as noise, we cannot accurately reflect this via the serial, hence we emit a 0x00 here to signal the condition
            UDR = MSG_UNDERFLOW;
            program_counter();  // switches to sin-repeat-hi state, effectively
        } else {
            // sin-repeat-hi state
            if (!nen) {
                // signal is high, so nothing to do (noise?)
                return;
            }
            // signal went low, enter delay state
            // first we need to read the counter value
            const uint16_t value = read_timer();
            program_delay(SOUT_DELAY_L, SOUT_DELAY_H);  // switches to sin-repeat-delay state, effectively
            // we count in units of 32 us; that should suffice for all the protocols, while not causing us overflows. (units of 32us let the division degrade into a shr)
            // round properly
            const uint8_t count = ((value + 16) / 32 + 1) << 1;
            if (count >= MSG_OVERFLOW) {
                UDR = MSG_OVERFLOW - 2;  // minus two to keep the signalling of signal vs. pause
            } else {
                UDR = count;
            }
        }
    } else {
        // either sin-hold or idle states
        // in both cases, we transition to sin-repeat-hi, if the input gets us a signal
        if (nen) {
            // no signal on input (noise?)
            return;
        }

        uint16_t value = 0;
        if (TCCR1B) {
            // if timer enabled, we need to send the current count once we recorded the transition
            value = read_timer();
        }

        // enable output waveform
        SOUT_EN_REG |= SOUT_EN_BIT(1);
        // configure timer
        program_counter();

        if (value > 0) {
            const uint8_t count = ((value + 16) / 32 + 1) << 1;
            if (count >= MSG_OVERFLOW) {
                UDR = MSG_OVERFLOW - 1;  // minus one to keep the pause flag
            } else {
                UDR = count | 1;  // add the pause flag
            }
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    const uint8_t output_enabled = SOUT_EN_REG & SOUT_EN_BIT(1);
    const uint8_t delay_mode = TCCR1B & TIMER_DELAY_FLAG;
    if (!delay_mode && output_enabled) {
        UDR = MSG_OVERFLOW;
    } else if (output_enabled) {
        // delay timer expired
        // disable output and configure hold timer
        SOUT_EN_REG &= ~SOUT_EN_BIT(1);
        program_delay(HOLD_DELAY_L, HOLD_DELAY_H);
    } else if (!output_enabled && delay_mode) {
        // hold timer expired
        // disable timer to return to idle state, emit end of symbol
        disable_timer();
        UDR = MSG_END_OF_SYMBOL;
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

    /** 38 kHz PWM configuration */

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

    /** signal input interrupt configuration */

    PCMSK0 = ( 0
               | (1<<PCINT0)  // enable pin change interrupt for PB0
               );
    GIMSK = ( 0
              | (1<<PCIF0)  // enable pin change interrupts for 0..7
              );

    /** generic delay/counter configuration */

    TCCR1A |= 0;
    TCCR1C |= 0;
    TCCR1B |= 0;
    TIMSK |= ( 0
               | (1<<OCIE1A)
               );

    /** USART configuration */

    // baud rate follows from fosc / (8 * (UBRR + 1); division by 8 because async mode with double speed
    // we can not reasonably reach beyond 57600 with that.
    UBRRH = 0;
    UBRRL = 16;
    // => 58823 Baud, should be close enough. this allows us to send a delay symbol every ~six strobes of the 38 kHz clock (170 us), which should be good enough for everything, as most codes use symbols longer than that (at least pauses)
    UCSRA = ( 0
              | (1<<U2X) // double speed mode
              );
    UCSRC = ( 0
              /* UCSZ[1:0] := */ | (1<<UCSZ0) | (1<<UCSZ1) // with UCSZ[2] = 0, this configures 8 bit chars
              );
    UCSRB = ( 0
              /* UCSZ[2] := */ | 0 // with UCSZ[1:0] set above, this configures 8 bit chars
              | (1<<TXEN)  // enable transmitter
              );

    sei();  // setup done, we can now safely engage in doing things
    while (1) {
        __asm__ __volatile__ ("nop");
    }
}
