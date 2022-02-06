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
//                   COM0A0  TIMER1_EN  TIMER1_CTC  MAY_RECEIVE  PCINT0  RXCIE
// idle              x       0          x           x            1       1
// sin-repeat-hi     1       1          0           0            1       0
// sin-repeat-delay  1       1          1           0            1       0
// sin-hold          0       1          1           0            1       0
// inject            x       1          1           x            0       1

// TRANSITIONS
// idle [PCINT0 w/ !SIN] -> sin-repeat-hi
// idle [PCINT0 w/ SIN] -> idle
// idle [RX w/ signal] -> inject-hi
// idle [RX w/o signal] -> inject-lo
// sin-repeat-hi [PCINT0 w/ SIN] -> sin-repeat-delay
// sin-repeat-hi [PCINT0 w/ !SIN] -> sin-repeat-hi
// sin-* [RX] -> sin-* (disable RXCIE)
// sin-repeat-delay [PCINT0 w/ !SIN] -> sin-repeat-hi
// sin-repeat-delay [PCINT0 w/ SIN] -> sin-repeat-delay
// sin-repeat-delay [TIM1] -> sin-hold
// sin-hold [PCINT0 w/ !SIN] -> sin-repeat-hi
// sin-hold [PCINT0 w/ SIN] -> sin-hold
// sin-hold [TIM1] -> idle
// inject [RX] -> inject
// inject [PCINT0] -> inject (disable RXCIE)
// inject [TIM1, next symbol available] -> inject
// inject [TIM1, next symbol not available && COM0A0] -> inject (with emulated end of symbol)
// inject [TIM1, next symbol not available] -> idle

// INTERRUPTS
// PCINT0:
//   [idle: !TIMER1_EN] && SIN -> idle (noop)
//   [idle: !TIMER1_EN] && !SIN -> sin-repeat-hi: COM0A0 := 1, TIMER1_EN := 1, TIMER1_CTC := 0, RXCIE := 0
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && !TIMER1_CTC] && SIN -> sin-repeat-delay: COM0A0 := 1, TIMER1_EN := 1, TIMER1_CTC := 1, emit duration, reset timer, set deadline
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && !TIMER1_CTC] && !SIN -> sin-repeat-hi (noop)
//   [sin-repeat-delay: COM0A0 && TIMER1_EN && TIMER1_CTC] && SIN -> sin-repeat-delay (noop)
// ! [sin-repeat-delay: COM0A0 && TIMER1_EN && TIMER1_CTC] && !SIN -> sin-repeat-hi: TIMER1_CTC := 0, reset timer, emit zero
//   [sin-hold: !COM0A0 && TIMER1_EN && TIMER_CTC] && SIN -> sin-hold (noop)
//   [sin-hold: !COM0A0 && TIMER1_EN && TIMER_CTC] && !SIN -> sin-repeat-hi: COM0A0 := 1, TIMER1_CTC := 0, emit duration, reset timer
//   [inject: RXCIE] -> inject; PCINT0 := 0
//
// TIMER1_COMPA
//   !TIMER1_EN -> noop
//   [sin-repeat-hi: PCINT0 && COM0A0 && TIMER1_EN && !TIMER1_CTC] -> emit 0xfe (overflow)
//   [sin-repeat-delay: PCINT0 && COM0A0 && TIMER1_EN && TIMER1_CTC] -> COM0A0 := 0, reset timer
//   [sin-hold: PCINT0 && !COM0A0 && TIMER1_EN && TIMER1_CTC] -> idle: TIMER1_EN := 0, PCINT0 := 1, RXCIE := 1, emit 0xff (end of symbol)
//   [inject: RXCIE] && buffer non empty -> inject: COM0A0 := v, TIMER1_EN := 1, TIMER1_CTC := 1, PCINT0 := 0, set timer to value
//   [inject: RXCIE] && buffer empty && COM0A0 -> inject: synthesize end of symbol
//   [inject: RXCIE] && buffer empty && !COM0A0 -> idle: TIMER1_EN := 0, PCINT0 := 1, RXCIE := 1
//
// RXC:
//   [idle: !COM0A0 && !TIMER1_EN] -> inject: COM0A0 := v, TIMER1_EN := 1, TIMER1_CTC := 1, PCINT0 := 0, set timer to value
//   [inject: TIMER1_EN && RXCIE && !PCINT0] -> inject: write to buffer


typedef struct {
    uint8_t rdptr;
    uint8_t wrptr;
    uint8_t buf[BUFFER_SIZE];
} buffer_t;

static volatile buffer_t BUFFER = {
    .rdptr = 0,
    .wrptr = 0,
    .buf = {0},
};

// ONLY SAFE TO CALL FROM NON-NESTABLE INTERRUPTS
// ALSO ONLY SAFE TO CALL AFTER ASSERTING THAT NO UNDERRUN WILL HAPPEN
static inline __attribute__((always_inline)) uint8_t buf_read() {
    const uint8_t curr = BUFFER.rdptr;
    const uint8_t next = (BUFFER.rdptr + 1) & BUFFER_MASK;
    BUFFER.rdptr = next;
    return BUFFER.buf[curr];
};

// ONLY SAFE TO CALL FROM NON-NESTABLE INTERRUPTS
// ALSO ONLY SAFE TO CALL AFTER ASSERTING THAT NO OVERRUN WILL HAPPEN
static inline __attribute__((always_inline)) void buf_write(uint8_t value) {
    const uint8_t curr = BUFFER.wrptr;
    const uint8_t next = (BUFFER.wrptr + 1) & BUFFER_MASK;
    BUFFER.buf[curr] = value;
    BUFFER.wrptr = next;
};

// ONLY SAFE TO CALL FROM NON-NESTABLE INTERRUPTS
static inline __attribute__((always_inline)) uint8_t will_underrun() {
    return BUFFER.rdptr == BUFFER.wrptr;
};

// ONLY SAFE TO CALL FROM NON-NESTABLE INTERRUPTS
static inline uint8_t should_slow_down() {
    const uint8_t next = (BUFFER.wrptr + 1) & BUFFER_MASK;
    const uint8_t soon = (BUFFER.wrptr + 2) & BUFFER_MASK;
    const uint8_t later = (BUFFER.wrptr + 3) & BUFFER_MASK;
    const uint8_t future = (BUFFER.wrptr + 4) & BUFFER_MASK;
    return BUFFER.rdptr == future || BUFFER.rdptr == later || BUFFER.rdptr == soon || BUFFER.rdptr == next;
};

// ONLY SAFE TO CALL FROM NON-NESTABLE INTERRUPTS
static inline __attribute__((always_inline)) uint8_t will_overrun() {
    const uint8_t next = (BUFFER.wrptr + 1) & BUFFER_MASK;
    return BUFFER.rdptr == next;
};

static inline __attribute__((always_inline)) uint8_t may_receive() {
    // CTS is active low!
    return ((PORTD & (1<<PORTD4)) >> PORTD4) ^ 1;
}

static inline __attribute__((always_inline)) void set_may_receive() {
    // CTS is active low!
    PORTD &= ~(1<<PORTD4);
}

static inline __attribute__((always_inline)) void clear_may_receive() {
    // CTS is active low!
    PORTD |= (1<<PORTD4);
}

static inline __attribute__((always_inline)) uint8_t is_inject_mode() {
    return UCSRB & (1<<RXCIE);
}

static inline __attribute__((always_inline)) void clear_inject_mode() {
    UCSRB &= ~(1<<RXCIE);
}

static inline __attribute__((always_inline)) void set_inject_mode() {
    UCSRB |= (1<<RXCIE);
}

static inline __attribute__((always_inline)) uint8_t is_forward_mode() {
    return PCMSK0 & (1<<PCINT0);
}

static inline __attribute__((always_inline)) void clear_forward_mode() {
    PCMSK0 &= ~(1<<PCINT0);
}

static inline __attribute__((always_inline)) void set_forward_mode() {
    PCMSK0 |= (1<<PCINT0);
}

static inline __attribute__((always_inline)) uint8_t is_timer_enabled() {
    return TCCR1B != 0;
}

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

static inline __attribute__((always_inline)) void set_timer_delay_u16(uint16_t d) {
    OCR1AH = (d >> 8) & 0xff;
    OCR1AL = d & 0xff;
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

static inline __attribute__((always_inline)) void program_delay_u16(uint16_t d) {
    disable_timer();
    set_timer_delay_u16(d);
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

static inline void force_idle() {
    disable_timer();
    set_may_receive();
    set_forward_mode();
    set_inject_mode();
}

#define MSG_END_OF_SYMBOL (0xff)
#define MSG_OVERFLOW (0xfe)
#define MSG_UNDERFLOW (0x01)
#define MSG_FLAG_PAUSE (0x01)

// ~2.5 ms of pause
#define CMD_END_OF_SYMBOL (0x9d)

static void start_inject(uint8_t cmd) {
    const uint8_t count = cmd >> 1;
    const uint8_t en = (cmd & 1) ^ 1;
    SOUT_EN_REG = (SOUT_EN_REG & (~SOUT_EN_BIT(1))) | SOUT_EN_BIT(en);
    const uint16_t timer_value = (uint16_t)(count + 1) * 32;
    program_delay_u16(timer_value);
}

ISR(PCINT0_vect) {
    // read input as soon as possible
    const uint8_t nen = (PINB & (1<<PINB0)) >> PINB0;
    const uint8_t timer_enabled = is_timer_enabled();

    if (timer_enabled && is_inject_mode()) {
        // inject won a race here
        clear_forward_mode();
        return;
    }

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

        clear_may_receive();
        clear_inject_mode();

        uint16_t value = 0;
        if (timer_enabled) {
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
    const uint8_t forward_mode = is_forward_mode();
    const uint8_t inject_mode = is_inject_mode();

    if (forward_mode && inject_mode) {
        // this should never ever happen. the timer is not enabled in idle, and the other ISRs disable the respective other mode first thing
        // disable the timer, clear the output, to return to idle state.
        disable_timer();
        SOUT_EN_REG &= ~SOUT_EN_BIT(1);
        return;
    }

    if (forward_mode) {
        if (!delay_mode && output_enabled) {
            UDR = MSG_OVERFLOW;
        } else if (forward_mode && output_enabled) {
            // delay timer expired
            // disable output and configure hold timer
            SOUT_EN_REG &= ~SOUT_EN_BIT(1);
            program_delay(HOLD_DELAY_L, HOLD_DELAY_H);
        } else if (forward_mode && !output_enabled && delay_mode) {
            // hold timer expired
            // disable timer to return to idle state, emit end of symbol
            UDR = MSG_END_OF_SYMBOL;
            force_idle();
        }
    } else {
        const uint8_t underrun = will_underrun();
        if (underrun && !output_enabled) {
            // end of buffer and transmission completed, disable timer, return to idle state
            force_idle();
        } else if (underrun) {
            // end of buffer, but we're still high, so we have to inject a low
            start_inject(CMD_END_OF_SYMBOL);
            // after an underrun, a write is always possible
            set_may_receive();
        } else {
            const uint8_t data = buf_read();
            start_inject(data);
            if (!should_slow_down()) {
                // after an underrun, a write is always possible
                set_may_receive();
            }
        }
    }
}

ISR(USART0_RX_vect) {
    const uint8_t data = UDR;
    const uint8_t timer_enabled = is_timer_enabled();
    if (timer_enabled && is_forward_mode()) {
        // forward won a race here
        clear_inject_mode();
        return;
    }

    if (!timer_enabled) {
        // we're coming from idle
        // ensure that forward mode is not engaged
        clear_forward_mode();
        // no need to write the first bit somewhere; this will set up the timers etc. which will move the state machine into the correct state
        start_inject(data);
    } else {
        if (will_overrun()) {
            PORTB |= (1<<DDB3);
            // drop all frames we receive while we cannot receive
            return;
        }
        buf_write(data);
        if (should_slow_down()) {
            // overrun, we have to clear the may receive bit and wait for signal generation
            clear_may_receive();
        }
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
    DDRD = ( 0
             | (1<<DDD4)  // CTS
             );

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

    // baud rate follows from fosc / (16 * (UBRR + 1); division by 8 because async mode with double speed
    // we can not reasonably reach beyond 38400 with that.
    UBRRH = 0;
    UBRRL = 12;
    // => 38461 Baud, should be close enough. this allows us to send a delay symbol every ~ten strobes of the 38 kHz clock (260 us), which should be good enough for everything, as most codes use symbols longer than that (at least pauses)
    UCSRA = ( 0
              // | (1<<U2X) // double speed mode
              );
    UCSRC = ( 0
              /* UCSZ[1:0] := */ | (1<<UCSZ0) | (1<<UCSZ1) // with UCSZ[2] = 0, this configures 8 bit chars
              );
    UCSRB = ( 0
              /* UCSZ[2] := */ | 0 // with UCSZ[1:0] set above, this configures 8 bit chars
              | (1<<RXCIE)  // enable receive complete interrupt
              | (1<<TXEN)  // enable transmitter
              | (1<<RXEN)  // enable receiver
              );
    set_may_receive();

    sei();  // setup done, we can now safely engage in doing things
    while (1) {
        __asm__ __volatile__ ("nop");
    }
}
