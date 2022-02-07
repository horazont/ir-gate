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
#define SOUT_EN_SHIFT (COM0A0)

#define BUFFER_SIZE (0x08)
#define BUFFER_MASK (0x07)

#define TIMER1_CLK_MASK ((1<<CS10) | (1<<CS11) | (1<<CS12))
#define TIMER1_CLK_FLAG ((1<<CS11) | (1<<CS10)) // enabled, div-64 prescaler (that way, we get an 8us timer)
#define TIMER1_MODE_DISABLED (0)
#define TIMER1_MODE_COUNTER (TIMER1_CLK_FLAG)

// STATE MACHINE
//                   COM0A0  TIMER1_EN  MAY_RECEIVE  PCINT0  RXCIE  OCIE1A
// idle              x       0          x            1       1      0
// sin-repeat-hi     1       1          0            1       0      1
// sin-repeat-delay  1       1          0            1       0      0
// sin-hold          0       1          0            1       0      1
// inject            x       1          x            0       1      0

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
//   [idle: !TIMER1_EN] && !SIN -> sin-repeat-hi: COM0A0 := 1, TIMER1_EN := 1, OCIE1B := 1, RXCIE := 0, set OCR1AL = 0xff, OCR1AH = 0
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && OCIE1A] && SIN -> sin-repeat-delay: COM0A0 := 1, TIMER1_EN := 1, OCIE1B := 0, emit duration, reset timer, set deadline
//   [sin-repeat-hi: COM0A0 && TIMER1_EN && OCIE1A] && !SIN -> sin-repeat-hi (noop)
//   [sin-repeat-delay: COM0A0 && TIMER1_EN && !OCIE1A] && SIN -> sin-repeat-delay (noop)
// ! [sin-repeat-delay: COM0A0 && TIMER1_EN && !OCIE1A] && !SIN -> sin-repeat-hi: TIMER1_CTC := 0, reset timer, emit zero
//   [sin-hold: !COM0A0 && TIMER1_EN && OCIE1A] && SIN -> sin-hold (noop)
//   [sin-hold: !COM0A0 && TIMER1_EN && OCIE1A] && !SIN -> sin-repeat-hi: COM0A0 := 1, TIMER1_CTC := 0, emit duration, reset timer
//   [inject: RXCIE] -> inject; PCINT0 := 0
//
// TIMER1_COMPB
//   !TIMER1_EN -> noop
//   [sin-repeat-hi: PCINT0 && COM0A0 && TIMER1_EN && OCIE1A] -> force idle, too long a symbol
//   [sin-repeat-delay: PCINT0 && COM0A0 && TIMER1_EN && !OCIE1A] -> COM0A0 := 0, OCIE1A := 1
//   [sin-hold: PCINT0 && !COM0A0 && TIMER1_EN && TIMER1_CTC] -> idle: TIMER1_EN := 0, PCINT0 := 1, RXCIE := 1, emit 0xff (end of symbol)
//   [inject: RXCIE] && buffer non empty -> inject: COM0A0 := v, TIMER1_EN := 1, TIMER1_CTC := 1, PCINT0 := 0, set timer to value
//   [inject: RXCIE] && buffer empty && COM0A0 -> inject: synthesize end of symbol
//   [inject: RXCIE] && buffer empty && !COM0A0 -> idle: TIMER1_EN := 0, PCINT0 := 1, RXCIE := 1
//
// TIMER1_COMPA
//   increase OCR1AH by one, emit overflow frame according to output status
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
    const uint8_t currw = BUFFER.wrptr;
    const uint8_t next = (currw + 1) & BUFFER_MASK;
    const uint8_t soon = (currw + 2) & BUFFER_MASK;
    const uint8_t later = (currw + 3) & BUFFER_MASK;
    const uint8_t future = (currw + 4) & BUFFER_MASK;
    const uint8_t currr = BUFFER.rdptr;
    return currr == future || currr == later || currr == soon || currr == next;
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

static inline __attribute__((always_inline)) void set_output_to(uint8_t en) {
    TCNT0 = 0;
    SOUT_EN_REG = (SOUT_EN_REG & (~SOUT_EN_BIT(1))) | SOUT_EN_BIT(en);
}

static inline __attribute__((always_inline)) void set_output_enable() {
    TCNT0 = 0;
    SOUT_EN_REG |= SOUT_EN_BIT(1);
}

static inline __attribute__((always_inline)) void clear_output_enable() {
    SOUT_EN_REG &= ~SOUT_EN_BIT(1);
}

static inline __attribute__((always_inline)) uint8_t is_output_enabled() {
    return SOUT_EN_REG & SOUT_EN_BIT(1);
}

static inline __attribute__((always_inline)) uint8_t is_output_enabled_bit() {
    return is_output_enabled() >> SOUT_EN_SHIFT;
}

static inline __attribute__((always_inline)) void set_counting_for_forwarding() {
    OCR1AH = 0;
    OCR1AL = 0xff;
    TIMSK |= (1<<OCIE1A);
}

static inline __attribute__((always_inline)) void clear_counting_for_forwarding() {
    TIMSK &= ~(1<<OCIE1A);
}

static inline __attribute__((always_inline)) uint8_t is_counting_for_forwarding() {
    return TIMSK & (1<<OCIE1A);
}

static inline void send_counter(uint16_t cnt, uint8_t pause_bit) {
    // cnt is in units of 8us, we want units of 16us -> strip the least significant bit
    // output is shifted to the left by one bit, but we already stripped the LSB by masking it
    // LSB should be pause bit
    // most simple assembly of output ever ^_^
    const uint8_t value = (cnt & 0xfe) | pause_bit;
    UDR = value;
}

// TIMING REQUIREMENTS
//
// Minimum pulse length: 158us
// Common maximum pulse length: 4000us, could possibly do with 2000us especially for DENON coding
// Maximum pulse length: 9000us (27100us for NIKON, 15500us for B&O which we deliberately not support for now)
//
// More notes:
// - we can send at most 7 bits of length information per UART frame -- we need one bit for pulse vs. pause
// - one UART frame takes 100us (or 800 instructions) to transmit
// - a 38 kHz cycle is 26.316 us (26 us for our odd frequency)
// - => we can send one UART frame per ~four strobes of 38 kHz clock at most
// - the timer can run at periods of 0.125 us, 1 us, 8 us, or 32 us
// - we need four special symbols for the UART output:
//   - overflow pulse
//   - overflow pause
//   - underflow pause
//   - end-of-symbol(? possibly not needed)
//
// Solution:
// - Timer resolution is configured to 8us (64 step prescaler)
// - Input/output count duration in units of 16us
//   => minimum 16us (~half a 38 kHz strobe), maximum 2032us
// - When the timer overflows (use COMPB for that maybe, to have it separate from the expiry of the timer), send an overflow/repeat frame
//   This is O(t) for long signals
// - Consider a pause of 30ms end-of-signal
//   This is longer than the repeat interval of any
//
// UART Coding:
// - least significant bit is pulse(0)/pause(1)
// - remaining seven bits are duration in units of 16us
// - End of symbol: 0x00 (zero-length pulse) after a pause
// - Underflow: 0x01 (zero-length pause) after a pulse
// - Overflows are coded by transmitting the same LSB twice or more times in a row (which should cause an addition)
//
// Timeouts/Deadlines:
// - "delay slot" (continuation of the 38 kHz signal after end of pulse to accomodate slow IR ICs): 56us (slightly more than two full cycles; seven counts of TIMER1)
// - "hold" (locking of the output after the last pulse and time until end of symbol): 16ms (2000 counts of TIMER1)
//
// Remaining issues:
// - How can the receiving end of our UART symbols reliably detect repeat frames and their timing? Do we care about the accurate timing? Probably not.
// - Implementation of overflow checking using COMPB -- we only need to use COMPB to emit overflow frames, we do NOT need to keep track of anything as the mod operation required for sending the final frame is just an &

#define SOUT_DELAY_H (0)
#define SOUT_DELAY_L (7)

// 16ms
#define HOLD_DELAY_H (0x07)
#define HOLD_DELAY_L (0xd0)

static inline __attribute__((always_inline)) void set_timer_delay(uint8_t lo, uint8_t hi) {
    OCR1BH = hi;
    OCR1BL = lo;
    TCNT1H = 0;
    TCNT1L = 0;
}

static inline __attribute__((always_inline)) void set_timer_delay_u16(uint16_t d) {
    OCR1BH = (d >> 8) & 0xff;
    OCR1BL = d & 0xff;
    TCNT1H = 0;
    TCNT1L = 0;
}

static inline __attribute__((always_inline)) void disable_timer() {
    TCCR1B = TIMER1_MODE_DISABLED;
}

static inline __attribute__((always_inline)) void program_delay(uint8_t lo, uint8_t hi) {
    disable_timer();
    set_timer_delay(lo, hi);
    TCCR1B = TIMER1_MODE_COUNTER;
}

static inline __attribute__((always_inline)) void program_delay_u16(uint16_t d) {
    disable_timer();
    set_timer_delay_u16(d);
    TCCR1B = TIMER1_MODE_COUNTER;
}

static inline __attribute__((always_inline)) void program_counter() {
    disable_timer();
    // counter shall notify us of overflow
    set_counting_for_forwarding();
    set_timer_delay(0xff, 0xff);
    TCCR1B = TIMER1_MODE_COUNTER;
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
    clear_output_enable();
    clear_counting_for_forwarding();
}

#define MSG_END_OF_SYMBOL (0x00)

// force a pause of maximum length
#define CMD_END_OF_SYMBOL (0xff)

static void start_inject(uint8_t cmd) {
    const uint8_t count = (cmd & 0xfe);
    const uint8_t en = (cmd & 1) ^ 1;
    set_output_to(en);
    const uint16_t timer_value = (uint16_t)(count + 1);
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

    const uint8_t output_enabled = is_output_enabled();
    if (output_enabled) {
        if (is_counting_for_forwarding()) {
            // sin-repeat-hi state
            if (!nen) {
                // signal is high, so nothing to do (noise?)
                return;
            }
            // signal went low, enter delay state
            // first we need to read the counter value
            const uint16_t value = read_timer();
            // switches to sin-repeat-delay state, effectively
            program_delay(SOUT_DELAY_L, SOUT_DELAY_H);
            clear_counting_for_forwarding();
            send_counter(value, 0);
        } else {
            // sin-repeat-delay state
            if (nen) {
                // signal is low, so nothing to do (noise?)
                return;
            }
            // XXX: signal is high while we were still emitting the "delay slot"
            // switch to sin-repeat-hi state, effectively, while re-using the current counter; this treats the short pause as noise.
            program_counter();
        }
    } else {
        // either sin-hold or idle states
        // in both cases, we transition to sin-repeat-hi, if the input gets us a signal
        if (nen) {
            // no signal on input (noise?)
            return;
        }

        uint16_t value = 0;
        if (timer_enabled) {
            // if timer enabled, we need to send the current count once we recorded the transition
            value = read_timer();
        } else {
            // coming from idle, need to switch modes
            clear_may_receive();
            clear_inject_mode();
        }

        // enable output waveform
        set_output_enable();
        // configure timer
        program_counter();

        if (value > 0) {
            send_counter(value, 1);
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    const uint8_t output_enabled = is_output_enabled_bit();
    const uint8_t pause_bit = output_enabled ^ 1;
    (void)OCR1AL;
    const uint8_t cnt = OCR1AH;
    OCR1AH = cnt + 1;
    OCR1AL = 0xff;
    UDR = 0xfe | pause_bit;
}

// NOTE: we use COMPB for orchestration, because COMPA has a higher priority
// over PCINT0, and we need high priority over PCINT0 for overflow handling in
// COMPA
ISR(TIMER1_COMPB_vect) {
    const uint8_t output_enabled = is_output_enabled();
    const uint8_t forward_mode = is_forward_mode();
    const uint8_t inject_mode = is_inject_mode();

    if (forward_mode && inject_mode) {
        // this should never ever happen. the timer is not enabled in idle, and the other ISRs disable the respective other mode first thing
        // disable the timer, clear the output, to return to idle state.
        force_idle();
        return;
    }

    if (forward_mode) {
        const uint8_t is_forwarding = is_counting_for_forwarding();
        if (output_enabled) {
            if (is_forwarding) {
                // sin-repeat-hi: massive overflow (1s symbol), force idle
                force_idle();
            } else {
                // sin-repeat-delay: end of delay, switch to pause mode
                set_counting_for_forwarding();
                // not going through program_delay or so because those clear the counter, which we do not want
                OCR1BH = HOLD_DELAY_H;
                OCR1BL = HOLD_DELAY_L;
                clear_output_enable();
            }
        } else {
            // sin-hold: hold timer expired, emit end-of-symbol and enter idle state
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
    PORTB = (0
             | (1<<PORTB2)  // pull PWM output high if disabled (the level shifter is inverting)
             );
    PORTD = (0
             | (1<<PORTD5)  // baud rate configuration pin; pull up by default
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
    OCR0A = 52;  // this provides 38.46 kHz on an 8 MHz clock; that should be ok. The IR receiver IC in the AVR has a band pass for 37 kHz..40kHz, so I'd like to err on the high side here.

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
               | (1<<OCIE1B)
               );

    /** USART configuration */

    // baud rate follows from fosc / (16 * (UBRR + 1); division by 16 because async mode
    if (PIND & (1<<PIND5)) {
        // Non-standard 100000 Baud
        UBRRH = 0;
        UBRRL = 4;
    } else {
        UBRRH = 0;
        UBRRL = 12;
        // => 38461 Baud, should be close enough. this allows us to send a delay symbol every ~ten strobes of the 38 kHz clock (260 us), which should be good enough for everything, as most codes use symbols longer than that (at least pauses)
    }
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
