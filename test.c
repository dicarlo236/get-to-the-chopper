#include <avr/io.h>
#include <avr/interrupt.h>

// LOW SIDE CHOPPER FIRMWARE
// requires atmega328pb "pack"
// avrdude modification from "attila zoltan kovacs"
// include file for registers copied to avr library
// avr-gcc
// avr-objcopy
// avr libc and friends

// timer 1 is used to generate pwm
// timer 2 is used to measure the length of the pwm in
// timer 2's overflow is used for the watchdog

// one "tick" is 31.25 us long

#define IN_MIN 32        // minimum expected pwm in pulse length (ticks)
#define IN_MAX 70        // maximum expected pwm in pulse length (ticks)

#define DEADBAND_MIN 36  // values less than this result in 0 output
#define DEADBAND_MAX 60  // values larger than this (but less than in_max) result in full on
#define OUT_MIN 0        // minimum duty cycle (out of 255)
#define OUT_MAX 0xff     // maximum duty cycle (out of 255)
#define MAX_DOGS 3       // number of timer 2 overflows between watchdog checks
#define MIN_EDGES 2      // minimum number of edges required to make watchdog happy
#define MAX_EDGES 12      // maximum number of edges required to make watchdog happy

#define MAX_DOGS 3

uint8_t has_pwm = 0;

uint8_t get_desired_pwm(uint8_t timer_ticks)
{
    if(timer_ticks < DEADBAND_MIN) return 0;
    if(timer_ticks > IN_MAX) return 0;
    if(timer_ticks > DEADBAND_MAX) return OUT_MAX;
    return (timer_ticks - DEADBAND_MIN) * (OUT_MAX - OUT_MIN) / (DEADBAND_MAX - DEADBAND_MIN) + OUT_MIN;
}

void disable()
{
    TCCR1A = 0;
    TCCR1B = 0;
    PORTB = 0;
}

void enable()
{
    TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12)|(1 << WGM13);
    TCCR1B |= (1 << CS10);
}

void init_timers()
{
    // set gate out as output pin
    DDRB |= (1 << DDB1)|(1 << DDB2);

    // timer reset value of 255
    ICR1 = 0x00FF;

    // duty cycle starts at zero
    OCR1A = 0x00;

    // setup up for pwm out
    TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12)|(1 << WGM13);


    // clear interuppts
    cli();

    // 1/32 prescaler for timer 2 (pwm in)
    TCCR2B |= (1 << CS21)|(1 << CS20);

    // initialize counter
    TCNT2 = 0;

    // enable overflow interrupt (for watchdog)
    TIMSK2 |= (1 << TOIE2);

    // enable pcicr (for pwm_in)
    PCICR |= 0b00000100;    // turn on port d
    PCMSK2 |= 0b00000100;   // only use d2
    sei();
}


uint8_t num_overflows = 0;
uint8_t num_edges = 0;;
ISR(TIMER2_OVF_vect)
{
    num_overflows++;
    if(num_overflows > MAX_DOGS)
    {
        num_overflows = 0;
        has_pwm = (num_edges > 2);
        num_edges = 0;
    }

}


uint8_t timer_start = 0;
ISR(PCINT2_vect)
{
    num_edges++;

    if(!has_pwm) return;

    uint8_t pin_value = PIND & (1 << PD2);

    if(pin_value)
        timer_start = TCNT2;
    else
    {
        uint8_t t_diff = TCNT2 - timer_start;
        uint8_t pwm_des = get_desired_pwm(t_diff);
        if(pwm_des == 0)
            disable();
        else
        {
            enable();
            OCR1A = pwm_des;
        }
    }

}

void main(void)
{
    init_timers();
    for(;;)
    {
        if(!has_pwm)
            disable();
    }
}
