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

// exceeding in_min/max results in a zero output
#define IN_MIN 32        // minimum expected pwm in pulse length (ticks)
#define IN_MAX 80        // maximum expected pwm in pulse length (ticks)

// between the maximums and the deadbacks, the output is either full on or full off
// avoid switching when we don't need to
#define DEADBAND_MIN 36  // values less than this result in 0 output
#define DEADBAND_MAX 60  // values larger than this (but less than in_max) result in full on
#define OUT_MIN 0        // minimum duty cycle (out of 255)
#define OUT_MAX 0xff     // maximum duty cycle (out of 255)
#define MAX_DOGS 3       // number of timer 2 overflows between watchdog checks
#define MIN_EDGES 2      // minimum number of edges required to make watchdog happy
#define MAX_EDGES 12      // maximum number of edges required to make watchdog happy

#define NOTE_1 (92*2*4)
#define NOTE_2 (82*2*4)
#define NOTE_3 (73*2*4)
#define NOTE_4 (61*2*4)

// number of PWM_IN timer overflows
uint8_t dog_counter = 0;
// has valid PWM in recently
uint8_t has_pwm = 0;
// number of PWM in edges
uint8_t edges = 0;

uint8_t is_startup = 1;


uint8_t filtered_value = 0;

// map PWM pulse duration to output
uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
    if(x < DEADBAND_MIN) return 0;
    if(x > IN_MAX) return 0;
    if(x > DEADBAND_MAX) return OUT_MAX;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// remove output pin from timer 1
void safe_output_pin()
{
    TCCR1A = 0;
    TCCR1B = 0;
    PORTB = 0;
}

// set output pin to work with timer 1
void danger_output_pin()
{
    TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12)|(1 << WGM13);
}

void delay_ms(uint8_t ms) {
    uint16_t delay_count = F_CPU / 17500;
    volatile uint16_t i;

    while (ms != 0) {
        for (i=0; i != delay_count; i++);
        ms--;
    }
}


void note_delay()
{
    delay_ms(12);
}

void hobby_king()
{
    delay_ms(24);
    OCR1A = 0x10;
    danger_output_pin();
    ICR1 = NOTE_1;
    note_delay();
    ICR1 = NOTE_2;
    note_delay();
    ICR1 = NOTE_3;
    note_delay();
    ICR1 = NOTE_4;
    note_delay();
    note_delay();
    ICR1 = NOTE_3;
    note_delay();
    ICR1 = NOTE_4;
    note_delay();
    note_delay();
    ICR1 = 0xff;
    safe_output_pin();
}


// initialize all timer stuff
void init_pwm()
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

// value of the pwm in pin
uint8_t k = 0;
// last value of the pwm in pin
uint8_t last_k = 0;
// value of timer when pwm in goes high
uint8_t timer_start = 0; 
// current time - start time
uint8_t t_diff = 0;

// timer 2 overflow (watchdog)
ISR(TIMER2_OVF_vect)
{
    if(is_startup)
    {
        danger_output_pin();
        return;
    }
    // increment number of overflows
    dog_counter++;
    // if we've had enough, check edges
    if(dog_counter >= MAX_DOGS)
    {
        dog_counter = 0;
        if(edges > MAX_EDGES || edges < MIN_EDGES)
        {
            filtered_value = 0;
            has_pwm = 0;
        }
        else
            has_pwm = 1;

        edges = 0;
    }

    // if we failed, go to failsafe mode
    if(!has_pwm)
        safe_output_pin();
    else
        danger_output_pin();
}

// pin change interrupt
ISR(PCINT2_vect)
{
    if(is_startup)
    {
        TCCR1B |= (1 << CS10);
        return;
    }
    // value of pwm in pin
    k = PIND & (1 <<PD2);


    // if it has toggled, increment edges
    if(k ^ last_k)
        edges++;

    // timer values
    if(k)
        timer_start = TCNT2;
    else
    {
        if(timer_start > TCNT2)
        {
            t_diff = ((int16_t)(TCNT2) + 0xff) - timer_start;
        }
        else
            t_diff = TCNT2 - timer_start;
    }

    //compute pwm duty cycle desired
    uint8_t pwm_des = map(t_diff,DEADBAND_MIN,DEADBAND_MAX,OUT_MIN,OUT_MAX);


    filtered_value = pwm_des;


    // if it's zero, turn off PWM
    if(!pwm_des)
    {
        TCCR1B &= ~(1 << CS10);
        PORTB &= ~(1 << PB1);
        OCR1A = 0;
    }
    else if(has_pwm)
    {
        OCR1A = pwm_des;
        TCCR1B |= (1 << CS10);
    }
    else
        OCR1A = 0;

    last_k = k;
}

int main(void) {
    // initialize the direction of PORTD #6 to be an output
    //set_output(DDRB, LED);  
    DDRB |= (1 << DDB1)|(1 << DDB2);
    safe_output_pin();
    init_pwm();
    hobby_king();
    is_startup = 0;

    for(;;)
    {
    }
}
