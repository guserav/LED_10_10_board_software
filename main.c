#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 1000000
#include <util/delay.h>
#include <stdint.h>
#include <avr/eeprom.h>

// Read from ADC2: PB4; ADC3: PB3
// LED is on MOSI Pin: PB0
// Vmid = 4V / (10 + 1.6 ) * 1.6    ~0.552
// Vmid * 1024 / 1.1V (V_in * 1024 / V_ref)
#define V_MID 514
// 0.447 V max diff -10dBV Line level consumer
#define MAX_DIFF 416
#define CYCLES_LED_ON 100
#define PWM_1_P PA3
#define PWM_2_P PA1
#define PWM_3_P PA2
#define PWM_4_P PB2
#define LED PA6
#define PWM_1 OCR2B
#define PWM_2 OCR1B
#define PWM_3 OCR1A
#define PWM_4 OCR2A
#define RESOLUTION 0x03ff

void out_init() {
    // Write on Pin PB0
    DDRA |= (1 << LED);
    PUEA &= ~(1 << LED);
    PORTA &= ~(1 << LED);
    DDRA |= (1 << PWM_1_P);
    PUEA &= ~(1 << PWM_1_P);
    PORTA &= ~(1 << PWM_1_P);
    DDRA |= (1 << PWM_2_P);
    PUEA &= ~(1 << PWM_2_P);
    PORTA &= ~(1 << PWM_2_P);
    DDRA |= (1 << PWM_3_P);
    PUEA &= ~(1 << PWM_3_P);
    PORTA &= ~(1 << PWM_3_P);
    DDRB |= (1 << PWM_4_P);
    PUEB &= ~(1 << PWM_4_P);
    PORTB &= ~(1 << PWM_1_P);
}

void pwm_init() {
    TCCR1B = 0b001; // No prescaling and use complete Range of CLOCK
    TCCR2B = 0b001; // No prescaling and use complete Range of CLOCK
    TCCR1A = 3; // Fast_PWM 10 bit
    TCCR1B |= 1 << 3;
    TCCR2A = 3; // Fast_PWM 10 bit
    TCCR2B |= 1 << 3;

    TOCPMSA0 = 0b00100101;
    TOCPMSA1 = 0b10000000;
    TOCPMSA0 |= (1 << 1);

    TOCPMCOE = 0b10000111; // Enable outputs for desired pins;

    TCCR1A |= 0b10100000; // Set pin low on compare match and reset on BOTTOM
    TCCR2A |= 0b10100000; // Set pin low on compare match and reset on BOTTOM
}

void adc_init() {
    // Enable ADC and set prescaler for ADC clock to 2
    ADCSRA |= (1<<ADEN) | (1<<ADPS0);
}

#define led_sig_set(P) \
void led_sig_set_##P(uint8_t led, uint8_t x) { \
    if(x) \
        PORT##P |= (1<<led); \
    else \
        PORT##P &= ~(1<<led); \
}

led_sig_set(A)
led_sig_set(B)

int16_t readADC(uint8_t channel) {
    // Use Vcc
    ADMUXB = 0;
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADCW;
}

int main(void) {
    out_init();
    pwm_init();
    uint8_t flip = 0;
    uint16_t counter = 0;

    while (1) {
        _delay_ms(3000);
        led_sig_set_A(LED, flip);
        PWM_1 = counter;
        PWM_2 = counter;
        PWM_3 = counter;
        PWM_4 = counter;
        if(flip) {
            counter = 0;
        } else {
            counter = 1 << 6;

        }
        //counter += 1 << 5;
        //if(counter > RESOLUTION) {
            //counter = 0;
        //}
        flip = !flip;
    }
    return 0;
}
