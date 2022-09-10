
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 12000000
#include <util/delay.h>
#include <stdint.h>
#include <avr/eeprom.h>

#include "dip.h"

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
#define RESOLUTION 0x03ff
#define B(SHIFT) (1 << SHIFT)

#define ADC_READ_BTN_TEMP 0
#define ADC_READ_DIP_1 5
#define ADC_READ_DIP_2 7
#define ADC_READ_NONE 14

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
    TCCR1B = B(CS10); // No prescaling and use complete Range of CLOCK
    TCCR2B = B(CS20); // No prescaling and use complete Range of CLOCK
    TCCR1A = B(WGM10) | B(WGM11); // Fast_PWM 10 bit
    TCCR1B |= B(WGM12);
    TCCR2A = B(WGM20) | B(WGM21); // Fast_PWM 10 bit
    TCCR2B |= B(WGM22);

    TOCPMSA0 = B(TOCC0S0) | B(TOCC1S0) | B(TOCC2S1);
    TOCPMSA1 = B(TOCC7S1);

    //TOCPMCOE = B(TOCC0OE) | B(TOCC1OE) | B(TOCC2OE) | B(TOCC7OE); // Enable outputs for desired pins;
    TOCPMCOE = 0; // Disable pin output for the start

    TCCR1A |= B(COM1A1) | B(COM1B1);
    TCCR2A |= B(COM2A1) | B(COM2B1);
}

volatile uint16_t tot_overflow_5_ms;
// initialize timer, interrupt and variable
void timer_5ms_init() {
    // set up timer with prescaler = 8
    TCCR0B = (1 << CS02);
    // Normal Timer operations, 0 to MAX
    TCCR0A = 0;

    // initialize counter
    TCNT0 = 0;

    // enable overflow interrupt
    TIMSK0 |= (1 << TOIE0);

    sei();
    tot_overflow_5_ms = 0;
}

ISR(TIMER0_OVF_vect) {
    // keep a track of number of overflows
    tot_overflow_5_ms++;
}

void uart_init() {
#define BAUD_RATE 250000
    PRR &= ~B(PRUSART1);
    UBRR1H = 0;
#if 2 != (F_CPU / (16 * BAUD_RATE) - 1)
#warning "CPU frequency doesn't match the expected one"
#endif
    UBRR1L = 2;
    // Consider changing U2Xn/U2X1 to 1 would change UBRR value
    UCSR1B = B(RXEN1);
    UCSR1C = B(USBS1)|B(UCSZ10)|B(UCSZ11); // 2-stop-bits, 8-bit word size; Implicit No parity, Asynchronous USART, no parity
    UCSR1C = B(UCSZ10)|B(UCSZ11); // 1-stop-bits, 8-bit word size; Implicit No parity, Asynchronous USART, no parity
}

void adc_init() {
    // Enable ADC and set prescaler for ADC clock to 64
#if F_CPU/64 > 200000
#warning ADC has a to high clock rate
#elif F_CPU/64 < 100000
#warning ADC could be used a bit faster
#endif
    ADCSRA |= B(ADEN) | B(ADPS2) | B(ADPS1);
    // Use Vcc
    ADMUXB = 0;
    ADMUXA = ADC_READ_NONE;
}

#define led_sig_set(P) \
void led_sig_set_##P(uint8_t led, uint8_t x) { \
    if(x) \
        PORT##P |= (1<<led); \
    else \
        PORT##P &= ~(1<<led); \
}

led_sig_set(A);
led_sig_set(B);

#define pwm_set_gen(X, R, L) \
void pwm_set_##X(uint16_t x) { \
    if(x > state.pwm_limit) x = state.pwm_limit; \
    if(x != 0) { \
        R = x; \
        TOCPMCOE |= B(L); \
    } else { \
        TOCPMCOE &= ~B(L); \
    } \
} \
void pwm_limit_##X (uint16_t x) { \
    if(x < R) { \
        R = x; \
    } \
}

int16_t read_ADC(uint8_t channel) {
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADCW;
}

struct state {
    int16_t dmx_frame;
    uint8_t dmx_lock;
    uint8_t flip;
    uint16_t adc_accumulator;
    uint8_t adc_count;
    uint16_t convert_result;
    uint8_t adc_count_limit;
    uint16_t pwm_limit;
    uint8_t pressed_down;
    uint16_t pressed_time;
} state = {
    .dmx_frame = 0,
    .dmx_lock = 0,
    .flip = 0,
    .adc_accumulator = 0,
    .adc_count = 0,
    .convert_result = 0,
    .adc_count_limit = 16,
    .pwm_limit = 0xffff
};

pwm_set_gen(1, OCR2B, TOCC0OE);
pwm_set_gen(2, OCR1B, TOCC2OE);
pwm_set_gen(3, OCR1A, TOCC1OE);
pwm_set_gen(4, OCR2A, TOCC7OE);

void process_uart_dmx() {
    uint16_t uart_data;
    if(UCSR1A & B(FE1)) {
        state.dmx_lock = 1;
        state.dmx_frame = -2;
        uart_data = UDR1;
    } else {
        uart_data = UDR1;
        uart_data = (uart_data * uart_data) >> 6;
        if(state.dmx_lock) {
            state.dmx_frame++;
            if(state.dmx_frame == -1) {
                // start code in uart_data
            } else {
                switch(state.dmx_frame) {
                    case 0:
                        pwm_set_1(uart_data);
                        break;
                    case 1:
                        pwm_set_2(uart_data);
                        break;
                    case 2:
                        pwm_set_3(uart_data);
                        break;
                    case 3:
                        pwm_set_4(uart_data);
                        break;
                    default:
                        //led_sig_set_A(LED, 1);
                }
                if(state.dmx_frame > 512) {
                    state.dmx_lock = 0; // To many data packets in transmission
                }
            }
        } else {
            state.dmx_frame = -3;
        }
    }
}

void handle_btn_long_press() {

}

void handle_btn_short_press() {

}

#define OLD_PWM_LIMITS_N 16
uint16_t old_pwm_limits[OLD_PWM_LIMITS_N] = {0};
uint16_t old_pwm_limits_p = 0;
uint16_t old_pwm_limits_acc = 0;
#if RESOLUTION * OLD_PWM_LIMITS_N > 0xffff
#error "Accumulator to small for pwm_limits"
#endif

void handle_btn_and_temp() {
    if(state.convert_result < 16) {
        if(state.pressed_down) {
            state.pressed_down = 2;
        } else {
            state.pressed_time = tot_overflow_5_ms;
            state.pressed_down = 1;
        }
        if(state.pressed_down == 2 && (tot_overflow_5_ms - state.pressed_time > 1000/5)) {
            state.pressed_down = 3;
            handle_btn_long_press();
        }
    } else {
        if(state.pressed_down == 2) {
            handle_btn_short_press();

        }
        state.pressed_down = 0;
        // LIMIT = y * 1024 / (1 + y)
        // y = 0.4721 = 40 C
#define TEMP_UPER_LIMIT 329
        // y = 0.1236 = 70 C
#define TEMP_LOWER_LIMIT 113
//FOR DEBUGGING:
//#define TEMP_UPER_LIMIT 447
//#define TEMP_LOWER_LIMIT 233
        uint16_t pwm_limit;
        if(state.convert_result > TEMP_UPER_LIMIT) {
            pwm_limit = RESOLUTION;
            //state.pwm_limit = 0x01 << 2;
        } else {
            if(state.convert_result < TEMP_LOWER_LIMIT) {
                pwm_limit = 0;
            } else {
                uint32_t x = state.convert_result - TEMP_LOWER_LIMIT;
                pwm_limit = (x * RESOLUTION) / (TEMP_UPER_LIMIT - TEMP_LOWER_LIMIT);;
            }
        }
        old_pwm_limits_acc -= old_pwm_limits[old_pwm_limits_p];
        old_pwm_limits[old_pwm_limits_p] = pwm_limit;
        old_pwm_limits_acc += old_pwm_limits[old_pwm_limits_p++];
        if(old_pwm_limits_p >= OLD_PWM_LIMITS_N) {
            old_pwm_limits_p = 0;
        }
        state.pwm_limit = old_pwm_limits_acc / OLD_PWM_LIMITS_N;

        pwm_limit_1(state.pwm_limit);
        pwm_limit_2(state.pwm_limit);
        pwm_limit_3(state.pwm_limit);
        pwm_limit_4(state.pwm_limit);
    }
}

int main(void) {
    out_init();
    pwm_init();
    uart_init();
    adc_init();
    timer_5ms_init();

    uint16_t nonce = 0;

    while (1) {
        led_sig_set_A(LED, 0);
        if(UCSR1A & B(RXC1)) {
            process_uart_dmx();
        } else { // Nothing to read from uart doing something else
            if(!(ADCSRA & B(ADSC))) {
                //led_sig_set_A(LED, 1);
                if(state.adc_count != 0) {
                    state.adc_accumulator += ADCW;
                } else {
                    nonce = ADCW;
                }
                state.adc_count++;
                if(state.adc_count > state.adc_count_limit) {
                    if(state.adc_count_limit > 0) {
                        state.convert_result = state.adc_accumulator / (state.adc_count - 1);
                    } else {
                        state.convert_result = nonce;
                    }
                    state.adc_count = 0;
                    state.adc_accumulator = 0;
                    switch(ADMUXA) {
                        case ADC_READ_DIP_2:
                            ADMUXA = ADC_READ_DIP_1;
                            state.adc_count_limit = 16;
                            break;
                        case ADC_READ_DIP_1:
                            ADMUXA = ADC_READ_BTN_TEMP;
                            state.adc_count_limit = 0;
                            break;
                        case ADC_READ_BTN_TEMP:
                            handle_btn_and_temp();
                            ADMUXA = ADC_READ_NONE;
                            state.adc_count_limit = 5;
                            break;
                        case ADC_READ_NONE:
                        default:
                            ADMUXA = ADC_READ_DIP_2;
                            state.adc_count_limit = 16;
                    }
                } else if(ADMUXA == ADC_READ_NONE) {
                    switch(state.adc_count) {
                        case 1:
                            UCSR1B |= B(TXEN1);
                            break;
                        case 2:
                            UDR1 = state.convert_result >> 2;
                            break;
                        case 3:
                            //UDR1 = (state.convert_result & 0x03) << 3;
                            UDR1 = state.pwm_limit >> 2;
                            break;
                        case 4:
                            UDR1 = tot_overflow_5_ms >> 8;
                            break;
                        case 5:
                            UCSR1B &= ~B(TXEN1);
                    }
                }

                nonce = ADCW;
                ADCSRA |= B(ADSC);
            }
        }
    }
    return 0;
}
