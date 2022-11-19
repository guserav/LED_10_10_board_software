
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 12000000
#include <util/delay.h>
#include <stdint.h>
#include <avr/eeprom.h>

#include "dip.h"

#define TIME int16_t

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

#define STATE_DMX 0
#define STATE_MANUAL 1

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

volatile TIME tot_overflow_200_micros;
// initialize timer, interrupt and variable
void timer_100micros_init() {
    // set up timer with prescaler = 8
    // Normal Timer operations, 0 to OCRA
    TCCR0B = B(CS01);
    TCCR0A = 0;

    // initialize counter
    TCNT0 = 0;

    // enable overflow interrupt
    TIMSK0 |= (1 << TOIE0);

    sei();
    tot_overflow_200_micros = 0;
}

ISR(TIMER0_OVF_vect) {
    // keep a track of number of overflows
    tot_overflow_200_micros++;
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
    if(x > RESOLUTION) x = RESOLUTION - 1; \
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
    int8_t state;
    int16_t dmx_frame;
    uint8_t dmx_lock;
    uint8_t flip;
    uint16_t adc_accumulator;
    uint8_t adc_count;
    uint16_t convert_result;
    uint8_t adc_count_limit;
    uint16_t pwm_limit;
    uint8_t pressed_down;
    TIME pressed_time;
} state = {
    .state = STATE_DMX,
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
            } else if(state.state != STATE_DMX) {
                // do nothing
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


struct anim {
    int16_t from;
    int16_t to;
    TIME t_from;
    TIME t_to;
    uint8_t enabled;
} anim = {
    .from = 0,
    .to = 0,
    .t_from = 0,
    .t_to = 0,
    .enabled = 0
};

void set_current_animation_value() {
    if(anim.enabled) {
        TIME diff = anim.t_to - tot_overflow_200_micros;
        uint16_t set = 0;
        if(diff < 0) {
            anim.enabled = 0;
            set = anim.to;
        } else {
            int32_t temp = (tot_overflow_200_micros - anim.t_from);
            temp = temp * (anim.to - anim.from);
            temp = temp / (anim.t_to - anim.t_from);
            set = anim.from + temp;
        }
        set = anim.to;
        pwm_set_1(set);
        pwm_set_2(set);
        pwm_set_3(set);
        pwm_set_4(set);
    }
}

void animate(int16_t from, int16_t to, TIME micros) {
    anim.from = from;
    anim.to = to;
    anim.t_from = tot_overflow_200_micros;
    anim.t_to = tot_overflow_200_micros + micros;
    anim.enabled = 1;
}

uint16_t manual_value = 0;
void handle_btn_long_press() {
    switch(state.state) {
        case STATE_MANUAL:
            state.state = STATE_DMX;
            break;
        case STATE_DMX:
        default:
            state.state = STATE_MANUAL;
            manual_value = 0;
            animate(0, manual_value, 5000);
    }
}

void handle_btn_short_press() {
    if(state.state == STATE_MANUAL) {
        int16_t new = 0;
        if(manual_value == 0) {
            new = 1;
        } else if (manual_value > RESOLUTION || RESOLUTION == manual_value) {
            new = 0;
        } else {
            new = manual_value * 4;
        }
        animate(manual_value, new, 5000);
        manual_value = new;
    }
}

#define OLD_PWM_LIMITS_N 16
uint16_t old_pwm_limits[OLD_PWM_LIMITS_N] = {0};
uint16_t old_pwm_limits_p = 0;
uint16_t old_pwm_limits_acc = 0;
#if RESOLUTION * OLD_PWM_LIMITS_N > 0xffff
#error "Accumulator to small for pwm_limits"
#endif

void handle_btn_and_temp() {
    if(state.convert_result < 16) {  // Low enough value to consider ADC pulled to Ground
        if(state.pressed_down) {
            if (state.pressed_down == 1) {
                state.pressed_down = 2;
            } else if(state.pressed_down == 2 && ((tot_overflow_200_micros - state.pressed_time) > 5000)) {
                state.pressed_down = 3;
                handle_btn_long_press();
            }
        } else {
            state.pressed_time = tot_overflow_200_micros;
            state.pressed_down = 1;
        }
    } else {
        if(state.pressed_down == 2) {
            handle_btn_short_press();
        }
        if(state.pressed_down) {
            state.pressed_down = 0;
            return;
        }
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
    timer_100micros_init();

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
                            UDR1 = manual_value >> 8;
                            break;
                        case 5:
                            UCSR1B &= ~B(TXEN1);
                    }
                }

                nonce = ADCW;
                ADCSRA |= B(ADSC);
            } else {
                set_current_animation_value();
            }
        }
    }
    return 0;
}
