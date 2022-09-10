
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
#define PWM_1 OCR2B
#define PWM_2 OCR1B
#define PWM_3 OCR1A
#define PWM_4 OCR2A
#define RESOLUTION 0x03ff
#define B(SHIFT) (1 << SHIFT)

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

    TOCPMCOE = B(TOCC0OE) | B(TOCC1OE) | B(TOCC2OE) | B(TOCC7OE); // Enable outputs for desired pins;

    TCCR1A |= B(COM1A1) | B(COM1B1);
    TCCR2A |= B(COM2A1) | B(COM2B1);
}

void uart_init() {
#define BAUD_RATE 250000
    PRR &= ~B(PRUSART1);
    UBRR1H = 0;
#if 2 != (F_CPU / (16 * BAUD_RATE) - 1)
#warning CPU frequency doesn't match the expected one
#endif
    UBRR1L = 2;
    // Consider changing U2Xn/U2X1 to 1 would change UBRR value
    UCSR1B = B(RXEN1);
    UCSR1C = B(USBS1)|B(UCSZ10)|B(UCSZ11); // 2-stop-bits, 8-bit word size; Implicit No parity, Asynchronous USART, no parity
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
    uart_init();
    int16_t dmx_frame = 0;
    uint8_t dmx_lock = 0;
    uint16_t uart_data = 0;
    uint8_t flip = 0;

    while (1) {
        led_sig_set_A(LED, 0);
        if(UCSR1A & B(RXC1)) {
            if(UCSR1A & B(FE1)) {
                dmx_lock = 1;
                dmx_frame = -2;
                uart_data = UDR1;
            } else {
                uart_data = UDR1;
                uart_data = (uart_data * uart_data) >> 6;
                if(dmx_lock) {
                    dmx_frame++;
                    if(dmx_frame == -1) {
                        // start code in uart_data
                    } else {
                        switch(dmx_frame) {
                            case 0:
                                PWM_1 = uart_data;
                                break;
                            case 1:
                                PWM_2 = uart_data;
                                break;
                            case 2:
                                PWM_3 = uart_data;
                                break;
                            case 3:
                                PWM_4 = uart_data;
                                break;
                            default:
                                led_sig_set_A(LED, 1);
                        }
                        if(dmx_frame > 512) {
                            dmx_lock = 0; // To many data packets in transmission
                        }
                    }
                } else {
                    dmx_frame = -3;
                }
            }
        }
    }
    return 0;
}
