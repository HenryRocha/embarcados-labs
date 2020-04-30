#ifndef IO_STUFF
#define IO_STUFF

// LED PLACA
#define LED_PLACA_PIO_ID ID_PIOC
#define LED_PLACA_PIO PIOC
#define LED_PLACA_PIN 8
#define LED_PLACA_IDX_MASK (1 << LED_PLACA_PIN)

// LED1 OLED1
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_IDX 0
#define LED1_IDX_MASK (1 << LED1_IDX)

// LED2 OLED1
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_IDX 30
#define LED2_IDX_MASK (1 << LED2_IDX)

// LED3 OLED1
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_IDX 2
#define LED3_IDX_MASK (1 << LED3_IDX)

// BUT1 OLED1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID 16
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

// BUT2 OLED1
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1 << BUT2_PIO_IDX)

// BUT3 OLED1
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1 << BUT3_PIO_IDX)

void setup_led(short int led, short int initialState) {
    switch (led) {
        case 0:
            // LED PLACA
            pmc_enable_periph_clk(LED_PLACA_PIO_ID);
            pio_configure(LED_PLACA_PIO, PIO_OUTPUT_0, LED_PLACA_IDX_MASK, PIO_DEFAULT);

            if (initialState) {
                pio_clear(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
            } else {
                pio_set(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
            }

            break;

        case 1:
            // LED1 OLED
            pmc_enable_periph_clk(LED1_PIO_ID);
            pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);

            if (initialState) {
                pio_clear(LED1_PIO, LED1_IDX_MASK);
            } else {
                pio_set(LED1_PIO, LED1_IDX_MASK);
            }

            break;

        case 2:
            // LED2 OLED
            pmc_enable_periph_clk(LED2_PIO_ID);
            pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);

            if (initialState) {
                pio_clear(LED2_PIO, LED2_IDX_MASK);
            } else {
                pio_set(LED2_PIO, LED2_IDX_MASK);
            }

            break;

        case 3:
            // LED3 OLED
            pmc_enable_periph_clk(LED3_PIO_ID);
            pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_IDX_MASK, PIO_DEFAULT);

            if (initialState) {
                pio_clear(LED3_PIO, LED3_IDX_MASK);
            } else {
                pio_set(LED3_PIO, LED3_IDX_MASK);
            }

            break;

        default:
            printf("LED %hi não encontrado\n", led);
            break;
    }
}

void setup_but(short int but, unsigned int pinAttributes, unsigned int edge, void (*callback)(unsigned int, unsigned int)) {
    switch (but) {
        case 1:
            // BUT1 OLED
            pmc_enable_periph_clk(BUT1_PIO_ID);
            pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, pinAttributes);
            pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, edge, callback);
            pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
            NVIC_EnableIRQ(BUT1_PIO_ID);
            NVIC_SetPriority(BUT1_PIO_ID, 4);
            break;

        case 2:
            // BUT2 OLED
            pmc_enable_periph_clk(BUT2_PIO_ID);
            pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, pinAttributes);
            pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, edge, callback);
            pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
            NVIC_EnableIRQ(BUT2_PIO_ID);
            NVIC_SetPriority(BUT2_PIO_ID, 4);
            break;

        case 3:
            // BUT3 OLED
            pmc_enable_periph_clk(BUT3_PIO_ID);
            pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, pinAttributes);
            pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, edge, callback);
            pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
            NVIC_EnableIRQ(BUT3_PIO_ID);
            NVIC_SetPriority(BUT3_PIO_ID, 4);
            break;

        default:
            printf("BUT %hi não encontrado\n", but);
            break;
    }
}

#endif