//=============================================================================================
// INCLUDES
//=============================================================================================
#include <asf.h>
#include <io_stuff.h>
#include <string.h>

#include "conf_board.h"

//=============================================================================================
// DEFINES
//=============================================================================================
// Task MONITOR
#define TASK_MONITOR_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY (tskIDLE_PRIORITY)

// Task LED PLACA
#define TASK_LED_PLACA_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED_PLACA_STACK_PRIORITY (tskIDLE_PRIORITY)

// Task LED1
#define TASK_LED1_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)

// Task LED2
#define TASK_LED2_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED2_STACK_PRIORITY (tskIDLE_PRIORITY)

// Task LED3
#define TASK_LED3_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_LED3_STACK_PRIORITY (tskIDLE_PRIORITY)

// Task UARTRX
#define TASK_UARTRX_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_UARTRX_STACK_PRIORITY (tskIDLE_PRIORITY)
QueueHandle_t xQueueRx;

// Task EXECUTE
#define TASK_EXECUTE_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define TASK_EXECUTE_STACK_PRIORITY (tskIDLE_PRIORITY)
QueueHandle_t xQueueCommand;

// Semáforos dos botões
SemaphoreHandle_t xSemaphoreBut1;
SemaphoreHandle_t xSemaphoreBut2;
SemaphoreHandle_t xSemaphoreBut3;

//=============================================================================================
// PROTOTYPES
//=============================================================================================
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
void pin_toggle(Pio *pio, uint32_t mask);
static void USART1_init(void);
static void configure_console(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
void USART1_Handler(void);
uint32_t usart1_puts(uint8_t *pstring);
static void task_monitor(void *pvParameters);
static void task_led1(void *pvParameters);
static void task_led2(void *pvParameters);
static void task_uartRX(void *pvParameters);
static void task_execute(void *pvParameters);

//=============================================================================================
// RTOS FUNCTIONS
//=============================================================================================
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
    printf("Stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
    /* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
    for (;;) {
    }
}

extern void vApplicationIdleHook(void) {
    // Deixa o embarcado em Sleep Mode enquanto o processador estiver ocioso.
    pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

extern void vApplicationTickHook(void) {
}

extern void vApplicationMallocFailedHook(void) {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}

//=============================================================================================
// FUNCTIONS
//=============================================================================================
void pin_toggle(Pio *pio, uint32_t mask) {
    // Inverte o valor do pino dado.
    // Se estava ligado, desliga.
    // Se estava desligado, liga.
    if (pio_get_output_data_status(pio, mask))
        pio_clear(pio, mask);
    else
        pio_set(pio, mask);
}

static void USART1_init(void) {
    /* Configura USART1 Pinos */
    sysclk_enable_peripheral_clock(ID_PIOB);
    sysclk_enable_peripheral_clock(ID_PIOA);
    pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);   // RX
    pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21);  // TX
    MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

    /* Configura opções USART */
    const sam_usart_opt_t usart_settings = {
        .baudrate = 115200,
        .char_length = US_MR_CHRL_8_BIT,
        .parity_type = US_MR_PAR_NO,
        .stop_bits = US_MR_NBSTOP_1_BIT,
        .channel_mode = US_MR_CHMODE_NORMAL};

    /* Ativa Clock periferico USART0 */
    sysclk_enable_peripheral_clock(ID_USART1);

    stdio_serial_init(CONF_UART, &usart_settings);

    /* Enable the receiver and transmitter. */
    usart_enable_tx(USART1);
    usart_enable_rx(USART1);

    /* map printf to usart */
    ptr_put = (int (*)(void volatile *, char)) & usart_serial_putchar;
    ptr_get = (void (*)(void volatile *, char *)) & usart_serial_getchar;

    /* ativando interrupcao */
    usart_enable_interrupt(USART1, US_IER_RXRDY);
    NVIC_SetPriority(ID_USART1, 4);
    NVIC_EnableIRQ(ID_USART1);
}

static void configure_console(void) {
    const usart_serial_options_t uart_serial_options = {
        .baudrate = CONF_UART_BAUDRATE,
    // clang-format off
		#if (defined CONF_UART_CHAR_LENGTH)
				.charlength = CONF_UART_CHAR_LENGTH,
		#endif
				.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
				.stopbits = CONF_UART_STOP_BITS,
		#endif
			};

			/* Configure console UART. */
			stdio_serial_init(CONF_UART, &uart_serial_options);

		/* Specify that stdout should not be buffered. */
		#if defined(__GNUC__)
			setbuf(stdout, NULL);
		#else
			/* Already the case in IAR's Normal DLIB default configuration: printf()
									* emits one character at a time.
									*/
		#endif
    // clang-format on
}

//=============================================================================================
// HANDLERS / CALLBACKS
//=============================================================================================
void but1_callback(void) {
    // Libera o semáforo xSemaphoreBut1
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
    printf("But1 callback, xSemaphoreBut1 cleared\n");
}

void but2_callback(void) {
    // Libera o semáforo xSemaphoreBut2
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreBut2, &xHigherPriorityTaskWoken);
    printf("But2 callback, xSemaphoreBut2 cleared\n");
}

void but3_callback(void) {
    // Libera o semáforo xSemaphoreBut3
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreBut3, &xHigherPriorityTaskWoken);
    printf("But3 callback, xSemaphoreBut3 cleared\n");
}

void USART1_Handler(void) {
    uint32_t ret = usart_get_status(USART1);

    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    char c;

    // Verifica por qual motivo entrou na interrupção?
    // RXRDY ou TXRDY
    // Dados disponível para leitura
    if (ret & US_IER_RXRDY) {
        // Lê o caracter da USART1 e guarda em c.
        usart_serial_getchar(USART1, &c);

        // Envia o dado recebido para a fila xQueueRx.
        xQueueSendFromISR(xQueueRx, &c, &xHigherPriorityTaskWoken);
    } else if (ret & US_IER_TXRDY) {
    }
}

uint32_t usart1_puts(uint8_t *pstring) {
    uint32_t i;

    while (*(pstring + i))
        if (uart_is_tx_empty(USART1))
            usart_serial_putchar(USART1, *(pstring + i++));
}

//=============================================================================================
// TASKS
//=============================================================================================
static void task_monitor(void *pvParameters) {
    static portCHAR szList[256];
    UNUSED(pvParameters);

    /* Block for 3000ms. */
    const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;

    for (;;) {
        printf("--- Tasks: %u\n", (unsigned int)uxTaskGetNumberOfTasks());
        vTaskList((signed portCHAR *)szList);
        printf(szList);
        vTaskDelay(xDelay);
    }
}

static void task_led_placa(void *pvParameters) {
    // Cria o semáforo usado nessa task.
    xSemaphoreBut1 = xSemaphoreCreateBinary();

    // Verificando se o semáforo foi criado com sucesso.
    if (xSemaphoreBut1 == NULL) printf("Falha ao criar xSemaphoreBut1\n");

    // Configurando o LED 0 (LED da placa) e iniciando ele como desligado (0).
    setup_led(0, 0);

    // Configurando o BUTTON1 (da placa OLED1).
    setup_but(1, PIO_PULLUP | PIO_DEBOUNCE, PIO_IT_RISE_EDGE, but1_callback);

    while (1) {
        if (xSemaphoreTake(xSemaphoreBut1, (TickType_t)500) == pdTRUE) {
            // Quando o semáforo for liberado, trocar o sinal do LED PLACA.
            pin_toggle(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
        }
    }
}

static void task_led1(void *pvParameters) {
    // Configurando o LED1 (LED da placa OLED1) e iniciando ele como desligado (0).
    setup_led(1, 0);

    // Delay entre as três piscadas.
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

    // Delay entre a troca do sinal atual.
    const TickType_t xDelayLed = 100 / portTICK_PERIOD_MS;

    for (;;) {
        pin_toggle(LED1_PIO, LED1_IDX_MASK);
        vTaskDelay(xDelayLed);
        pin_toggle(LED1_PIO, LED1_IDX_MASK);
        vTaskDelay(xDelayLed);

        pin_toggle(LED1_PIO, LED1_IDX_MASK);
        vTaskDelay(xDelayLed);
        pin_toggle(LED1_PIO, LED1_IDX_MASK);
        vTaskDelay(xDelayLed);

        pin_toggle(LED1_PIO, LED1_IDX_MASK);
        vTaskDelay(xDelayLed);
        pin_toggle(LED1_PIO, LED1_IDX_MASK);
        vTaskDelay(xDelayLed);

        vTaskDelay(xDelay);
    }
}

static void task_led2(void *pvParameters) {
    // Cria o semáforo usado nessa task.
    xSemaphoreBut2 = xSemaphoreCreateBinary();

    // Verificando se o semáforo foi criado com sucesso.
    if (xSemaphoreBut2 == NULL) printf("Falha ao criar xSemaphoreBut2\n");

    // Configurando o LED2 (LED da placa OLED1) e iniciando ele como desligado (0).
    setup_led(2, 0);

    // Configurando o BUTTON2 (da placa OLED1).
    setup_but(2, PIO_PULLUP | PIO_DEBOUNCE, PIO_IT_RISE_EDGE, but2_callback);

    while (1) {
        if (xSemaphoreTake(xSemaphoreBut2, (TickType_t)500) == pdTRUE) {
            // Quando o semáforo for liberado, trocar o sinal do LED2.
            pin_toggle(LED2_PIO, LED2_IDX_MASK);
        }
    }
}

static void task_led3(void *pvParameters) {
    // Cria o semáforo usado nessa task.
    xSemaphoreBut3 = xSemaphoreCreateBinary();

    // Verificando se o semáforo foi criado com sucesso.
    if (xSemaphoreBut3 == NULL) printf("Falha ao criar xSemaphoreBut3\n");

    // Configurando o LED3 (LED da placa OLED1) e iniciando ele como desligado (0).
    setup_led(3, 0);

    // Configurando o BUTTON1 (da placa OLED1).
    setup_but(3, PIO_PULLUP | PIO_DEBOUNCE, PIO_IT_RISE_EDGE, but3_callback);

    while (1) {
        if (xSemaphoreTake(xSemaphoreBut3, (TickType_t)500) == pdTRUE) {
            // Quando o semáforo for liberado, trocar o sinal do LED3.
            pin_toggle(LED3_PIO, LED3_IDX_MASK);
        }
    }
}

static void task_uartRX(void *pvParameters) {
    // Variável que guarda a mensagem completa.
    char msgBuffer[64] = {0};

    // Variável que guarda o caracter recebido em RX.
    char msgChar;

    // Contador usado para guardar cada caracter no array da mensagem.
    int counter = 0;

    // Criando a fila dessa task, fila de caracteres.
    xQueueRx = xQueueCreate(32, sizeof(char));

    while (1) {
        if (xQueueReceive(xQueueRx, &msgChar, (TickType_t)500)) {
            // Printando o caracter recebido no terminal.
            printf("Recebeu: %c\n", msgChar);

            // Se não for o caracter '\n' ainda não é o fim da string.
            if (msgChar != '\n') {
                msgBuffer[counter] = msgChar;
                counter++;
            } else {
                // Foi encontrado o caracter '\n'. Devemos adicionar o '0' para marcar
                // o fim da mensagem.
                msgBuffer[counter] = 0;

                // Colocar a mensagem recebida na fila de comandos a serem executados.
                xQueueSend(xQueueCommand, &msgBuffer, 0);

                // Resetar o contador para receber outra mensagem.
                counter = 0;
            }
        }
    }
}

static void task_execute(void *pvParameters) {
    // Variável que guarda a mensagem completa.
    char msgBuffer[64];

    // Criando a fila dessa task, fila de comandos.
    // Cada comando por ter até 64 caracteres, incluindo o '0'.
    // O número máximo de comandos é 5, mas pode ser qualquer número, limitado apenas pela
    // memória do embarcado.
    xQueueCommand = xQueueCreate(5, sizeof(char[64]));

    while (1) {
        if (xQueueReceive(xQueueCommand, &msgBuffer, (TickType_t)500)) {
            // Printando o comando recebido no terminal.
            printf("Comando: %s\n", msgBuffer);

            // Verificando qual o comando recebido e realizando as tarefas de acordo.
            if (strcmp(msgBuffer, "led placa toggle") == 0) {
                pin_toggle(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 1 toggle") == 0) {
                pin_toggle(LED1_PIO, LED1_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 2 toggle") == 0) {
                pin_toggle(LED2_PIO, LED2_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 3 toggle") == 0) {
                pin_toggle(LED3_PIO, LED3_IDX_MASK);
            } else if (strcmp(msgBuffer, "led placa on") == 0) {
                pio_clear(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
            } else if (strcmp(msgBuffer, "led placa off") == 0) {
                pio_set(LED_PLACA_PIO, LED_PLACA_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 1 on") == 0) {
                pio_clear(LED1_PIO, LED1_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 1 off") == 0) {
                pio_set(LED1_PIO, LED1_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 2 on") == 0) {
                pio_clear(LED2_PIO, LED2_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 2 off") == 0) {
                pio_set(LED2_PIO, LED2_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 3 on") == 0) {
                pio_clear(LED3_PIO, LED3_IDX_MASK);
            } else if (strcmp(msgBuffer, "led 3 off") == 0) {
                pio_set(LED3_PIO, LED3_IDX_MASK);
            }
        }
    }
}

//=============================================================================================
// MAIN
//=============================================================================================
int main(void) {
    /* Initialize the SAM system */
    sysclk_init();
    board_init();

    /* Initialize the console uart */
    configure_console();
    USART1_init();

    /* Output demo information. */
    printf("-- FreeRTOS Example --\r\n");
    printf("-- %s\r\n", BOARD_NAME);
    printf("-- Compiled: %s %s --\r\n", __DATE__, __TIME__);

    /* Create task to monitor processor activity */
    if (xTaskCreate(task_monitor, "MONITOR", TASK_MONITOR_STACK_SIZE, NULL, TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_monitor\r\n");
    }

    // Criando a task do LED PLACA.
    if (xTaskCreate(task_led_placa, "LED_PLACA", TASK_LED_PLACA_STACK_SIZE, NULL, TASK_LED_PLACA_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_led_placa\r\n");
    }

    // Criando a task do LED 1.
    if (xTaskCreate(task_led1, "LED1", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_led1\r\n");
    }

    // Criando a task do LED 2.
    if (xTaskCreate(task_led2, "LED2", TASK_LED2_STACK_SIZE, NULL, TASK_LED2_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_led2\r\n");
    }

    // Criando a task do LED 3.
    if (xTaskCreate(task_led3, "LED3", TASK_LED3_STACK_SIZE, NULL, TASK_LED3_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_led3\r\n");
    }

    // Criando a task do UART RX.
    if (xTaskCreate(task_uartRX, "UART-RX", TASK_UARTRX_STACK_SIZE, NULL, TASK_UARTRX_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_uartRX\r\n");
    }

    // Criando a task que executa comandos, EXECUTE.
    if (xTaskCreate(task_execute, "EXECUTE", TASK_EXECUTE_STACK_SIZE, NULL, TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Falha ao criar task_execute\r\n");
    }

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle task. */
    return 0;
}
