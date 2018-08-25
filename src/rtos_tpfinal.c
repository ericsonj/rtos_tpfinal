/* Copyright 2018, Ericson Joseph
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Date: 2018-08-24
 */

/*==================[inclusions]=============================================*/

//#include "rtos_tpfinal.h"   // <= own header (optional)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "sapi.h" // <= sAPI header
#include "semphr.h"
#include "task.h"

/*==================[macros and definitions]=================================*/

#define FALLING 0
#define RASING 1
#define DEBOUNCE_TIME 50
#define TEC1_TEC2 0
#define TEC2_TEC1 1

typedef enum {
    BUTTON_UP,
    BUTTON_FALLING,
    BUTTON_DOWN,
    BUTTON_RASING
} buttonState_t;

typedef enum {
    IDLE,
    WAIT_TIC2_FALLIN,
    WAIT_TIC2_RASING,
    WAIT_TIC1_FALLIN,
    WAIT_TIC1_RASING,
    MEDIR,
} edgeState;

typedef struct {
    gpioMap_t tec;
    uint8_t type;
    tick_t time;
} gpioInterrupt_t;

typedef struct {
    QueueHandle_t queue;
    SemaphoreHandle_t semf;
    buttonState_t state;
    gpioMap_t led;
    TickType_t tmpTicks;
    TickType_t ledTicks;
    TickType_t defaultTicks;
} gpioQueue_t;

typedef struct {
    char *bx;
    char *by;
    TickType_t time;
} serialInfo;

gpioQueue_t buttons[2];

QueueHandle_t totalQueue;
edgeState eState;

QueueHandle_t serialQueue;

TickType_t startTick;
TickType_t endTick;

SemaphoreHandle_t medicionSem;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void initISR() {

    // TEC1
    /*Seteo la interrupción para el flanco descendente
     *                channel, GPIOx, [y]    <- no es la config del pin, sino el
     * nombre interno de la señal
     *                      |  |      |
     *                      |  |      |    */
    Chip_SCU_GPIOIntPinSel(0, 0, 4);
    // Borra el pending de la IRQ
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,
                               PININTCH(0)); // INT0 (canal 0 -> hanlder GPIO0)
    // Selecciona activo por flanco
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(0)); // INT0
    // Selecciona activo por flanco descendente
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(0)); // INT0
    // Selecciona activo por flanco acendente
    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(0)); // INT0

    // TEC2
    /*Seteo la interrupción para el flanco descendente
     *                channel, GPIOx, [y]    <- no es la config del pin, sino el
     * nombre interno de la señal
     *                      |  |      |
     *                      |  |      |    */
    Chip_SCU_GPIOIntPinSel(1, 0, 8);
    // Borra el pending de la IRQ
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,
                               PININTCH(1)); // INT1  (canal 1 -> hanlder GPIO1)
    // Selecciona activo por flanco
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1)); // INT1
    // Selecciona activo por flanco descendente
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(1)); // INT1
    // Selecciona activo por flanco descendente
    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(1)); // INT1

    // TEC1
    // IMPORTANTE SET PRIORITY
    NVIC_SetPriority(PIN_INT0_IRQn, 0xFFFF);
    NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
    NVIC_EnableIRQ(PIN_INT0_IRQn);

    // TEC2
    // IMPORTANTE SET PRIORITY
    NVIC_SetPriority(PIN_INT1_IRQn, 0xFFFF);
    NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
    NVIC_EnableIRQ(PIN_INT1_IRQn);
}

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void ledTask(void *taskParmPtr);

void debounceTask(void *taskParmPtr);

void edgeTask(void *taskParmPtr);

void serialDisplayTask(void *taskParmPtr);

void ledMedicionTask(void *taskParmPtr);

/*==================[external functions definition]==========================*/

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void) {

    /* ------------- INICIALIZACIONES ------------- */

    /* Inicializar la placa */
    boardConfig();

    initISR();

    startTick = 0;
    endTick   = 0;

    totalQueue = xQueueCreate(2, sizeof(gpioInterrupt_t));
    eState     = IDLE;

    serialQueue = xQueueCreate(10, sizeof(serialInfo));

    medicionSem = xSemaphoreCreateBinary();

    uint8_t i = 0;
    for (i = 0; i < 2; i++) {
        buttons[i].queue        = xQueueCreate(2, sizeof(gpioInterrupt_t));
        buttons[i].semf         = xSemaphoreCreateBinary();
        buttons[i].state        = BUTTON_UP;
        buttons[i].tmpTicks     = 0;
        buttons[i].ledTicks     = 0;
        buttons[i].defaultTicks = 500;
    }
    buttons[0].led = LED1;
    buttons[1].led = LED2;

    i = 0;
    for (i = 0; i < 2; i++) {

        xTaskCreate(debounceTask,                 // FUNCTION
                    (const char *)"debounceTask", // TASK NAME
                    configMINIMAL_STACK_SIZE,     // TASK STACK
                    (void *)&buttons[i],          // TASK ARGS
                    tskIDLE_PRIORITY + 3,         // TASK PRIORITY
                    NULL                          // SYSTEM TASK POINTER
                    );

        xTaskCreate(ledTask,                  // FUNCTION
                    (const char *)"ledTask",  // TASK NAME
                    configMINIMAL_STACK_SIZE, // TASK STACK
                    (void *)&buttons[i],      // TASK ARGS
                    tskIDLE_PRIORITY + 1,     // TASK PRIORITY
                    NULL                      // SYSTEM TASK POINTER
                    );
    }

    xTaskCreate(edgeTask,                 // FUNCTION
                (const char *)"edgeTask", // TASK NAME
                configMINIMAL_STACK_SIZE, // TASK STACK
                (void *)&buttons[i],      // TASK ARGS
                tskIDLE_PRIORITY + 1,     // TASK PRIORITY
                NULL                      // SYSTEM TASK POINTER
                );

    xTaskCreate(serialDisplayTask,                 // FUNCTION
                (const char *)"serialDisplayTask", // TASK NAME
                configMINIMAL_STACK_SIZE * 2,      // TASK STACK
                (void *)&buttons[i],               // TASK ARGS
                tskIDLE_PRIORITY + 1,              // TASK PRIORITY
                NULL                               // SYSTEM TASK POINTER
                );

    xTaskCreate(ledMedicionTask,                 // FUNCTION
                (const char *)"ledMedicionTask", // TASK NAME
                configMINIMAL_STACK_SIZE * 2,      // TASK STACK
                (void *)&buttons[i],               // TASK ARGS
                tskIDLE_PRIORITY + 1,              // TASK PRIORITY
                NULL                               // SYSTEM TASK POINTER
                );

    // Iniciar scheduler
    vTaskStartScheduler();

    /* ------------- REPETIR POR SIEMPRE ------------- */
    while (TRUE) {
    }

    /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
       por ningun S.O. */
    return 0;
}

void serialDisplayTask(void *taskParmPtr) {
    while (TRUE) {
        serialInfo info;
        BaseType_t resp = xQueueReceive(serialQueue, &info, portMAX_DELAY);
        printf("%s:%s:%d\r\n", info.bx, info.by ,info.time);
    }
}

void ledMedicionTask(void *taskParmPtr){
    while (TRUE) {
        BaseType_t resp = xSemaphoreTake(medicionSem, portMAX_DELAY);
        if (resp = pdTRUE) {
            gpioWrite(LED3, ON);
            vTaskDelay(200 / portTICK_RATE_MS);
            gpioWrite(LED3, OFF);
        }
    }
}

void ledTask(void *taskParmPtr) {
    gpioQueue_t *aux = (gpioQueue_t *)taskParmPtr;
    while (TRUE) {
        BaseType_t resp = xSemaphoreTake(aux->semf, portMAX_DELAY);
        if (resp = pdTRUE) {
            gpioWrite(aux->led, ON);
            vTaskDelay(200 / portTICK_RATE_MS);
            gpioWrite(aux->led, OFF);
        }
    }
}

void edgeTask(void *taskParmPtr) {
    uint8_t order = TEC1_TEC2;
    while (TRUE) {
        gpioInterrupt_t action;
        BaseType_t resp = xQueueReceive(totalQueue, &action, 40);
        switch (eState) {
        case IDLE:
            if (resp == pdTRUE && action.tec == TEC1 &&
                action.type == FALLING) {
                startTick = action.time;
                eState    = WAIT_TIC2_FALLIN;
            }else if(resp == pdTRUE && action.tec == TEC1 &&
                     action.type == RASING){
                startTick = action.time;
                eState    = WAIT_TIC2_RASING;
            }else if (resp == pdTRUE && action.tec == TEC2 &&
                 action.type == FALLING) {
                startTick = action.time;
                eState  = WAIT_TIC1_FALLIN;
            }else if (resp == pdTRUE && action.tec == TEC2 &&
                      action.type == RASING) {
                startTick = action.time;
                eState  = WAIT_TIC1_RASING;
            }
            break;
        case WAIT_TIC2_FALLIN:
            if (resp == pdTRUE && action.tec == TEC2 &&
                action.type == FALLING) {
                endTick = action.time;
                order = TEC1_TEC2;
                eState  = MEDIR;
            }else if(resp == pdTRUE && action.tec == TEC1 &&
                     action.type == RASING){
                eState = IDLE;
            }
            break;
        case WAIT_TIC2_RASING:
            if (resp == pdTRUE && action.tec == TEC2 &&
                action.type == RASING) {
                endTick = action.time;
                order = TEC1_TEC2;
                eState  = MEDIR;
            }else if(resp == pdTRUE && action.tec == TEC1 &&
                     action.type == FALLING){
                eState = IDLE;
            }
            break;
        case WAIT_TIC1_FALLIN:
            if (resp == pdTRUE && action.tec == TEC1 &&
                action.type == FALLING) {
                endTick = action.time;
                order = TEC2_TEC1;
                eState  = MEDIR;
            }else if(resp == pdTRUE && action.tec == TEC2 &&
                     action.type == RASING){
                eState = IDLE;
            }
            break;
        case WAIT_TIC1_RASING:
            if (resp == pdTRUE && action.tec == TEC1 &&
                action.type == RASING) {
                endTick = action.time;
                order = TEC2_TEC1;
                eState  = MEDIR;
            }else if(resp == pdTRUE && action.tec == TEC2 &&
                     action.type == FALLING){
                eState = IDLE;
            }
            break;
        case MEDIR:
            eState = IDLE;
            serialInfo info;
            info.bx   = "TEC1";
            info.by   = "TEC2";
            if(order == TEC2_TEC1){
                info.bx   = "TEC2";
                info.by   = "TEC1";
            }
            info.time = endTick - startTick;
            xSemaphoreGive(medicionSem);
            xQueueSend(serialQueue, &info, portMAX_DELAY);
            break;
        }
    }
}

void debounceTask(void *taskParmPtr) {
    gpioQueue_t *aux = (gpioQueue_t *)taskParmPtr;
    while (TRUE) {
        gpioInterrupt_t action;
        BaseType_t resp = xQueueReceive(aux->queue, &action, DEBOUNCE_TIME);
        switch (aux->state) {
        case BUTTON_UP:
            if (resp == pdTRUE && action.type == FALLING) {
                aux->state    = BUTTON_FALLING;
                aux->tmpTicks = action.time;
            } else {
            }
            break;
        case BUTTON_FALLING:
            if (resp == pdFALSE) {
                action.type = FALLING;
                action.time = xTaskGetTickCount();
                xSemaphoreGive(aux->semf);
                xQueueSend(totalQueue, &action, portMAX_DELAY);
                aux->state = BUTTON_DOWN;
            }
            break;
        case BUTTON_DOWN:
            if (resp == pdTRUE && action.type == RASING) {
                aux->state    = BUTTON_RASING;
                aux->tmpTicks = action.time - aux->tmpTicks;
            } else {
            }
            break;
        case BUTTON_RASING:
            if (resp == pdFALSE) {
                action.type = RASING;
                action.time = xTaskGetTickCount();
                xSemaphoreGive(aux->semf);
                xQueueSend(totalQueue, &action, portMAX_DELAY);
                aux->state = BUTTON_UP;
            }
            break;
        default: break;
        }
    }
}

void GPIO0_IRQHandler(void) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpioInterrupt_t action;
    action.tec  = TEC1;
    action.time = xTaskGetTickCount();
    if (gpioRead(TEC1) == OFF) {
        action.type = FALLING;
    } else {
        action.type = RASING;
    }
    xQueueSendFromISR(buttons[0].queue, &action, &xHigherPriorityTaskWoken);
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIO1_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpioInterrupt_t action;

    action.tec  = TEC2;
    action.time = xTaskGetTickCount();
    if (gpioRead(TEC2) == OFF) {
        action.type = FALLING;
    } else {
        action.type = RASING;
    }
    xQueueSendFromISR(buttons[1].queue, &action, &xHigherPriorityTaskWoken);

    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*==================[end of file]============================================*/
