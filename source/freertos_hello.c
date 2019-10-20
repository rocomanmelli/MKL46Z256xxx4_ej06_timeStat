/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS kernel includes. */
#include "key.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"

#include "board_dsi.h"
#include "uart_rtos.h"
#include "adc.h"
#include "mma8451.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SAMPLES_COUNT 20
#define LENGTH_STR_BUFFER 20
#define LUZ_THR 2000
#define TURNED_ON_TIME 50
#define TURNED_OFF_TIME 100

/*******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef enum{
    TURNED_OFF = 0,
    TURNED_ON_WAITING,
    TURNED_ON,
    TURNED_OFF_WAITING,
} LightSensorStates;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void LightSensorTask(TimerHandle_t xTimer);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */

int main(void){

    TimerHandle_t periodic_task_handle;

    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    board_init();

    key_init();

    adc_init(0);

    uart_rtos_init();

    periodic_task_handle = xTimerCreate("LightSensorTask",
                                        1 / portTICK_PERIOD_MS,
			                            pdTRUE,
		                                NULL,
			                            LightSensorTask);

    xTimerStart(periodic_task_handle, portMAX_DELAY);

    /* xTaskCreate(blinky_task, "adcAcc", 300, NULL, 1, NULL); */

    vTaskStartScheduler();

    for (;;);
}

/*!
 * @brief Task responsible of the light sensor part.
 */
static void LightSensorTask(TimerHandle_t xTimer){
    (void) xTimer;
    int32_t light_average = 0;
    static uint8_t samples = 0, index = 0, ms_count = 0;
    static uint32_t timestamp = 0;
    static LightSensorStates state = TURNED_OFF;
    uint8_t i;
    char str[LENGTH_STR_BUFFER];
    static int32_t light_measurement[SAMPLES_COUNT];

    timestamp++;

    /* Getting value. */
    ADC_IniciarConv();
    if (!adc_getValueBlocking(light_measurement + index, 1)){
        /* Once we note code never gets to this point, we should remove it. */
        while (1);
    }
    if (++index == SAMPLES_COUNT){
        index = 0;
    }

    /* Processing (till sample 20, average is not representative). */
    if (samples < SAMPLES_COUNT - 1){
        samples++;
    }
    else{
        /* Calculating average. */
        for (i = 0; i < SAMPLES_COUNT; i++){
            light_average += light_measurement[i];
        }
        light_average /= SAMPLES_COUNT;
        /* Finite state machine. */
        switch (state){
            case TURNED_OFF:
                if (light_average < LUZ_THR){
                    board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
                    snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:ON\r\n", timestamp);
                    (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                    state = TURNED_ON_WAITING;
                }
                break;
            case TURNED_ON_WAITING:
                ms_count++;
                if (ms_count == TURNED_ON_TIME){
                    ms_count = 0;
                    if (light_average > LUZ_THR){
                        board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
                        snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:OFF\r\n", timestamp);
                        (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                        state = TURNED_OFF_WAITING;
                    }
                    else{
                        state = TURNED_ON;
                    }
                }
                break;
            case TURNED_ON:
                if (light_average > LUZ_THR){
                    board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
                    snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:OFF\r\n", timestamp);
                    (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                    state = TURNED_OFF_WAITING;
                }
                break;
            case TURNED_OFF_WAITING:
                ms_count++;
                if (ms_count == TURNED_OFF_TIME){
                    ms_count = 0;
                    if (light_average < LUZ_THR){
                        board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
                        snprintf(str, LENGTH_STR_BUFFER, "[%d] LED:ON\r\n", timestamp);
                        (void) uart_rtos_envDatos((uint8_t*) str, strlen(str), 1);
                        state = TURNED_ON_WAITING;
                    }
                    else{
                        state = TURNED_OFF;
                    }
                }
                break;
            default:
                break;
        }
    }

}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void blinky_task(void *pvParameters){
    int16_t acc;

    for (;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        acc = mma8451_getAcX();

        if (acc > 50)
            board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
        else
            board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);

        if (acc < -50)
            board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_ON);
        else
            board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_OFF);
    }
}

void vApplicationTickHook(void){
    key_periodicTask1ms();
}

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName){
    while(1);
}









