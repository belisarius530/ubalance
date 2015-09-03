#include <math.h>
#include <stdlib.h>
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "task_safety.h"

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h" 
#include "semphr.h"
#include "shares.h"


void task_safety(void* pvParameters){
	uint8_t default_safety_prio = uxTaskPriorityGet(NULL);
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    

	GPIO_PinState M0ENAB = GPIO_PIN_RESET;
    GPIO_PinState M1ENAB = GPIO_PIN_RESET;
    volatile uint32_t adcbuffer[6] = {0,0,0,0,0,0};
    error_code_SHARED  = 0;
    HAL_ADC_Start(&hadc1); 
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcbuffer, 6);
    float angle;
    while(current_state_SHARED == reset)
    {
        ;
        vTaskDelayUntil(&xLastWakeTime, 250/portTICK_RATE_MS);
    }
	while(1){
        //HAL_UART_Transmit(&huart1, (uint8_t *)adcbuffer,4,HAL_MAX_DELAY);
        M0ENAB = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
        M1ENAB = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
        taskENTER_CRITICAL(); 
            angle = physical_states_SHARED[0];
            if (abs((int32_t)angle)>35)
            {
                current_state_SHARED = reset;
            }
            if(!M0ENAB)
            {
                error_code_SHARED = fault_m0;
            }
            if(!M1ENAB)
            {
                error_code_SHARED = fault_m1;
            }
        taskEXIT_CRITICAL();
	    vTaskDelayUntil(&xLastWakeTime, 100/portTICK_RATE_MS);
	}
}

