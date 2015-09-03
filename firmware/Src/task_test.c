#include <math.h>
#include <stdlib.h>
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "task_test.h"

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h" 
#include "semphr.h"
#include "shares.h"

void task_test(void* pvParameters){
	uint8_t default_test_prio = uxTaskPriorityGet(NULL);
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t num_samples = 50;
    int16_t control_sig_buff[50] = {0};
    float angle_buff[50] = {0};
    float angle_vel_buff[50] = {0};
    float position_buff[50] = {0};
    float velocity_buff[50] = {0};
    
    
    uint8_t i;
	while(1){
        //HAL_UART_Transmit(&huart1, (uint8_t *)adcbuffer,4,HAL_MAX_DELAY);
        if(pdPASS == xSemaphoreTake(testing_SEMAPHORE, portMAX_DELAY))
        {
            printf("Grabs semaphore, drops down into beginning of actual test data collection");
            for(i=0; i<num_samples; i++)
            {
                if(pdTRUE == xQueueReceive(control_sig_QUEUE, &control_sig_buff[i], portMAX_DELAY))
                {
                   taskENTER_CRITICAL();
                       angle_buff[i]     =  physical_states_SHARED[0];
                       angle_vel_buff[i] =  physical_states_SHARED[1];
                       position_buff[i]  =  physical_states_SHARED[2];
                       velocity_buff[i]  =  physical_states_SHARED[3];
                   taskEXIT_CRITICAL();
                }
                else
                {
                    printf("data collection error: Test task timed out waiting for data\n\r");
                }
                
            }
            printf("Sample,   Angle,  Angular Velocity,   Position,   Velocity,   Control\n\r");
            for(i=0; i<num_samples; i++)
            {
                printf("%d,   %f,   %f,   %f,   %f,   %f\n\r", i, angle_buff[i],
                angle_vel_buff[i],position_buff[i],velocity_buff[i],
                (float)control_sig_buff[i]);
            }
             taskENTER_CRITICAL();
                       test_in_progress_SHARED = 0;
             taskEXIT_CRITICAL();
        }
        else
            ;
       
	    //vTaskDelayUntil(&xLastWakeTime, 100/portTICK_RATE_MS);
	}
}