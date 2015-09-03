/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    25/04/2015 12:09:51
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
#include "task_master.h"
#include "shares.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/
 
extern void xPortSysTickHandler(void);

extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    xPortSysTickHandler();
  }
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        my_HAL_UART_RxCpltCallback(huart);
    }
}

void EXTI15_10_IRQHandler(void){
	
	   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
       HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}
void EXTI9_5_IRQHandler(void){
       HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
       HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    const uint8_t sequence[6]={3,2,0,1,3,2}; //This is the possible sequence of encoder readings
    static uint8_t state_enc_0_prev = 0;
    static uint8_t state_enc_1_prev = 0;

    if(  (GPIO_Pin==GPIO_PIN_11) || (GPIO_Pin==GPIO_PIN_12) )
    {
        uint8_t state_enc_0 = 0;
        //Get The current state of encoder 0
        if( GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) )
        {
            state_enc_0 += 2;
        }
        if( GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) )
        {
            state_enc_0 += 1;
        }
        
        for (uint8_t i=1; i<5;i++)
        {
            if (state_enc_0==sequence[i])
            {
                //check for ccw, write to dir
                if (state_enc_0_prev==sequence[i-1])
                {
                    position_enc_0_SHARED = position_enc_0_SHARED - 1;
                    break;
                }
                //check for cw, write to dir
                else if(state_enc_0_prev==sequence[i+1])
                {
                    position_enc_0_SHARED = position_enc_0_SHARED + 1;
                    break;
                }
                //
                else if (state_enc_0_prev==sequence[i])
                {
                        position_enc_0_SHARED = position_enc_0_SHARED;
                    break;
                }
                //check for error, write to error
                else
                {
                    error_enc_0_SHARED++;
                    break;
                }
            }
        }
        state_enc_0_prev = state_enc_0;
    }
    else if(  (GPIO_Pin==GPIO_PIN_8) || (GPIO_Pin==GPIO_PIN_9) )
    {
        uint8_t state_enc_1 = 0;
        //Get The current state of encoder 0
        if( GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) )
        {
            state_enc_1+= 2;
        }
        if( GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) )
        {
            state_enc_1+= 1;
        }
        
        for (uint8_t i=1; i<5;i++)
        {
            if (state_enc_1==sequence[i])
            {
                //check for ccw, write to dir
                if (state_enc_1_prev==sequence[i-1])
                {
                    position_enc_1_SHARED = position_enc_1_SHARED - 1;
                    break;
                }
                //check for cw, write to dir
                else if(state_enc_1_prev==sequence[i+1])
                {
                    position_enc_1_SHARED = position_enc_1_SHARED + 1;
                    break;
                }
                //if same, pin change likely occured on M2
                else if (state_enc_1_prev==sequence[i])
                {
                        position_enc_1_SHARED = position_enc_1_SHARED;
                    break;
                }
                //check for error, write to error
                else
                {
                    error_enc_1_SHARED++;
                    break;
                }
            }
        }
        state_enc_1_prev = state_enc_1;
    }
    
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
