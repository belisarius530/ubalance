#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h" 
#include "semphr.h"

#include "task_control.h"
#include "shares.h"
#include "matrix.h"

#define GYRO_REG_CTRL_1_ADDR 0x20 // L3GD20 internal memory address of control register 1.
                             // Change to 0xA0 for multiple write
                             // cycles to subsequent control registers.
#define GYRO_REG_CTRL_1_SETTING 0x0F // 95Hz ODR, 12.5Hz high pass cut-off, data enabled.
#define GYRO_I2C_ADDR 0xD6
#define GYRO_REG_DATA_ADDR 0xA8 // 0x28 for a single read. XL, XH, YL, YH, ZL, ZH.


#define ACCL_REG_CTRL_1_ADDR 0x20 // LSM303DLHC internal memory address of control register 1.
                             // Change to 0xA0 for multiple write
                             // cycles to subsequent control registers.
#define ACCL_REG_CTRL_1_SETTING 0x57 // 100Hz ODR, data enabled.
#define ACCL_I2C_ADDR 0x32
#define ACCL_REG_DATA_ADDR 0xA8 // 0x28 for a single read. XL, XH, YL, YH, ZL, ZH.
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

#define PWM_PERIOD 10000
#define MAX_POWER 1000

#define ANGLE 0
#define ANGULAR_VELOCITY 1
#define POSITION 2
#define VELOCITY 3

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define RAD_PER_COUNT (0.0032724923F)
#define WHEEL_RADIUS_METERS (0.0619125F)
#define RADIAN_TO_DEGREES (57.3F)
#define GYRO_DPS_PER_BIT (0.00875F)
#define SAMPLE_TIME_MS (100.0F)

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

volatile int32_t position_enc_0_SHARED;
volatile int32_t position_enc_1_SHARED;
volatile int32_t error_enc_0_SHARED;
volatile int32_t error_enc_1_SHARED;

static uint8_t gyro_reg_ctrl_1_setting = GYRO_REG_CTRL_1_SETTING;
static uint8_t accl_reg_ctrl_1_setting = ACCL_REG_CTRL_1_SETTING;
volatile float K_DER_SHARED =  .01;
volatile float K_PROP_SHARED =  1.0;
volatile float K_INT_SHARED = 0.0;
volatile float DCM_ALPHA_SHARED = .1;
volatile float TEST_MTRX_SHARED[2][2] = {1.0, 2.3, 0.0, 2.3};
volatile int32_t control_select_SHARED = 0;
volatile int32_t filter_select_SHARED = 0;
float physical_states_SHARED[4] = {0.0};
int16_t (*control_fxn_table[num_control_fxn])(float physical_states[4], float reference_input[4], uint8_t update_coefficients);
void (*filter_fxn_table[num_filter_fxn])( float physical_states[4],int16_t *accl_data, int16_t *gyro_data, uint8_t update_coefficients);


void task_control(void* pvParameters){
	uint8_t default_control_prio = uxTaskPriorityGet(NULL);
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	
	int16_t accl_data_buffer[3] = {0}; //X-1, Y-2, Z-3
    int16_t gyro_data_buffer[3] = {0};
    
    filter_fxn_table[DCM_fixed] = DCM_fix;
    filter_fxn_table[DCM_tuneable] = DCM;
    
   //PLEASE UNCOMMENT ASAP control_fxn_table[PID_fixed] = PID_fix;
    control_fxn_table[PID_fixed] = PID;
    control_fxn_table[PID_tuneable] = PID;
    control_fxn_table[no_power] = No_Pwr;
    
    variable_table[k_proportional].data = &K_PROP_SHARED;
    variable_table[k_derivative].data = &K_DER_SHARED;
    variable_table[k_integral].data = &K_INT_SHARED;
    variable_table[dcm_alpha].data = &DCM_ALPHA_SHARED;
    variable_table[control_select].data = &control_select_SHARED;
    variable_table[filter_select].data = &filter_select_SHARED;
    variable_table[test_matrix].data = &TEST_MTRX_SHARED[0][0];
    
    position_enc_0_SHARED = 0;
    position_enc_1_SHARED = 0;
    
    error_enc_0_SHARED = 0;
    error_enc_1_SHARED = 0;
    
    software_state current_state = reset;
    
//    control_law
    control_fxn user_control_select = no_power;
    filter_fxn user_filter_select = DCM_fixed;
    
    control_fxn control_law = no_power;
    filter_fxn filter_law = DCM_fixed;
    
    uint8_t update_coefficients = 1;
	uint8_t n = 0, p = 0;
    int16_t common_motor_signal = 0;
    int16_t diff_motor_signal = 0;
    float reference_input[4] = {0};
    
    gyro_init();
    accl_init();
    xSemaphoreTake(testing_SEMAPHORE,0);
    while(1){
	
         // Take Measurements
         accl_measure(accl_data_buffer);
         gyro_measure(gyro_data_buffer);
   
         // Check to see if the coefficients in the filter and PID functions need to be updated
         xQueueReceive(update_coefficients_QUEUE, &update_coefficients, 0);
         
         // Perform filter and control law computation
         taskENTER_CRITICAL(); 
             current_state = current_state_SHARED;
             if(update_coefficients){
                     user_control_select = control_select_SHARED;
                     user_filter_select = filter_select_SHARED;
             }
         taskEXIT_CRITICAL();
         
         filter_law = filter_logic(current_state, user_filter_select);
         control_law = control_logic(current_state, user_control_select);
             
         (*filter_fxn_table[filter_law])(physical_states_SHARED,accl_data_buffer, gyro_data_buffer, update_coefficients);
         common_motor_signal = -(*control_fxn_table[control_law])(physical_states_SHARED, reference_input, update_coefficients);
         
         
        
         //complimentary_filter(physical_states_SHARED,accl_data_buffer, gyro_data_buffer, update_coefficients);
         //common_motor_signal = PID(physical_states_SHARED, reference_input, update_coefficients);

         // Reset "update_coefficients" variable, so that coeff's don't update until the next time a request
         // is processed through the queue
         update_coefficients = 0;
        
         // Ordinarily (common_motor_signal + differential_motor_signal) would be the actual parameter of the
         // motor_n_power() functions, but I'm still testing here.
         motor_1_power(common_motor_signal + diff_motor_signal);
         motor_0_power(common_motor_signal - diff_motor_signal);
         if(current_state == test)
         {
             xQueueSend(control_sig_QUEUE, &common_motor_signal, 0);
         }
   
        vTaskDelayUntil(&xLastWakeTime, SAMPLE_TIME_MS/portTICK_RATE_MS);
	}
}

void accl_init(void){
    HAL_I2C_Mem_Write(&hi2c1, ACCL_I2C_ADDR, ACCL_REG_CTRL_1_ADDR,
    I2C_MEMADD_SIZE_8BIT, &accl_reg_ctrl_1_setting, 1, HAL_MAX_DELAY);
}
void gyro_init(void){
    HAL_I2C_Mem_Write(&hi2c1, GYRO_I2C_ADDR, GYRO_REG_CTRL_1_ADDR, 
	  I2C_MEMADD_SIZE_8BIT, &gyro_reg_ctrl_1_setting, 1, HAL_MAX_DELAY);
}
void accl_measure(int16_t *accl_data){
    uint8_t *data_pointer = (uint8_t *)accl_data;
    HAL_I2C_Mem_Read(&hi2c1, ACCL_I2C_ADDR, 0xA8, I2C_MEMADD_SIZE_8BIT, data_pointer, 6, HAL_MAX_DELAY);
    //*(accl_data+2)=*(accl_data+2) - 1516;
    *(accl_data+2)=*(accl_data+2);// - 1516;
}
void gyro_measure(int16_t *gyro_data){
    uint8_t *data_pointer = (uint8_t *)gyro_data;
    HAL_I2C_Mem_Read(&hi2c1, GYRO_I2C_ADDR, 0xA8, I2C_MEMADD_SIZE_8BIT, data_pointer, 6, HAL_MAX_DELAY);
}
void motor_0_power(int16_t power){
   // printf("power: %d\n\r", power);
     if(power>0){ //POWER IS POSITIVE
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // PA15 - M0INB
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11 - M0INA
    //     printf("positive");
     }
     else{ //POWER IS NEGATIVE
    //     printf("negative");
         power = abs(power);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);   // PA15 - M0INB
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // PC11 - M0INA
     }
     if(power>MAX_POWER){
         power = MAX_POWER; //1000 currently corresponds to 100%duty cycle
     }
     power *= 10;// With a period setting of 10,000(for 8kHz), values with a max of 1000 only need to be 
                 // multiplied by 10 to be mapped into the 0-10,000 range for duty cycle.
     __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, (uint16_t)power);
}
void motor_1_power(int16_t power){
     if(power>0){ //POWER IS POSITIVE
         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);   // PD2 - M0INB
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // PB5 - M0INA
     }
     else{ //POWER IS NEGATIVE
         power = abs(power);
         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // PD2 - M0INB
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // PB5 - M0INA
     }
     if(power>MAX_POWER){
         power = MAX_POWER; //1000 currently corresponds to 100%duty cycle
     }
     power *= 10;// With a period setting of 10,000, values with a max of 1000 only need to be 
                 // multiplied by 10 to be mapped into the 0-10,000 range
     __HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, (uint16_t)power);
}
void DCM( float physical_states[4],int16_t *accl_data, int16_t *gyro_data, uint8_t update_coefficients){
    // All parameters are out parameters
    static float alpha = .5; // Must be between 0 and 1
    //static int32_t position_enc_0_prev = 0;
    static int32_t position_prev = 0;
    int32_t position = 0;
    
    
    float sample_time_sec = (SAMPLE_TIME_MS)/1000.0;
    if(update_coefficients){
        taskENTER_CRITICAL();
            alpha = DCM_ALPHA_SHARED;
        taskEXIT_CRITICAL();
        position_prev = physical_states[POSITION];
        printf("\n\r Coefficients updated \n\r");
        printf("DELETME in DCM function");
    }
    float accl_pitch_angle = RADIAN_TO_DEGREES*atan2( (double)(accl_data[X_AXIS]),(double)(accl_data[Z_AXIS]) );
    //printf("accl_pitch_angle : %f\n\r",accl_pitch_angle);
    float gyro_delta_angle = (gyro_data[Y_AXIS])*GYRO_DPS_PER_BIT*sample_time_sec;
    //printf("gyro_delta_angle : %f\n\r",gyro_delta_angle);
    physical_states[ANGLE] = alpha*(physical_states[ANGLE]-gyro_delta_angle)+ (1.0-alpha)*accl_pitch_angle;
    physical_states[ANGULAR_VELOCITY] = -GYRO_DPS_PER_BIT*gyro_data[Y_AXIS];
    
    position = (position_enc_0_SHARED + position_enc_1_SHARED)/2;
    physical_states[POSITION] = (float)position*RAD_PER_COUNT*WHEEL_RADIUS_METERS;
    physical_states[VELOCITY] = (float)(position - position_prev)*(100.0)*RAD_PER_COUNT*WHEEL_RADIUS_METERS; //100 comes from 1/T, T being the sample time of .1 sec
    position_prev = position;
    
    //physical_states[POSITION] = 0;
    //physical_states[TRANSLATIONAL_VELOCITY] = 0;
        
}
void DCM_fix( float physical_states[4],int16_t *accl_data, int16_t *gyro_data, uint8_t update_coefficients){
    // All parameters are out parameters
    static float alpha = .5; // Must be between 0 and 1
    float sample_time_sec = (SAMPLE_TIME_MS)/1000.0;
    static int32_t position_prev = 0;
    int32_t position = 0;
    if(update_coefficients){
        taskENTER_CRITICAL();
            alpha = DCM_ALPHA_SHARED;
        taskEXIT_CRITICAL();
        position_prev = physical_states[POSITION];
        printf("\n\r Coefficients updated \n\r");
        printf("DELETME in DCM fixed function");
    }

    float accl_pitch_angle = RADIAN_TO_DEGREES*atan2( (double)(accl_data[X_AXIS]),(double)(accl_data[Z_AXIS]) );
    float gyro_delta_angle = (gyro_data[Y_AXIS])*GYRO_DPS_PER_BIT*sample_time_sec;

    physical_states[ANGLE] = alpha*(physical_states[ANGLE]-gyro_delta_angle)+ (1.0-alpha)*accl_pitch_angle;
    physical_states[ANGULAR_VELOCITY] = -GYRO_DPS_PER_BIT*gyro_data[Y_AXIS];

    position = (position_enc_0_SHARED + position_enc_1_SHARED)/2;
    physical_states[POSITION] = (float)position*RAD_PER_COUNT*WHEEL_RADIUS_METERS;
    physical_states[VELOCITY] = (float)(position - position_prev)*(100.0)*RAD_PER_COUNT*WHEEL_RADIUS_METERS; //100 comes from 1/T, T being the sample time of .1 sec
    position_prev = position;
        
}
int16_t No_Pwr(float physical_states[4], float reference_input[4], uint8_t update_coefficients){
    return 0;
}
int16_t PID(float physical_states[4], float reference_input[4], uint8_t update_coefficients){

	static float integrator_out_prev = 0; 
	static float error_prev = 0;
    
    static float K_INT = 0;
   
    static float K_PROP =  1.0;
    
    static float K_DER = .01;
    
    const int16_t OUT_CLAMP = 1000;
	static float INT_CLAMP = 100;
    
    float error;
	float derivative_out;
	float proportional_out;
	float integrator_out;
    
    if(update_coefficients){
        taskENTER_CRITICAL();
            K_PROP = K_PROP_SHARED;
            K_DER = K_DER_SHARED;
            K_INT = K_INT_SHARED;
        taskEXIT_CRITICAL();
        printf("\n\r Coefficients updated \n\r");
        integrator_out_prev = 0;//flush integrator, update coeffs
    }
	
	error = reference_input[ANGLE] - physical_states[ANGLE];//FOR THE LOVE OF GOD DELETE THIS SHIT!!! FIX THE 4.17 BUGGG!!!
	///////////////////////////////////////
	// Proportional Term Calculation
	proportional_out = K_PROP*error;
	
	
	///////////////////////////////////////
	// Integral Term Calculation
	
	integrator_out = K_INT*(float)error + integrator_out_prev; 
	
	if( integrator_out > INT_CLAMP ) // This integrator saturation limit should be something that is specified as a parameter of the constructor
	{
		integrator_out = INT_CLAMP;
	}
	if( integrator_out < -INT_CLAMP )
	{
		integrator_out = -INT_CLAMP;
	}
	
	integrator_out_prev = integrator_out;
	//////////////////////////////////////
	// Derivative Term Calculation
	derivative_out = K_DER*(error - error_prev );     //derivative of measurement--constant ref drops out.
	
	error_prev = error;

	int16_t out = proportional_out + integrator_out + derivative_out;
	
	if( out  > OUT_CLAMP ) out = OUT_CLAMP;
	if( out < -OUT_CLAMP ) out = -OUT_CLAMP;
	return out;
}
int16_t PID_fix(float physical_states[4], float reference_input[4], uint8_t update_coefficients){

	static float integrator_out_prev = 0; 
	static float error_prev = 0;
    
    float K_INT = 0;
   
    float K_PROP =  1.0;
    
    float K_DER = .01;
    
    const int16_t OUT_CLAMP = 300;
	static float INT_CLAMP = 75;
    
    float error;
	float derivative_out;
	float proportional_out;
	float integrator_out;
    
   
	
	error = reference_input[ANGLE] - physical_states[ANGLE];
	///////////////////////////////////////
	// Proportional Term Calculation
	proportional_out = K_PROP*error;
	
	
	///////////////////////////////////////
	// Integral Term Calculation
	
	integrator_out = K_INT*(float)error + integrator_out_prev; 
	
	if( integrator_out > INT_CLAMP ) // This integrator saturation limit should be something that is specified as a parameter of the constructor
	{
		integrator_out = INT_CLAMP;
	}
	if( integrator_out < -INT_CLAMP )
	{
		integrator_out = -INT_CLAMP;
	}
	
	integrator_out_prev = integrator_out;
	//////////////////////////////////////
	// Derivative Term Calculation
	//derivative_out = K_DER*(error - error_prev );     //derivative of measurement--constant ref drops out.
	derivative_out = -K_DER*physical_states[ANGULAR_VELOCITY];
	error_prev = error;

	int16_t out = proportional_out + integrator_out + derivative_out;
	
	if( out  > OUT_CLAMP ) out = OUT_CLAMP;
	if( out < -OUT_CLAMP ) out = -OUT_CLAMP;
	return out;
}
//control_fxn control_logic(uint8_t software_state, control_fxn user_control_select){
//    
//    switch(software_state)
//        {
//            case error  :
//                return no_power;
//                break;
//            
//            case test  :
//                return user_control_select;
//                break;
//            
//            case manual  :
//                return user_control_select;
//                break;
//            
//            case stabilize  :
//                return PID_fixed;
//                break;
//            
//            case reset  :
//                return no_power;
//                break;
//            
//            default :
//                return no_power;
//                break;
//        }
//}
control_fxn control_logic(uint8_t software_state, control_fxn user_control_select){
    
    switch(software_state)
        {
            case error  :
                return no_power;
                break;
            
            case test  :
                return user_control_select;
                break;
            
            case manual  :
                return user_control_select;
                break;
            
            case stabilize  :
                return PID_fixed;
                break;
            
            case reset  :
                return no_power;
                break;
            
            default :
                return no_power;
                break;
        }
}

filter_fxn filter_logic(uint8_t software_state, filter_fxn user_filter_select){
     switch(software_state)
        {
            case error  :
                return DCM_tuneable;
                break;
            
            case test  :
                return user_filter_select;
                break;
            
            case manual  :
                return user_filter_select;
                break;
            
            case stabilize  :
                return DCM_tuneable;
                break;
            
            case reset  :
                return DCM_tuneable;
                break;
            
            default :
                return DCM_tuneable;
                break;
        }
}
//filter_fxn filter_logic(uint8_t software_state, filter_fxn user_filter_select){
//     switch(software_state)
//        {
//            case error  :
//                return DCM_fixed;
//                break;
//            
//            case test  :
//                return user_filter_select;
//                break;
//            
//            case manual  :
//                return user_filter_select;
//                break;
//            
//            case stabilize  :
//                return DCM_fixed;
//                break;
//            
//            case reset  :
//                return DCM_fixed;
//                break;
//            
//            default :
//                return DCM_fixed;
//                break;
//        }
//}