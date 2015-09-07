/** \file task_control.c
 *  \brief  This file contains the control task, and all of the functions used by the control task. 
 *  \details Within this file is the control task. The purpose of the control task is to collect
 *  sensor measurements for position and orientation, perform some filtering algorithm
 *  upon those measurements to obtain estimates for the physical states of the system, and to 
 *  operate on those state estimates with a control algorithm to produce a control signal for the motors.
 *  The correct control and filter algorithms to be used are calculated in this file as well.
 *  They can be specified by the user when the software state is in the "Manual" or "Test" states.  
 */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "math.h"
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"                     
#include "task.h"       
#include "queue.h"       
#include "croutine.h" 
#include "semphr.h"

#include "task_control.h"
#include "shares.h"
#include "matrix.h"
// PUT ALL DEFINES IN THE HEADER FILE PLS
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

// Declare required peripheral handles here.
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

// Declare Global variables.
volatile int32_t position_enc_0_SHARED;
volatile int32_t position_enc_1_SHARED;
volatile int32_t error_enc_0_SHARED;
volatile int32_t error_enc_1_SHARED;
float physical_states_SHARED[4] = {0.0};

// Declare constants.
static uint8_t gyro_reg_ctrl_1_setting = GYRO_REG_CTRL_1_SETTING;
static uint8_t accl_reg_ctrl_1_setting = ACCL_REG_CTRL_1_SETTING;

// Define Global Adjustable Variables here.
volatile float K_DER_SHARED =  .01; //PID
volatile float K_PROP_SHARED =  1.0; //PID
volatile float K_INT_SHARED = 0.0;  //PID
volatile float DCM_ALPHA_SHARED = .1; //DCM
volatile float TEST_MTRX_SHARED[2][2] = {1.0, 2.3, 0.0, 2.3}; //Not associated with any control laws
volatile int32_t control_select_SHARED = 0; //
volatile int32_t filter_select_SHARED = 0;

// Declare control and filter function dispatch tables here. 
int16_t (*control_fxn_table[num_control_fxn])(float physical_states[4], float reference_input[4], uint8_t update_coefficients);
void (*filter_fxn_table[num_filter_fxn])( float physical_states[4],int16_t *accl_data, int16_t *gyro_data, uint8_t update_coefficients);

/** \fn task_control
 *  \brief Handles sensor data collection, fusion, selection of control & filter laws, and computation of control signal.
 */
void task_control(void* pvParameters){
	uint8_t default_control_prio = uxTaskPriorityGet(NULL);
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	
    // These arrays are the memory location where the raw data is stored.
    int16_t accl_data_buffer[3] = {0}; //X-1, Y-2, Z-3
    int16_t gyro_data_buffer[3] = {0};
    
    // Map all filter functions into their proper places within the filter law dispatch table
    filter_fxn_table[DCM_fixed] = DCM_fix;
    filter_fxn_table[DCM_tuneable] = DCM;
   
    // Map all control functions into their proper places within the control law dispatch table.
    control_fxn_table[PID_fixed] = PID_fix;
    control_fxn_table[PID_tuneable] = PID;
    control_fxn_table[no_power] = No_Pwr;
    
    // The adjustable variable table is an array of structs. One of the fields within that struct is
    // a void pointer to the actual data itself(another field corresponds to the type of data pointed
    // to). Here we assign the actual global adjustable variables to their pointers within the adjustable variable
    // table.
    variable_table[k_proportional].data = &K_PROP_SHARED;
    variable_table[k_derivative].data = &K_DER_SHARED;
    variable_table[k_integral].data = &K_INT_SHARED;
    variable_table[dcm_alpha].data = &DCM_ALPHA_SHARED;
    variable_table[control_select].data = &control_select_SHARED;
    variable_table[filter_select].data = &filter_select_SHARED;
    variable_table[test_matrix].data = &TEST_MTRX_SHARED[0][0];
    
    // Initialize encoder position to zero.
    position_enc_0_SHARED = 0;
    position_enc_1_SHARED = 0;
    
    // Initialize encoder error to zero.
    error_enc_0_SHARED = 0;
    error_enc_1_SHARED = 0;
    
    // Initialize local copy of the current system sofware state.
    software_state current_state = reset;
    
    // Initialize the user-selected control and filter laws.
    control_fxn user_control_select = PID;
    filter_fxn user_filter_select = DCM;
    
    // Initialize the start-up control and filter laws.
    control_fxn control_law = no_power;
    filter_fxn filter_law = DCM_fixed;
    
    uint8_t update_coefficients = 1; // Ordinarily when this variable is high, the main loop within 
                                     // the control task will update all local copies of the global adjustable variables. 
    uint8_t n = 0, p = 0;
    int16_t common_motor_signal = 0;
    int16_t diff_motor_signal = 0; // This will correspond to the turn signal.
    float reference_input[4] = {0}; // Desired values of the physical states - an input from the user.
    
    // Initialize i2c interfaced gyro and accelerometer.
    gyro_init();
    accl_init();
    
    // Make sure there are no free testing_semaphores lying around, lest a test routine
    // triggers prematurely.
    xSemaphoreTake(testing_SEMAPHORE,0);

    // Main task_control loop
    while(1){
	
         // Take Measurements. Mind you Encoders are constantly updating via external interrupts.
         accl_measure(accl_data_buffer);
         gyro_measure(gyro_data_buffer);
   
         // Check to see if the local copies of global adjustable variables need to be updated
         xQueueReceive(update_coefficients_QUEUE, &update_coefficients, 0);
         
         // Determine software state. If the update coeffients flag is high, determine which
         // control law the user wants to use. 
         taskENTER_CRITICAL(); 
             current_state = current_state_SHARED;
             if(update_coefficients){
                     user_control_select = control_select_SHARED;
                     user_filter_select = filter_select_SHARED;
             }
         taskEXIT_CRITICAL();
         
         // Based on the current software state, compute the proper control and filter laws to be used. 
         filter_law = filter_logic(current_state, user_filter_select);
         control_law = control_logic(current_state, user_control_select);
             
         // Execute the control and filter laws through their respective dipatch tables. This produces
         // the needed control signal for the motors.  
         (*filter_fxn_table[filter_law])(physical_states_SHARED,accl_data_buffer, gyro_data_buffer, update_coefficients);
         common_motor_signal = -(*control_fxn_table[control_law])(physical_states_SHARED, reference_input, update_coefficients);
         // Reset "update_coefficients" variable, so that coeff's don't update until the next time a request
         // is processed through the queue
         update_coefficients = 0;
        
         // Apply the motor control signal to the motors
         motor_1_power(common_motor_signal + diff_motor_signal);
         motor_0_power(common_motor_signal - diff_motor_signal);

         // If a test routine is in progress, send out copies of the control signal for data collection.
         if(current_state == test)
         {
             xQueueSend(control_sig_QUEUE, &common_motor_signal, 0);
         }
   
        // Delay for a sample period.
        vTaskDelayUntil(&xLastWakeTime, SAMPLE_TIME_MS/portTICK_RATE_MS);
	}
}
/** \fn accl_init 
 *  \brief This function initializes the accelerometer to use a 100Hz Output Data Rate(ODR),
 *  and enables data collection in the first place.
 */
void accl_init(void){
    HAL_I2C_Mem_Write(&hi2c1, ACCL_I2C_ADDR, ACCL_REG_CTRL_1_ADDR,
    I2C_MEMADD_SIZE_8BIT, &accl_reg_ctrl_1_setting, 1, HAL_MAX_DELAY);
}


/** \fn gyro_init 
 *  \brief This function initializes the angular rate gyro to use a 100Hz Output Data Rate(ODR),
 *  and enables data collection in the first place.
 */
void gyro_init(void){
    HAL_I2C_Mem_Write(&hi2c1, GYRO_I2C_ADDR, GYRO_REG_CTRL_1_ADDR, 
	  I2C_MEMADD_SIZE_8BIT, &gyro_reg_ctrl_1_setting, 1, HAL_MAX_DELAY);
}


/** \fn accl_measure 
 *  \brief This function prompts the accelerometer to collect a fresh set of data.
 *  \param accl_data This is a pointer to the location where the accelerometer data will
 *  be stored. 
 */
void accl_measure(int16_t *accl_data){
    uint8_t *data_pointer = (uint8_t *)accl_data;
    HAL_I2C_Mem_Read(&hi2c1, ACCL_I2C_ADDR, 0xA8, I2C_MEMADD_SIZE_8BIT, data_pointer, 6, HAL_MAX_DELAY);
    // *(accl_data+2)=*(accl_data+2);// - 1516; // PLEASE REVIEW
}

/** \fn gyro_measure 
 *  \brief This function prompts the gyro to collect a fresh set of data.
 *  \param accl_data This is a pointer to the location where the gyro data will
 *  be stored. 
 */
void gyro_measure(int16_t *gyro_data){
    uint8_t *data_pointer = (uint8_t *)gyro_data;
    HAL_I2C_Mem_Read(&hi2c1, GYRO_I2C_ADDR, 0xA8, I2C_MEMADD_SIZE_8BIT, data_pointer, 6, HAL_MAX_DELAY);
}


/** \fn motor_0_power 
 *  \brief This function sends pwm signal to the motor controller circuits, which then
 *  deliver an amount of power proportional to the duty cycle.
 *  \param power This is the power signal which will go to the motors. It's allowed
 *  values are in the range of (-1000, 1000), where |1000| corresponds to a duty cycle
 *  of 100, and the sign corresponds to the direction of wheel motion.
 */ 
void motor_0_power(int16_t power){
     // If the power signal is positive, drive the motors in one direction.
     if(power>0)
     { 
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // PA15 - M0INB
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11 - M0INA
     }
 
     // If the power signal is positive, drive the motors in the opposite direction
     else 
     {
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

// COPY COMMENTS FROM MOTOR 0 POWER
/** \fn motor_1_power 
 *  \brief This function sends pwm signal to the motor controller circuits, which then
 *  deliver an amount of power proportional to the duty cycle.
 *  \param power This is the power signal which will go to the motors. It's allowed
 *  values are in the range of (-1000, 1000), where |1000| corresponds to a duty cycle
 *  of 100, and the sign corresponds to the direction of wheel motion.
 */ 
void motor_1_power(int16_t power){
     if(power>0)
     { 
         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);   // PD2 - M0INB
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // PB5 - M0INA
     }
     else
     {
         power = abs(power);
         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // PD2 - M0INB
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // PB5 - M0INA
     }
     if(power>MAX_POWER)
     {
         power = MAX_POWER;
     }
     power *= 10;// With a period setting of 10,000, values with a max of 1000 only need to be 
                 // multiplied by 10 to be mapped into the 0-10,000 range
     __HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, (uint16_t)power);
}


/** \fn DCM 
 *  \brief Direction Cosine Matrix - this is an adjustable complimentary filter, which fuses the
 *  measurement data into state estimates by effectively low-pass filtering the accelerometers, 
 *  high-pass filtering the gyros, and producing a weighted average based on the two results.
 *  \param physical_states Location in memory of the current estimates of the physical states of the 
 *  system.
 *  \param accl_data This is a pointer to the latest accelerometer measurement.
 *  \param gyro_data This is a pointer to the latest gyro measurment.
 *  \param update_coefficients This is a flag which when set, prompts the function to 
 *  refresh its local copies of adjustable global variables
 */ 
void DCM( float physical_states[4],int16_t *accl_data, int16_t *gyro_data, uint8_t update_coefficients){
    static float alpha = .5; // Must be between 0 and 1
    static int32_t position_prev = 0;
    int32_t position = 0;  
    const static float sample_time_sec = (SAMPLE_TIME_MS)/1000.0;
   
    // If the update coefficients flag has been set, update local copies of global adjustable
    // variables in a thread-safe manner.
    if(update_coefficients)
    {
        taskENTER_CRITICAL();
            alpha = DCM_ALPHA_SHARED;
        taskEXIT_CRITICAL();
        position_prev = physical_states[POSITION];
        printf("\n\r Coefficients updated \n\r");
    }
    // Compute the angle from the accelerometer measurements, and the change in angle from the 
    // gyro measurements.
    float accl_pitch_angle = RADIAN_TO_DEGREES*atan2( (double)(accl_data[X_AXIS]),(double)(accl_data[Z_AXIS]) );
    float gyro_delta_angle = (gyro_data[Y_AXIS])*GYRO_DPS_PER_BIT*sample_time_sec;
    
    // Compute the estimated angle based on the measurements.
    physical_states[ANGLE] = alpha*(physical_states[ANGLE]-gyro_delta_angle)+ (1.0-alpha)*accl_pitch_angle;
    physical_states[ANGULAR_VELOCITY] = -GYRO_DPS_PER_BIT*gyro_data[Y_AXIS];
    
    // Compute the translational position and velocity of the device.
    position = (position_enc_0_SHARED + position_enc_1_SHARED)/2;
    physical_states[POSITION] = (float)position*RAD_PER_COUNT*WHEEL_RADIUS_METERS;
    physical_states[VELOCITY] = (float)(position - position_prev)*(100.0)*RAD_PER_COUNT*WHEEL_RADIUS_METERS; //100 comes from 1/T, T being the sample time of .1 sec
    position_prev = position;
}

// UPDATE COMMENTS WITH THOSE FROM DCM
/** \fn DCM_fix
 *  \brief Direction Cosine Matrix - this is an adjustable complimentary filter, which fuses the
 *  measurement data into state estimates by effectively low-pass filtering the accelerometers, 
 *  high-pass filtering the gyros, and producing a weighted average based on the two results.
 *  \param physical_states Location in memory of the current estimates of the physical states of the 
 *  system.
 *  \param accl_data This is a pointer to the latest accelerometer measurement.
 *  \param gyro_data This is a pointer to the latest gyro measurment.
 *  \param update_coefficients This is a flag which when set, prompts the function to 
 *  refresh its local copies of adjustable global variables
 */ 
void DCM_fix( float physical_states[4],int16_t *accl_data, int16_t *gyro_data, uint8_t update_coefficients){
    static float alpha = .5; // Must be between 0 and 1
    float sample_time_sec = (SAMPLE_TIME_MS)/1000.0;
    static int32_t position_prev = 0;
    int32_t position = 0;
    
    // If the update coefficients flag has been set, update local copies of global adjustable
    // variables in a thread-safe manner.
    if(update_coefficients){
        taskENTER_CRITICAL();
            alpha = DCM_ALPHA_SHARED;
        taskEXIT_CRITICAL();
        position_prev = physical_states[POSITION];
        printf("\n\r Coefficients updated \n\r");
        printf("DELETME in DCM fixed function");
    }

    // Compute the angle from the accelerometer measurements, and the change in angle from the 
    // gyro measurements.
    float accl_pitch_angle = RADIAN_TO_DEGREES*atan2( (double)(accl_data[X_AXIS]),(double)(accl_data[Z_AXIS]) );
    float gyro_delta_angle = (gyro_data[Y_AXIS])*GYRO_DPS_PER_BIT*sample_time_sec;

    // Compute the estimated angle based on the measurements.
    physical_states[ANGLE] = alpha*(physical_states[ANGLE]-gyro_delta_angle)+ (1.0-alpha)*accl_pitch_angle;
    physical_states[ANGULAR_VELOCITY] = -GYRO_DPS_PER_BIT*gyro_data[Y_AXIS];

    // Compute the translational position and velocity of the device.
    position = (position_enc_0_SHARED + position_enc_1_SHARED)/2;
    physical_states[POSITION] = (float)position*RAD_PER_COUNT*WHEEL_RADIUS_METERS;
    physical_states[VELOCITY] = (float)(position - position_prev)*(100.0)*RAD_PER_COUNT*WHEEL_RADIUS_METERS; //100 comes from 1/T, T being the sample time of .1 sec
    position_prev = position;
        
}

/** \fn No_Pwr
 *  \brief  Using this for the control function basically just turns off the motors.
 */
int16_t No_Pwr(float physical_states[4], float reference_input[4], uint8_t update_coefficients){
    return 0;
}

/** \fn PID
 *  \brief  This is an online tuneable PID function.
 *  \param physical_states These represent the physical states of the system as determined by
 *  the filter law that's in use: angle, angular velocity, translational position, and
 *  translational velocity, for example.
 *  \param reference_input This is an array which corresponds to the control system input -
 *  the desired physical states of the system.
 *  \param update_coefficients When this is non-zero, the local static copies of the global, 
 *  adjustable variables refresh their values. 
 *  \return Returns the computed control signal for the motor controllers. 
 */
int16_t PID(float physical_states[4], float reference_input[4], uint8_t update_coefficients){
	static float integrator_out_prev = 0; 
	static float error_prev = 0;
    static float K_INT = 0;
    static float K_PROP =  1.0;
    static float K_DER = .01;
    const static int16_t OUT_CLAMP = 1000;
    const static float INT_CLAMP = 100;
    float error;
	float derivative_out;
	float proportional_out;
	float integrator_out;
    
    // If necessary, update local copies of global adjustable variables.
    if(update_coefficients){
        taskENTER_CRITICAL();
            K_PROP = K_PROP_SHARED;
            K_DER = K_DER_SHARED;
            K_INT = K_INT_SHARED;
        taskEXIT_CRITICAL();
        printf("\n\r Coefficients updated \n\r");
        integrator_out_prev = 0;
    }
	
	error = reference_input[ANGLE] - physical_states[ANGLE];//FOR THE LOVE OF GOD DELETE THIS SHIT!!! FIX THE 4.17 BUGGG!!!
	///////////////////////////////////////
	// Proportional Term Calculation
	proportional_out = K_PROP*error;
	
	
	///////////////////////////////////////
	// Integral Term Calculation
	
	integrator_out = K_INT*(float)error + integrator_out_prev; 
	
	if( integrator_out > INT_CLAMP ) 
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
    //derivative_out = -K_DER*physical_states[ANGULAR_VELOCITY];
	derivative_out = K_DER*(error - error_prev );   // Derivative on measurement--constant reference input drops out.
	                                                // This also eliminates "derivative kick" problems.
	error_prev = error;

    /////////////////////////////////////
    // Compute Output Signal
	int16_t out = proportional_out + integrator_out + derivative_out;
	if( out  > OUT_CLAMP ) out = OUT_CLAMP;
	if( out < -OUT_CLAMP ) out = -OUT_CLAMP;
	return out;
}



/** \fn PID_fix
 *  \brief  This is a pre-tuned PID function, which will be used in the "stabilize" state.
 *  \param physical_states These represent the physical states of the system as determined by
 *  the filter law that's in use: angle, angular velocity, translational position, and
 *  translational velocity.
 *  \param reference_input This is an array which corresponds to desired physical states of the system.
 *  \param update_coefficients When this is non-zero, the local static copies of the global, 
 *  adjustable variables refresh their values. 
 *  \return Returns the computed control signal for the motor controllers. 
 */
int16_t PID_fix(float physical_states[4], float reference_input[4], uint8_t update_coefficients){
	static float integrator_out_prev = 0; 
	static float error_prev = 0;
    float K_INT = 0;
    float K_PROP =  1.0;
    float K_DER = .01;
    const static int16_t OUT_CLAMP = 300;
	const static float INT_CLAMP = 75;
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


/** \fn control_logic
 *  \brief This function implements logic for selecting the appropriate control law to use,
 *   given the current software state, and the user-selected control law. 
 *  \param software_state An enum corresponding to the current software state.
 *  \param user_control_select An enum corresponding to the desired user-selected control law.
 *  \return Returns an index for the appropriate filter function in the filter dispatch table.
 */
control_fxn control_logic(uint8_t software_state, control_fxn user_control_select){
    
    switch(software_state)
        {
            case error  :
                return no_power;
            case test  :
                return user_control_select;
            case manual  :
                return user_control_select;
            case stabilize  :
                return PID_fixed;
            case reset  :
                return no_power;
            default :
                return no_power;
        }
}

/** \fn filter_logic
 *  \brief This function implements logic for selecting the appropriate filter law to use,
 *   given the current software state, and the user-selected filter law. 
 *  \param software_state An enum corresponding to the current software state.
 *  \param user_filter_select An enum corresponding to the desired user-selected filter.
 *  \return Returns an index for the appropriate filter function in the filter dispatch table.
 */
filter_fxn filter_logic(uint8_t software_state, filter_fxn user_filter_select){
     switch(software_state)
        {
            case error  :
                return DCM_tuneable;
            case test  :
                return user_filter_select;
            case manual  :
                return user_filter_select;
            case stabilize  :
                return DCM_tuneable;
            case reset  :
                return DCM_tuneable;
            default :
                return DCM_tuneable;
        }
}

