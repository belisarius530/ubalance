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

void task_control(void* pvParameters);
void accl_init(void);
void gyro_init(void);
void accl_measure(int16_t *accl_data);
void gyro_measure(int16_t *gyro_data);
void motor_0_power(int16_t power);
void motor_1_power(int16_t power);

/*  CREATING A NEW CONTROL LAW
    1. Add new name to the control_fxn enum below.
    2. Place code for the control law function into the task_control file. Ensure that it has
    the correct format of taking physical states, reference inputs, and an update coeff. variable
    as the formal paramaters, and returns a 16 bit integer. 
    3. Map control function to a new index in the function pointer table. Use the enum created in 
    step 1 as the index.     
    4. Create any necessary new adjustable variables
    5. Update Printf messages in the report_control_law opcode section of the "task_master" file
*/
typedef enum control_fxn{
    no_power,
    PID_fixed,
    PID_tuneable,
    num_control_fxn
}control_fxn;
int16_t PID(float physical_states[4], float reference_input[4], uint8_t update_coefficients);
int16_t PID_fix(float physical_states[4], float reference_input[4], uint8_t update_coefficients);
int16_t No_Pwr(float physical_states[4], float reference_input[4], uint8_t update_coefficients);

typedef enum filter_fxn{
    DCM_fixed,
    DCM_tuneable,
    num_filter_fxn
}filter_fxn;
void DCM( float physical_states[4],int16_t *accl_data,
    int16_t *gyro_data, uint8_t update_coefficients);
void DCM_fix( float physical_states[4],int16_t *accl_data,
    int16_t *gyro_data, uint8_t update_coefficients);

control_fxn control_logic(uint8_t software_state, control_fxn user_control_select);
filter_fxn filter_logic(uint8_t software_state, filter_fxn user_filter_select);