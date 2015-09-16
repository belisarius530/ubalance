#ifndef _SHARES_H_
#define _SHARES_H_

typedef enum error_code{
    no_error = 0,
    overcurrent_m0 = 1,
    overcurrent_m1 = 2,
    fault_m0 = 3,
    fault_m1 = 4,
    unit_fallen = 5,
    low_battery = 6
}error_code;

typedef enum software_state{
    reset = 0,
    stabilize = 1,
    manual = 2,
    test = 3,
    error = 4
}software_state;

typedef enum type{
    float_type = 0,
    uint32_t_type = 1,
    int32_t_type = 2,
    int16_t_type = 3
}type;

typedef enum dimension{
    single_element = 1,
    matrix_2x2 = 4,
    matrix_4x4 = 16
}dimension;

typedef enum opcode{
    empty = 0,
    update_variable = 1,
    variable_report = 2,
    full_variable_report = 3,
    update_ref_input = 4,
    update_coefficients = 5,
    change_state = 6,
    report_state = 7,
    report_pysical_state = 8,
    initiate_test = 9,
    report_control_law = 10,
    report_filter_law = 11,
    num_opcodes
}opcode;

/*
WHEN CREATING A NEW ADJUSTABLE VARIABLE!!!
    1 - Pick a new name, and add it to the adjustable variable index enum, 'adj_variable_index'
    2 - Instantiate global struct associated with the adjustable variable, in the 
        variable table of task_master.
    3 - Create a global variable within task control.
    4 - Map the pointer from the struct to the global variable itself within task control
    5 - ensure that in the function of interest, the value of the global
        is assigned to the static local variable in a thread-safe manner(critical section).
        Consider writing this code into some sort of Update Variables subroutine. 
    6 - Declare the variable with "_SHARED" appended to the name in the shares.h file 
    (hint: this one)
*/
typedef enum adj_variable_index{
    k_proportional,
    k_derivative,
    k_integral ,
    dcm_alpha,
    control_select,
    filter_select,
    test_matrix,
    angle_offset,
    number_of_vars
}adj_variable_index;

typedef struct adjustable_variable {
  char* namestring; 
  adj_variable_index index;
  volatile void* data; // This points to the first element in an array of data. If num_elements is set to 1, it just points to a single variable.
  dimension data_size; // The number of elements stored/length of the array. Set to 1 for individual variables. 
  type data_type; // Since I'm using a void* pointer, in order to access the data appropriately, the program will
                 // need to know what type of data is stored in each element of the array. 
} adjustable_variable;

extern volatile adjustable_variable variable_table[number_of_vars];
extern volatile float velocity_enc_0_SHARED;       //defined in task_control
extern volatile float velocity_enc_1_SHARED;
extern volatile int32_t position_enc_0_SHARED;       //defined in task_control
extern volatile int32_t position_enc_1_SHARED;       //defined in task_control
extern volatile int32_t error_enc_0_SHARED;          //defined in task_control
extern volatile int32_t error_enc_1_SHARED;          //defined in task_control
extern volatile uint8_t packet_length_SHARED;        //defined in task_master
extern volatile uint8_t data_received_SHARED;        //defined in task_master
extern volatile uint8_t payload_length_SHARED;       //defined in task_master
extern volatile software_state current_state_SHARED;          //define in task_master
extern float physical_states_SHARED[4];              //defined in task_control
extern error_code error_code_SHARED;                 //defined in task_master
extern uint8_t test_in_progress_SHARED;              //defined in task_master
extern volatile int32_t control_select_SHARED;
extern volatile int32_t filter_select_SHARED;
extern volatile float angle_offset_SHARED;


extern xQueueHandle update_coefficients_QUEUE;        //defined in task_control
extern xQueueHandle control_sig_QUEUE;        

extern xSemaphoreHandle testing_SEMAPHORE;
extern xSemaphoreHandle data_SEMAPHORE;

#endif