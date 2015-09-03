#define ID_L 110
#define ID_H 9
#define MAX_PACKET_LENGTH 127
#define PAYLOAD_OPCODE 0
#define PACKET_LENGTH 0
#define PAYLOAD_INDEX 1
#define PAYLOAD_DATA 2

void task_master(void* pvParameters);
void my_HAL_UART_RxCpltCallback(UART_HandleTypeDef *hUART);
uint16_t Fletcher_16( uint8_t* data, int count );
uint8_t Update_Variable( uint8_t* data_ptr, uint8_t target_var_index, uint8_t data_length);
void Report_Variable(uint8_t target_var_index);
void Process_Command(void);
uint8_t Compute_Software_State(uint8_t current_state);

extern UART_HandleTypeDef huart1;
volatile uint8_t packet_hdr[3] = {0};//Two bytes for ID, one byte for length.
volatile uint8_t packet_payload[MAX_PACKET_LENGTH] = {0};
volatile uint8_t packet_length_SHARED;
volatile uint8_t data_received_SHARED;        //defined in task_master
volatile uint8_t payload_length_SHARED;       //defined in task_master
volatile software_state current_state_SHARED = reset;          //define in task_master
error_code error_code_SHARED = no_error;          //defined in task_master
uint8_t test_in_progress_SHARED = 0;       //defined in task_master

volatile adjustable_variable variable_table[number_of_vars] = {
       // Mapped to a local variable in PID function, in task_control
       {"k_proportional",  k_proportional,  NULL,  single_element,  float_type}, 
       
       // Mapped to a local variable in PID function, in task_control
       {"k_derivative",    k_derivative,    NULL,  single_element,  float_type}, 
       
       // Mapped to a local variable in PID function, in task_control
       {"k_integral",      k_integral,      NULL,  single_element,  float_type},
       
       // Mapped to a local variable in complimentary function, in task_control
       {"dcm_alpha",       dcm_alpha,       NULL,  single_element,  float_type},
       
       // Mapped to a global variable in in task_control
       {"control_select",  control_select,  NULL,  single_element,  int32_t_type},
       
       // Mapped to a global variable in in task_control
       {"filter_select",    filter_select,  NULL,  single_element,  int32_t_type},
       
       // Mapped to a global variable in in task_control
       {"test_matrix",      test_matrix,    NULL,      matrix_2x2,  float_type}

    };