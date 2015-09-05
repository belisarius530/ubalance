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



