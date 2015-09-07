#include "stdio.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_uart.h"
#include <stdlib.h>

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h" 
#include "semphr.h"

#include "shares.h"
#include "task_master.h"
#include "matrix.h"

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
       // The proportional coefficient from the PID function.
       {"k_proportional",  k_proportional,  NULL,  single_element,  float_type}, 
       
       // The derivative coefficient from the PID function.
       {"k_derivative",    k_derivative,    NULL,  single_element,  float_type}, 
       
       // The integral coefficient from the PID function.
       {"k_integral",      k_integral,      NULL,  single_element,  float_type},
       
       // This is the weighting coefficient used in the tuneable complimentary filter
       // function.
       {"dcm_alpha",       dcm_alpha,       NULL,  single_element,  float_type},
       
       // An index which corresponds to the desired control function in the filter
       // function dispatch table. 
       {"control_select",  control_select,  NULL,  single_element,  int32_t_type},
       
       // An index which corresponds to the desired filter function in the filter
       // function dispatch table. 
       {"filter_select",    filter_select,  NULL,  single_element,  int32_t_type},
       
       // This is a test matrix used to verify that the matrix update, report, and
       // math functions work properly.
       {"test_matrix",      test_matrix,    NULL,      matrix_2x2,  float_type}

    };

/** \fn task_master
 *  \brief This task processes all incoming commands from the user, and decides which software state the
 *  system is in at a given time. 
 */
void task_master(void* pvParameters){
   
    uint8_t default_master_prio = uxTaskPriorityGet(NULL);
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    HAL_UART_Receive_IT(&huart1, packet_hdr, 1);
    xSemaphoreTake(testing_SEMAPHORE,0); // Ensure that there are no free test semaphores available - this prevents the 
                                         // test task from activating prematurely.
    while(1)
    {
          //This section must be made thread-safe via critical section, since other tasks of the 
          //same priority use current_state_SHARED
          taskENTER_CRITICAL(); 
              current_state_SHARED = Compute_Software_State(current_state_SHARED); 
          taskEXIT_CRITICAL();

          // data_received_SHARED is set exclusively from within the UART receive-complete
          // interrupt callback function.
          if(data_received_SHARED)
          {                       
              Process_Command();
              
              // A valid, unprocessed packet will always have a non-zero value in the
              // opcode field. Manually setting the opcode portion of the packet to
              // zero marks the packet as *Processed*, so that it doesn't get processed
              // more than once by the UART interrupt callback function.
              packet_payload[PAYLOAD_OPCODE] = 0; 
              
              data_received_SHARED = 0;
          }
          vTaskDelayUntil(&xLastWakeTime, 10/portTICK_RATE_MS);
    }
}

/** \fn my_HAL_UART_RxCpltCallback
 *  \brief This callback function triggers upon UART receive-complete interrupts. It detects
 *  incoming packets within the stream of incoming received characters, and flags the packets
 *  for processing by the master task.
 * 
 *  \details The header contains two ID bytes and a payload length byte. The function receives characters one 
 *  at a time, until a valid header is detected. It then receives the number of characters specified by
 *  the payload length byte. Once the full payload has been received, the data_received_SHARED flag is set,
 *  prompting the task_master function to process the user-command embedded in the payload. In short:
 *
 *  1. Look for valid header.
 *  2. When header is detected, look for the corresponding payload.
 *  3. Flag the entire packet for processing by the master_task 
 *  
 *  The packet structure is as follows:
 *  Packet : |Header||Payload|
 *  Header : |ID_H|ID_L|Payload_length| ~ 3 bytes total length
 *  Payload : |opcode|index|data|fletcher_16 check bytes(2)| ~ between 4 bytes (no data case) and 255 bytes total length. 
 *  
 *  Fields:
 *  ID_H: 0xA
 *  ID_L: 0x9
 *  Payload_Length: length, in bytes of the payload to be received.
 *  opcode: This corresponds to the command specified by the user. For example, it could correspond to "Report_Variable",
 *      "Update_Variable", "Report_Physical_States", etc.
 *  index: The index, in the GAV table, of the GAV being operated on by the command. 
 *  data: Data associated with the command specified by the opcode. 
 *  Fletcher_16 check bytes: Bytes used with the fletcher_16 checksum algorithm to ensure data fidelity. 
 */
void my_HAL_UART_RxCpltCallback(UART_HandleTypeDef *hUART)
{
          // A UART receive complete interrupt must have triggered to be here. 

	  static uint8_t msg_length = 0;
          // Ensure that the uart receive buffer is clear. This helps, as the buffer can otherwise overflow and 
          // the entire microcontroller could potentially crash as a result. 
	  __HAL_UART_FLUSH_DRREGISTER(hUART);
      
      // This "if" checks that a valid header has arrived, that the packet length is within allowed range, and 
      // that the master task is ready to process a new packet.
      if((packet_hdr[2] == ID_H) && (packet_hdr[1] == ID_L) && (packet_hdr[PACKET_LENGTH]<=MAX_PACKET_LENGTH) && (!data_received_SHARED))
      {
          payload_length_SHARED = packet_hdr[PACKET_LENGTH]; // This variable will be used within other functions, so must be global.
          
          // The header must have arrived, but has payload arrived yet? if NOT, 
          if(0 == packet_payload[PAYLOAD_OPCODE]) 
          {
              // Re-enable uart interrupts to capture the payload - trigger another interrupt after payload_length bytes are received.
              HAL_UART_Receive_IT(hUART, packet_payload, payload_length_SHARED);
          }
          else // if the payload HAS arrived,
          {
              data_received_SHARED = 1; // flag to the master task that a new valid packet has arrived, also block isr from 
                                        // trying to receive any more packets.
              
              packet_hdr[2] = 0; // Flush header capture array. These usually contains ID bytes of the packet. 
              packet_hdr[1] = 0; 
              HAL_UART_Receive_IT(hUART, packet_hdr, 1); // Start looking for the next valid packet header, one byte at a time.
          }
      }
      
      //This "else" executes when no valid packet header has been detected yet.
      else
      {
          __HAL_UART_FLUSH_DRREGISTER(hUART); // Clear data receive buffer.
          
          packet_hdr[2] = packet_hdr[1];      // Shift received data over by 
          packet_hdr[1] = packet_hdr[0];      // one byte in the header buffer.
          packet_hdr[0] = 0;
          HAL_UART_Receive_IT(hUART, packet_hdr, 1); // Read another byte into the header buffer.
      }
}

/** \fn Fletcher_16
 *  \brief This function checks that the payload hasn't been corrupted, by verifying that the 
 *  fletcher_16 checksum of the entire payload equals zero.
 *  \param data This is a pointer to the data to be checked. 
 *  \param count This is the length of the data in bytes.
 */
uint16_t Fletcher_16( uint8_t* data, int count )
{
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   int index;
 
   for( index = 0; index < count; ++index )
   {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

 /** \fn Update_Variable
 *  \brief This function updates the value of one of the Global Adjustable Variables to the value
 *  specified by the user.
 *  \param packet_data_ptr This is a pointer to the data contained in the payload/packet. 
 *  \param target_var_index This is the index of the variable in the table of Global Adjustable Variables.
 *  \param packet_data_size This is the length, in bytes, of the data found in the payload/packet.
 *  \return Returns 1 when the variable's value is successfully updated, 0 when an error occurs.
 */
uint8_t Update_Variable( uint8_t* packet_data_ptr, uint8_t target_var_index, uint8_t packet_data_size){
    
    // The data in the packet must be read into a new buffer. This is so that if the data must be
    // type cast as a float or 32-bit int, its first memory location will be on an even multiple 
    // of 4(preventing a bus error). When this step is skipped, a bus error occurs intermittently,
    // causing the microcontroller to crash. The compiler doesn't catch the error!
    uint8_t count;
    uint8_t temp[64]; // Largest possible data size is a 4*4 Matrix of floats 
                      //(4 bytes/float)*(4 rows)*(4 cols) = 64 bytes
    for(count = 0; count<packet_data_size; count++)
    {
        temp[count] = packet_data_ptr[count];
    }
    
    // The Global Adjustable Variable Table is an array of structs found within the task_master.c file.
    // Each struct corresponds to one of the Global Adjustable Variables. Some of the fields in those
    // structs include a variable corresponding to the data type, and another field is a void* pointer
    // which holds the actual memory location of the Global Adjustable Variable.
    switch(variable_table[target_var_index].data_type)
    {
    
    // Note  - OPTIMIZE THIS WHEN POSSIBLE, VERY LIKELY A BETTER WAY TO DO IT.
        case float_type  :
            packet_data_size = packet_data_size/(sizeof(float));

            //Check that the correct number of elements have been received for the GAV.
            if(variable_table[target_var_index].data_size != packet_data_size)
            {
                printf("\n\r ERROR: Variable data size doesn't agree with packet size\n\r");
                return 0;          
            }
            volatile float *local_data_f;
            local_data_f = variable_table[target_var_index].data;

            // Recall that temp is the character array necessary to prevent that sneaky little bus error.
            // Here we pass the first memory location of that temporary char array to a float pointer.
            float * temp_fp = (float*)temp; 
            
            // Transfer received data, one element at a time, to the location of the GAV 
            for( count = 0; count<packet_data_size; count++)
            {
                local_data_f[count] = *(temp_fp + count);  
            }
            break;
            
        case uint32_t_type  :
            packet_data_size = packet_data_size/(sizeof(uint32_t));
            
            //Check that the correct number of elements have been received for the GAV.
            if(variable_table[target_var_index].data_size != packet_data_size)
            {
                printf("\n\r ERROR: Variable data size doesn't agree with packet size\n\r");
                return 0;
            }
            volatile uint32_t *local_data_u32;
            local_data_u32 = variable_table[target_var_index].data;
            // Recall that temp is the character array necessary to prevent that sneaky little bus error.
            // Here we pass the first memory location of that temporary char array to a uint32 pointer.
            uint32_t *temp_u32p = (uint32_t*)temp;
            
            // Transfer received data, one element at a time, to the location of the GAV 
            for( count = 0; count<packet_data_size; count++)
            {
                local_data_u32[count] = *(temp_u32p + count);  
            }
            break;
            
        case int32_t_type  :
            packet_data_size = packet_data_size/(sizeof(int32_t));
            
            //Check that the correct number of elements have been received for the GAV.
            if(variable_table[target_var_index].data_size != packet_data_size)
            {
                printf("\n\r ERROR: Variable data size doesn't agree with packet size\n\r");
                return 0;
            }
            volatile int32_t *local_data_32;
            local_data_32 = variable_table[target_var_index].data;
            // Recall that temp is the character array necessary to prevent that sneaky little bus error.
            // Here we pass the first memory location of that temporary char array to a int32 pointer.
            int32_t *temp_32p = (int32_t*)temp;
            
            // Transfer received data, one element at a time, to the location of the GAV 
            for( count = 0; count<packet_data_size; count++)
            {
                local_data_32[count] = *(temp_32p + count);  
            }
            break;

        case int16_t_type  :
            packet_data_size = packet_data_size/(sizeof(int16_t));
            
            //Check that the correct number of elements have been received for the GAV.
            if(variable_table[target_var_index].data_size != packet_data_size)
            {
                printf("\n\r ERROR:  Variable data size doesn't agree with packet size\n\r");
                return 0;
            }
            volatile int16_t *local_data_16;
            local_data_16 = variable_table[target_var_index].data;
            // Recall that temp is the character array necessary to prevent that sneaky little bus error.
            // Here we pass the first memory location of that temporary char array to a int16 pointer.
            int16_t * temp_16p = (int16_t*)temp;
            
            // Transfer received data, one element at a time, to the location of the GAV 
            for( count = 0; count<packet_data_size; count++)
            {
                local_data_16[count] = *(temp_16p + count);  
            }
            break; 
            
    }
    return 1;
}

 /** \fn Report_Variable
 *  \brief This function reports the value of one of the Global Adjustable Variables to the user.
 *  \param target_var_index This is the index of the variable in the table of Global Adjustable Variables.
 */
void Report_Variable(uint8_t target_var_index){
    uint8_t count;
   
    // number_of_vars is an enum, corresponding to the total number of Global Adjustable Variables.
    if(target_var_index >= number_of_vars){
        printf("Error: Requested variable index is out of range");
    }
    else
    {
        dimension data_size = variable_table[target_var_index].data_size;
        printf("\n\r");
        printf("----------------------------------------\n\r");
        printf("target_var_index : %d\n\r", target_var_index);
        printf("data size : %d\n\r", data_size);
        
        // Since the GAVs can be of several data types, we use a switch statement to print their values properly
        // according to the appropriate type. 
        switch(variable_table[target_var_index].data_type)
        {
            case float_type  :
                ;
                float *target_var_ptr_f = variable_table[target_var_index].data;
                printf("Variable: %s\n\r", variable_table[target_var_index].namestring);
                if(data_size == single_element)
                {
                    printf("element 0: %lf\n\r", *(target_var_ptr_f) );
                }   
                else if(data_size == matrix_2x2)
                {
                    print_mtrx_nxn(2, target_var_ptr_f);
                    printf("\n\r");
                }
                else if(data_size == matrix_4x4)
                {
                    print_mtrx_nxn(4, target_var_ptr_f);
                    printf("\n\r");
                }
                break;
            
            case uint32_t_type  :
                
                ;
                uint32_t *target_var_ptr_u32 = variable_table[target_var_index].data;
                printf("Variable: %s\n\r", variable_table[target_var_index].namestring);
                for(count =0; count<data_size; count++)
                {
                    printf("element %d: %d\n\r", count, *(target_var_ptr_u32+count) );
                }
                break;
            
            case int32_t_type  :
                ;
                int32_t *target_var_ptr_32 = variable_table[target_var_index].data;
                printf("Variable: %s\n\r", variable_table[target_var_index].namestring);
                for(count =0; count<data_size; count++)
                {
                    printf("element %d: %d\n\r", count, *(target_var_ptr_32+count) );
                }
                break; /* optional */
            
            case int16_t_type  :
                ;
                int16_t *target_var_ptr_16 = variable_table[target_var_index].data;
                printf("Variable: %s\n\r", variable_table[target_var_index].namestring);
                for(count =0; count<data_size; count++)
                {
                    printf("element %d: %d\n\r", count, *(target_var_ptr_16+count) );
                }
                break; /* optional */
        }
    printf("----------------------------------------\n\r");
    }
}
/** \fn Process_Command
 *  \brief This function processes commands received from the user.
 *  \details Every packet received from the 
 *  user contains an "opcode" field. The opcode corresponds to one of the possible commands. Each command
 *  is listed by name in the opcode enum found in the task_master.h file. When the Process_Command() function
 *  is run, it is assumed that there is a new, valid packed within the packet_payload array. This is further
 *  verified by the checksum algorithm, and the checks that the opcode, and index are within the correct range.
 */

void Process_Command(void){
    uint8_t count;
    uint16_t fletcher = Fletcher_16( packet_payload, payload_length_SHARED );// This should be zero for valid data
    uint8_t opcode = packet_payload[PAYLOAD_OPCODE];
    if( !opcode || fletcher )//Verify_fletcher_checksum(payload_length_SHARED, packet_payload))
    {
        // Something has gone wrong here - either the opcode is zero or negative, or the checksum failed
        printf("Error: Last packet corrupted\n\r");
        if (fletcher)
          printf("Fletcher checksum failed\n\r");
        else
          printf("Fletcher checksum OK\n\r");
        if(opcode)
        {
          printf("Current opcode: %d", opcode);
        }
        taskENTER_CRITICAL();
          __HAL_UART_FLUSH_DRREGISTER(&huart1);
          packet_hdr[2] = 0;
          packet_hdr[1] = 0;
          packet_hdr[0] = 0;
        taskEXIT_CRITICAL();
        HAL_UART_Receive_IT(&huart1, packet_hdr, 1);
        // Begin looking for the next valid packet, starting with the header.
    }
    else
    {
        printf("Processing command ");
        switch(packet_payload[PAYLOAD_OPCODE])
        {
            case empty  :
                printf("empty\n\r");
            case update_variable  :
                printf("update_variable \n\r");
                if(current_state_SHARED == error || current_state_SHARED == test)
                {
                    printf("Error: Variables are only user-adjustable from the \"manual\", \"reset\" and \"stabilize\" states\n\r");
                    printf("Current_state_SHARED: %d\n\r", current_state_SHARED);
                    break;
                }
                uint8_t payload_data_size = payload_length_SHARED - 4; // 2 bytes for checksum, 1 for index, 1 for Opcode 
                //if(!Update_Variable(&(packet_payload[PAYLOAD_DATA]), packet_payload[PAYLOAD_INDEX], payload_data_size))
                if(!Update_Variable(packet_payload+PAYLOAD_DATA, packet_payload[PAYLOAD_INDEX], payload_data_size))
                {
                    printf("Error: Incorrect packet data size\n\r");
                }
                break; 
            case variable_report  :
                printf("variable_report \n\r");
                Report_Variable(packet_payload[PAYLOAD_INDEX]);
                break; 
            case full_variable_report  :
                printf("full_variable_report \n\r");
                //statement(s);
                for(count = 0; count<number_of_vars; count++)
                {
                    Report_Variable(count);
                }
                break;
            case update_ref_input :
                printf("update_ref_input \n\r");
                if(current_state_SHARED == manual)
                {
                      // COMING SOON
                }
                else
                {
                    printf("Error: reference input only changeable from Manual State\n\r");
                }
                //statement(s);
                break;
            case update_coefficients :
                printf("update coefficients \n\r");
                //SEND DATA IN A QUEUE
                uint8_t update = 1;
                xQueueSend(update_coefficients_QUEUE, &update, 0);
                break; 
            case change_state  :
                printf("change state \n\r");
                if(current_state_SHARED == error || current_state_SHARED == test )
                {
                    printf("Error: Variables are only user-adjustable from the \"manual\", \"reset\", and \"stabilize\" states\n\r");
                    printf("Current_state_SHARED: %d\n\r", current_state_SHARED);
                    break;
                }
                else if(packet_payload[PAYLOAD_DATA]>error)
                {
                    printf("Error: Requested state does not exist\n\r");
                }
                else
                {
                    current_state_SHARED = packet_payload[PAYLOAD_DATA];
                    if(current_state_SHARED == test)
                    {
                        xSemaphoreGive(testing_SEMAPHORE);
                        test_in_progress_SHARED = 1;
                    }
                }
                //statement(s);
                  break; 
              case report_state :
                  printf("Processing Command : report_state \n\r");
                  printf("\n\r State Table \n\r");
                  printf("--------------- \n\r");
                  printf(" reset     = 0 \n\r");
                  printf(" stabilize = 1 \n\r");
                  printf(" manual    = 2 \n\r");
                  printf(" test      = 3 \n\r");
                  printf(" err       = 4 \n\r");
                  printf("--------------- \n\r\n\r");

                  printf("Current Software State: %d \n\r",current_state_SHARED);
                  break; /* optional */
              case report_pysical_state :
                  printf("Processing Command : report_physical_state \n\r");
                  printf("\n\r State Values \n\r");
                  printf("--------------- \n\r");
                  printf(" Angle            = %f \n\r", physical_states_SHARED[0]);
                  printf(" Angular Velocity = %f \n\r", physical_states_SHARED[1]);
                  printf(" Position         = %f \n\r", physical_states_SHARED[2]);
                  printf(" Velocity         = %f \n\r", physical_states_SHARED[3]);
                  printf("position_enc_0_SHARED = %d", position_enc_0_SHARED);
                  printf("--------------- \n\r\n\r");
                  break; /* optional */
              case initiate_test :
                  printf("Processing Command : initiate_test \n\r");
                  xSemaphoreGive(testing_SEMAPHORE);
                  current_state_SHARED = test;
                  test_in_progress_SHARED = 1;

                  break;
              case report_control_law :
                  printf("Processing Command : report_control_law \n\r");
                  printf("\n\r Control Law Table \n\r");
                  printf("--------------------- \n\r");
                  printf(" no_power        = 0  \n\r");
                  printf(" PID_fixed       = 1  \n\r");
                  printf(" PID_tuneable    = 2  \n\r");
                  printf("--------------------- \n\r\n\r");
                  printf("Current User-selected Control Law: %d \n\r",control_select_SHARED);
              
                  break; /* optional */
              case report_filter_law :
                  printf("Processing Command : report_filter_law \n\r");
                  printf("\n\r Filter Law Table \n\r");
                  printf("------------------- \n\r");
                  printf(" DCM_fixed     = 0  \n\r");
                  printf(" DCM_tuneable  = 1  \n\r");
                  printf("------------------- \n\r\n\r");
                  printf("Current User-selected Filter Law: %d \n\r",filter_select_SHARED);
                  break; /* optional */
              default :
                  printf("Error, incorrect opcode! \n\r");
                  break;
        }
        printf("Last command processed\n\r");
    }
}

/** \fn Compute_Software_State
 *  \brief This function performs the logic required to determine the next software state of the system.
 *  \details This function performs the logic required to determine the next software state of the system,
 *  based on the current software state, and several other internal variables, including flags indicating
 *  that a test is in progress, or that an error has occured.
 */

uint8_t Compute_Software_State(uint8_t current_state)
{
    static uint16_t  reset_count = 0;
    switch(current_state)
          {
              case reset  :
                  if (error_code_SHARED)
                  {
                      return error;
                  }
                  if(abs((int32_t)physical_states_SHARED[0])<5)
                  {
                      reset_count++;
                  }
                  else
                  {
                      reset_count =0;
                  }
                  if (reset_count>300)
                  {
                      return stabilize;
                  }
                  else
                  {                 
                      return reset;
                  }
                  break; 
                  
              case stabilize  :
                  if (error_code_SHARED)
                  {
                      return error;
                  }    
                  if(test_in_progress_SHARED)
                  {
                      return test;
                  }
                  else
                  {
                      return stabilize;
                  }
                  break; 
                  
              case manual  :
                  if (error_code_SHARED)
                  {
                      return error;
                  }
                  if(test_in_progress_SHARED)
                  {
                      return test;
                  }
                  else
                  {
                      return manual;
                  }
                  break; 
                  
              case test  :
                  if (error_code_SHARED)
                  {
                      return error;
                  }
                  else if(test_in_progress_SHARED)
                  {
                      return test;
                  }
                  else
                  {
                      return stabilize;
                  }
                  break;
                  
              case error  :
                  return error;
                  
                  break;
              default :
                  return reset;
                  break;
          }
    printf("problem with state logic - shouldn't reach this point");
    return reset;
}
