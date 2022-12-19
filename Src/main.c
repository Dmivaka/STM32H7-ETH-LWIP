/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "net.h"
#include "circ_buffer.h"
#include "helpers.h"
#include <string.h>

#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"

#include "lwip.h"
#include "lwip/udp.h"

#include "libcanard/canard.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/array/Real32_1_0.h"

#include "hl_command_msg.h"
#include "hl_state_msg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
uint8_t FDCAN1_TX_body[buf_size] = {0};
buffer_instance FDCAN1_TX_buf = {0, NULL, 0, FDCAN1_TX_body};

uint8_t FDCAN1_RX_body[buf_size] = {0};
buffer_instance FDCAN1_RX_buf = {0, NULL, 0, FDCAN1_RX_body};

uint8_t FDCAN2_TX_body[buf_size] = {0};
buffer_instance FDCAN2_TX_buf = {0, NULL, 0, FDCAN2_TX_body};

uint8_t FDCAN2_RX_body[buf_size] = {0};
buffer_instance FDCAN2_RX_buf = {0, NULL, 0, FDCAN2_RX_body};

uint8_t FDCAN3_TX_body[buf_size] = {0};
buffer_instance FDCAN3_TX_buf = {0, NULL, 0, FDCAN3_TX_body};

uint8_t FDCAN3_RX_body[buf_size] = {0};
buffer_instance FDCAN3_RX_buf = {0, NULL, 0, FDCAN3_RX_body};

FDCAN_HandleTypeDef * FDCAN_Handles_Map[3] = {&hfdcan1, &hfdcan2, &hfdcan3};
buffer_instance * TX_Buffers_Map[3] = { &FDCAN1_TX_buf, &FDCAN2_TX_buf, &FDCAN3_TX_buf};
buffer_instance * RX_Buffers_Map[3] = { &FDCAN1_RX_buf, &FDCAN2_RX_buf, &FDCAN3_RX_buf};

// the upper two kbytes of SRAM2 region
#pragma location=0x30007800
uint8_t SPI_TX_buf[512] = {0};
#pragma location=0x30007C00
uint8_t SPI_RX_body[1024] = {0};

buffer_instance SPI_RX_buf = {0, NULL, 0, SPI_RX_body};

CanardInstance 	canard; // This is the core structure that keeps all of the states and allocated resources of the library instance

// each queue is bound to its dedicated can-bus - 3 on the main and 3 on the companion chip.
CanardTxQueue 	queue0; // Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface
CanardTxQueue 	queue1;
CanardTxQueue 	queue2;
CanardTxQueue 	queue3;
CanardTxQueue 	queue4;
CanardTxQueue 	queue5;

CanardTxQueue * TxQueuesMap[6] = { &queue0, &queue1, &queue2, &queue3, &queue4, &queue5 };

//int8_t device_to_queue[12] = {1,1,2,2,3,3,4,4,5,5,6,6};
int8_t device_to_queue[12] = {  1,  0, -1,
                               -1, -1, -1,
                               -1, -1, -1,
                               -1, -1, -1}; // only null and first driver are enabled - the can1 and can0 buses respectevely

uint16_t device_node_id[12] = {  10, 11, 12,
                                 13, 14, 15,
                                 16, 17, 18,
                                 19, 20, 21 };

uint16_t device_tx_port[12] = {  1000, 1010, 1020,
                                 1030, 1040, 1050,
                                 1060, 1070, 1080,
                                 1090, 1100, 1200 };

uint16_t device_rx_port[12] = {  1001, 1011, 1021,
                                 1031, 1041, 1051,
                                 1061, 1071, 1081,
                                 1091, 1101, 1201 };

CanardRxSubscription Device0_Sub;
CanardRxSubscription Device1_Sub;
CanardRxSubscription Device2_Sub;
CanardRxSubscription Device3_Sub;
CanardRxSubscription Device4_Sub;
CanardRxSubscription Device5_Sub;
CanardRxSubscription Device6_Sub;
CanardRxSubscription Device7_Sub;
CanardRxSubscription Device8_Sub;
CanardRxSubscription Device9_Sub;
CanardRxSubscription Device10_Sub;
CanardRxSubscription Device11_Sub;

/*
CanardRxSubscription * CanardSubscriptionMap[12] = {  &Device0_Sub, &Device1_Sub, &Device2_Sub,
                                                      &Device3_Sub, &Device4_Sub, &Device5_Sub,
                                                      &Device6_Sub, &Device7_Sub, &Device8_Sub,
                                                      &Device9_Sub, &Device10_Sub, &Device11_Sub };
*/

CanardRxSubscription * CanardSubscriptionMap[12] = {  &Device0_Sub, &Device1_Sub, NULL,
                                                      NULL, NULL, NULL,
                                                      NULL, NULL, NULL,
                                                      NULL, NULL, NULL };

extern uint8_t LCM_rx_flag;
extern uint8_t LCM_tx_flag;
extern hl_command_msg rx_lcm_msg;
extern measurment tx_lcm_msg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
uint8_t write_can_frame(buffer_instance * s, uint8_t src_bus, FDCAN_RxHeaderTypeDef * head, uint8_t *data);
uint8_t read_can_frame(buffer_instance * s, FDCAN_TxHeaderTypeDef * head, uint8_t *data);

static void *memAllocate(CanardInstance *const canard, const size_t amount);
static void memFree(CanardInstance *const canard, void *const pointer);

void process_vb_rx_frame(uint8_t *local_buffer, uint8_t frame_length);
void process_canard_TX_queue( uint8_t queue_num );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint64_t TIM7_ITs;
uint64_t micros()
{ 
  return (uint64_t)(__HAL_TIM_GET_COUNTER(&htim7) + 50000u * TIM7_ITs);
}

buffer_instance gaga = {0, NULL, 0, NULL};
uint8_t my_buffer[buf_size] = {0};
uint8_t timer_update_flag = 0;

extern uint8_t companion_TX_buf[buf_size];
extern buffer_instance companion_TX_ins;

uint32_t previos_char = 9000;

uint8_t debug_collector[1024] = {0};
uint32_t debug_index = 0;

uint8_t UDP_TX_data[1024] = {0};
buffer_instance UDP_TX_ring = {0, NULL, 0, UDP_TX_data};

extern struct udp_pcb *upcb;

uint16_t index = 0;
uint8_t udp_out_buffer[512] = {0};  

uint16_t response_recorder = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  canard = canardInit(&memAllocate, &memFree);	// Initialization of a canard instance
  canard.node_id = 40;
  
  for( int i = 0; i < 6; i++ )
  {
    *TxQueuesMap[i] = canardTxInit( 10, CANARD_MTU_CAN_FD); // really we need 1 element only
  }

  for( int i = 0; i < 12; i++ )
  {
    if( CanardSubscriptionMap[i] != NULL )
    {
      if( canardRxSubscribe(    &canard,
                                CanardTransferKindMessage,
                                device_rx_port[i],
                                uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                CanardSubscriptionMap[i] ) != 1 ){ Error_Handler(); }
    }
  }
  
  // bind uint8_t arrays to each circular buffer instance
  
  gaga.buffer_body = my_buffer;
  
  /*********************************************

  for( int i = 0; i < 32; i++ )
  {
      dummy_buffer[i] = i + 32;
      more_dummy_buffer[i] = 0;
  }

  FDCAN_RxHeaderTypeDef RxHeader;
  RxHeader.Identifier = 134;
  RxHeader.DataLength = LengthCoder(20);
  
  write_can_frame(&gaga, &RxHeader, dummy_buffer);
  
  RxHeader.Identifier = 251;
  RxHeader.DataLength = LengthCoder(7);  
  
  write_can_frame(&gaga, &RxHeader, dummy_buffer);  
  
  RxHeader.Identifier = 17;
  RxHeader.DataLength = LengthCoder(32);    
  
  write_can_frame(&gaga, &RxHeader, dummy_buffer);     

  *********************************************/
  
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  
/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
  
  /* USER CODE BEGIN SysInit */
  __HAL_RCC_D2SRAM1_CLK_ENABLE(); // have no idea how to do this from the CubeMX software.
  HAL_Delay(1000); // does not work without this line. I believe it's linked to lan8720 start-up time. 
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_LWIP_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_SPI_Receive_DMA(&hspi3, SPI_RX_body, 1024);
  
  for( int i = 0; i < 512; i++)
  {
    SPI_TX_buf[i] = i;
  }
  
  udp_client_connect();
  udp_lcm_connect();
  
  HAL_TIM_Base_Start_IT(&htim7); // enable microseconds timesource
  HAL_TIM_Base_Start_IT(&htim1);
  
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Filter for messages from master to this dedicated device. 
  FDCAN_FilterTypeDef sFilterConfig;  
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x1FFFFFFF;  
  
  // iterate over three CAN bus handles and enable their parameters
  for( int i = 0; i < 3; i++)
  {
    // choose one FDCAN Handle from the three available
    FDCAN_HandleTypeDef * FDCAN_Handle = FDCAN_Handles_Map[i];
    
    if( FDCAN_Handle != NULL )
    {
      // Configure CAN frames filtering 
      if( HAL_FDCAN_ConfigFilter(FDCAN_Handle, &sFilterConfig) != HAL_OK ){ Error_Handler(); }  
      
      // Configure global filter: Filter all remote frames with STD and EXT ID. Reject non matching frames with STD ID and EXT ID
      if( HAL_FDCAN_ConfigGlobalFilter(FDCAN_Handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK ){ Error_Handler(); }
      
      // Configure and enable Tx Delay Compensation, required for BRS mode.
      if( HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_Handle, 5, 0) != HAL_OK){ Error_Handler(); }
      if( HAL_FDCAN_EnableTxDelayCompensation(FDCAN_Handle) != HAL_OK){ Error_Handler(); }
      
      // Activate Rx FIFO 0 new message notification
      //if( HAL_FDCAN_ActivateNotification(FDCAN_Handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK ){ Error_Handler(); }      
      
      // Activate TX FIFO empty notification
      if( HAL_FDCAN_ActivateNotification( FDCAN_Handle, 
                                          FDCAN_IT_TX_FIFO_EMPTY, 
                                          FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK )
                                          { Error_Handler(); }

      // Start FDCAN periphery
      if( HAL_FDCAN_Start(FDCAN_Handle) != HAL_OK ){ Error_Handler(); }
    }
  }
  ///////////////////////////////////////////////////////////////////////////
  
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t data[8] = {'a','b','c','d','e','f','g','h'};
  
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0)
  {
    // Add message to Tx FIFO 
    TxHeader.Identifier = 0x1;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {

  }
  
  /*
  CanardRxSubscription heartbeat_msg_subscription;
  if( canardRxSubscribe(        &canard,
                                CanardTransferKindMessage,
                                uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                                uavcan_node_Heartbeat_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &heartbeat_msg_subscription) != 1 ){ Error_Handler(); }  
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t message_transfer = 0;
  uint32_t timestamp = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( HAL_GetTick() > timestamp )
    {
      //LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_6);
      //HAL_SPI_Transmit_DMA(&hspi1, SPI_TX_buf, 512);
      
      //conke(); // debug transmition of sample data
      timestamp += 1000;
    }
    
    MX_LWIP_Process();
    
    if( LCM_rx_flag )
    {
      uint8_t c_serialized[uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_] = {0};
      
      uavcan_primitive_array_Real32_1_0 uavcan_tx_array;
      uavcan_tx_array.value.count = 5;
      
      for( int i = 0; i < 12; i++)
      {
        if( device_to_queue[i] >= 0 ) // check if selected drive is ebabled 
        {
          uavcan_tx_array.value.elements[0] = rx_lcm_msg.act[i].position;
          uavcan_tx_array.value.elements[1] = rx_lcm_msg.act[i].velocity;
          uavcan_tx_array.value.elements[2] = rx_lcm_msg.act[i].torque;
          uavcan_tx_array.value.elements[3] = rx_lcm_msg.act[i].kp;
          uavcan_tx_array.value.elements[4] = rx_lcm_msg.act[i].kd;
          
          size_t c_serialized_size = uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_;

          if ( uavcan_primitive_array_Real32_1_0_serialize_( &uavcan_tx_array, c_serialized, &c_serialized_size) < 0)
          {
            Error_Handler();
          }
          
          const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityHigh,
                                                                .transfer_kind  = CanardTransferKindMessage,
                                                                .port_id        = device_tx_port[i],
                                                                .remote_node_id = CANARD_NODE_ID_UNSET,
                                                                .transfer_id    = message_transfer }; 

          if(canardTxPush(  TxQueuesMap[device_to_queue[i]],
                            &canard,
                            0,
                            &transfer_metadata,
                            c_serialized_size,
                            c_serialized) < 0 )
                            {
                              Error_Handler();
                            }
          
          process_canard_TX_queue( device_to_queue[i] );
        }
      }
      
      message_transfer++ ;
      LCM_rx_flag = 0;
      
      response_recorder = 0;
    }
    
    if( LCM_tx_flag )
    {
      conke();
      LCM_tx_flag = 0;
    }
    
    /*
    if( gaga.bytes_written > 70 )
    {
      udp_client_send();
      __HAL_TIM_SET_COUNTER(&htim1, 0);
    }
    else if( timer_update_flag )
    {
      udp_client_send();
      timer_update_flag = 0;
    }
    */
    
    /// retrieve data from the FDCAN peripherals in polling mode
    FDCAN_RxHeaderTypeDef RxHeader = {0};
    uint8_t RxData[64];
    
    while( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0 )
    {
      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){ Error_Handler(); }
      write_can_frame(&FDCAN1_RX_buf, 0, &RxHeader, RxData);
    }
    
    while( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0 )
    {
      if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){ Error_Handler(); }
      write_can_frame(&FDCAN2_RX_buf, 1, &RxHeader, RxData);
    }
    
    while( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0) > 0 )
    {
      if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){ Error_Handler(); }
      write_can_frame(&FDCAN3_RX_buf, 2, &RxHeader, RxData);
    }
    
    /// processing of the received FDCAN frames from companion (via SPI) chip
    while( SPI_RX_buf.bytes_written > 0 )
    {
      uint8_t local_buffer[69] = {0};
      uint8_t frame_length = 0;
      
      uint32_t primask_bit = __get_PRIMASK();  // backup PRIMASK bit
      __disable_irq();                  // Disable all interrupts by setting PRIMASK bit on Cortex
      
        frame_length = SPI_RX_buf.buffer_body[SPI_RX_buf.head]; // get the frame length, it's located at the beginning of new frame in the buffer
        if( frame_length > SPI_RX_buf.bytes_written )
        {
          // buffer is corrupted
          Error_Handler();
        }
        read_buffer(&SPI_RX_buf, local_buffer, frame_length);
        
      __set_PRIMASK(primask_bit);     // Restore PRIMASK bit
      
      process_vb_rx_frame(local_buffer, frame_length); // filter frames into the LCM and UAVCAN -related buffers
      
      /// the rest is debug errors check
      if( frame_length != 13 )
      {
        Error_Handler();
      }

      if( previos_char == local_buffer[10] )
      {
        Error_Handler();
      }
      previos_char = local_buffer[10];
      
      debug_collector[debug_index] = previos_char;
      debug_index++;
      if( debug_index > 1023 )
      {
        debug_index = 0;
      }
    }
    
    /// processing of the received FDCAN frames from the main chip
    // if there's any non-empty RX-buffer
    if( FDCAN1_RX_buf.bytes_written || FDCAN2_RX_buf.bytes_written || FDCAN3_RX_buf.bytes_written )
    {
      for( int i = 0; i < 3; i++ )
      {
        buffer_instance * buffer = RX_Buffers_Map[i];

        while( buffer->bytes_written > 0 )
        {
          uint8_t local_buffer[69] = {0};
          uint8_t frame_length = 0;

          frame_length = buffer->buffer_body[buffer->head]; // get the frame length, it's located at the beginning of new frame in the buffer
          if( frame_length > buffer->bytes_written )
          {
            // buffer is corrupted
            Error_Handler();
          }
          read_buffer(buffer, local_buffer, frame_length);

          process_vb_rx_frame(local_buffer, frame_length); // filter frames into the LCM and UAVCAN -related buffers
        }
      }
    }
    
    /// break UDP TX ring buffer into a bunch of UDP frames each smaller than 512 bytes and send them out
    /// right now data is sent only when buffer is full
    while( UDP_TX_ring.bytes_written > 0 )
    {
      uint16_t length = UDP_TX_ring.buffer_body[UDP_TX_ring.head];
      if( index + length < 512 )
      {
         read_buffer(&UDP_TX_ring, &udp_out_buffer[index], length);
         index += length;
      }
      else
      {
        //send buffer
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, index, PBUF_RAM); // allocate LWIP memory for outgoing UDP packet
        
        if (p != NULL)
        {
          pbuf_take(p, (void *) udp_out_buffer, index);
          udp_send(upcb, p);
          pbuf_free(p);         
          index = 0;
        }
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 40;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 24;
  hfdcan1.Init.NominalTimeSeg1 = 55;
  hfdcan1.Init.NominalTimeSeg2 = 24;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* Configure and enable Tx Delay Compensation, required for BRS mode.
   TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
   TdcFilter default recommended value: 0 */
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Activate Rx FIFO 0 new message notification
  if( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK )
  {
    Error_Handler();
  }
  
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 24;
  hfdcan2.Init.NominalTimeSeg1 = 55;
  hfdcan2.Init.NominalTimeSeg2 = 24;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 5;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.MessageRAMOffset = 768;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.RxFifo0ElmtsNbr = 32;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  
  /* Configure and enable Tx Delay Compensation, required for BRS mode.
   TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
   TdcFilter default recommended value: 0 */
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  
  if( HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK )
  {
    Error_Handler();
  }
  
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 1;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 2;
  hfdcan3.Init.NominalTimeSeg2 = 2;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 0;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 0;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 479;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 240;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
uint16_t findDeviceIndex(uint16_t *array, size_t size, uint16_t target) 
{
    uint16_t i=0;
    while((i<size) && (array[i] != target)) i++;

    return (i<size) ? (i) : (-1);
}

void process_vb_rx_frame(uint8_t *local_buffer, uint8_t frame_length)
{
  uint32_t bus_id = 0;
  memcpy(&bus_id, &local_buffer[1], 4);
  //uint8_t bus_num = decode_bus_num( bus_id );
  //if( bus_num > 2 ){ Error_Handler(); } // only for debugging purposes

  CanardFrame rxf;

  rxf.extended_can_id = decode_can_id( bus_id );
  rxf.payload_size = (size_t)(frame_length - 5);
  rxf.payload = (void*)&local_buffer[5];

  CanardRxSubscription* out_subscription = NULL;
  CanardRxTransfer transfer;
  
  int8_t result = canardRxAccept(       &canard,
                                        micros(),
                                        &rxf,
                                        0,
                                        &transfer,
                                        &out_subscription);
  
  if( out_subscription != NULL ) // if we did subscribe to this subject we keep the frame and wait for the completion of transfer
  {
    if( result == 1 )
    {
      uint16_t index = findDeviceIndex( (uint16_t *)device_rx_port, 12, (uint16_t)transfer.metadata.port_id );
      
      if( device_node_id[index] == transfer.metadata.remote_node_id )
      {
        uavcan_primitive_array_Real32_1_0 array;
        size_t array_ser_buf_size = uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_;

        if ( uavcan_primitive_array_Real32_1_0_deserialize_( &array, transfer.payload, &array_ser_buf_size) < 0 )
        {
          Error_Handler();
        }

        tx_lcm_msg.act[index].position = array.value.elements[0];
        tx_lcm_msg.act[index].velocity = array.value.elements[1];
        tx_lcm_msg.act[index].torque = array.value.elements[2];
        
        response_recorder |= 1UL << index; // set bit corresponding to the device answered

        // check if every device returned an answer
        if( response_recorder == 0x3 )
        {
          LCM_tx_flag = 1;
          
          response_recorder = 0;
        }
      }
      else
      {
        Error_Handler();
      }
      // parse the transfer and load value into the TX LCM message 
      //if( transfer.metadata.remote_node_id == 1 ) // something happens I quess
      canard.memory_free(&canard, transfer.payload);      // Deallocate the dynamic memory afterwards.
    }
  }
  else // we did not subscribe to this subject - pass it up
  {
    // load frame into the UDP TX buffer
    write_buffer(&UDP_TX_ring, local_buffer, frame_length);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM1 )
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    //TIM1_Callback();
    timer_update_flag = 1;
  }
}

// https://stackoverflow.com/a/25004214
uint8_t findIndex(FDCAN_HandleTypeDef *array, size_t size, FDCAN_HandleTypeDef* target) 
{
    uint8_t i=0;
    while((i<size) && (&array[i] != target)) i++;

    return (i<size) ? (i) : (-1);
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
  // this callback shouldnt be called fof disabled CAN buses, so no need to check for NULL pointer
  
  // find bus number from the handle name 
  uint8_t bus_num = findIndex(*FDCAN_Handles_Map, 3, hfdcan);
  buffer_instance *TX_buffer = TX_Buffers_Map[bus_num];
  
  uint8_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
  while( free_level != 0 && TX_buffer->bytes_written > 0 )
  {
    uint8_t full_frame_length = TX_buffer->buffer_body[TX_buffer->head]; // the length of the frame is stored in it's head
    
    uint8_t local_buffer[69] = {0};
    
    // this code can be interrupted only by other CAN periph blocks so no need of critical section. 
    if( read_buffer(TX_buffer, local_buffer, full_frame_length) )
    {
      return ;
    }

    if( push_can_frame(hfdcan, local_buffer, full_frame_length) == 1 )
    {
      Error_Handler();
    }
    
    free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
  }
  
  return ;
}

// push VB CAN frame into CAN FIFO
uint8_t push_can_frame( FDCAN_HandleTypeDef *handle, uint8_t *frame_data, uint8_t frame_full_size)
{
  uint32_t bus_id = 0;
  memcpy(&bus_id, &frame_data[1], 4);

  uint8_t bus_num = decode_bus_num( bus_id );
  uint32_t id = decode_can_id( bus_id );    

  if (HAL_FDCAN_GetTxFifoFreeLevel(handle) != 0)
  {
    FDCAN_TxHeaderTypeDef TxHeader;
    // Add message to Tx FIFO 
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = LengthCoder( frame_full_size - 5 );
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;
    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &TxHeader, &frame_data[5]) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_Delay(10);
  }
  else
  {
    return 1;
  }
  
  return 0;
}

void process_canard_TX_queue( uint8_t queue_num )
{
  // Look at top of the TX queue of individual CAN frames
  for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek( TxQueuesMap[queue_num] )) != NULL;)
  {
    if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > micros()))  // Check the deadline.
    {
      uint8_t vb_frame[69];
      uint8_t vb_frame_len = ti->frame.payload_size + 5;
      vb_frame[0] = vb_frame_len;

      uint8_t bus_num = queue_num;
      
      uint32_t id = ti->frame.extended_can_id;
      uint32_t bus_id = encode_bus_id( bus_num, id );
      
      memcpy( &vb_frame[1], &bus_id, 4);
      memcpy( &vb_frame[5], (uint8_t *)ti->frame.payload, ti->frame.payload_size);
      
      distribute_vb_frame( vb_frame );
    }
    // After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
    canard.memory_free(&canard, canardTxPop( TxQueuesMap[queue_num], ti));
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if( hspi->Instance == SPI1 )
  {
    send_frame_SPI();
  }
}

// push VB CAN frame into CAN FIFO
uint8_t send_frame_SPI(void)
{
  if( !LL_SPI_IsActiveMasterTransfer(SPI1) )
  {
    buffer_instance *s = &companion_TX_ins;

    if( s->bytes_written > 0 )
    {
      uint8_t frame_length = s->buffer_body[s->head]; // get the frame length, it's located at the beginning of new frame in the buffer
      if( frame_length > s->bytes_written )
      {
        // buffer is corrupted
        Error_Handler();
      }
      
      /*
      if( frame_length != 13 )
      {
        Error_Handler();
      }
      */
      
      read_buffer( s, SPI_TX_buf, frame_length);
      
      LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_6);
      HAL_SPI_Transmit_DMA(&hspi1, SPI_TX_buf, frame_length);      
    }
  }
  
  return 0;
}

uint8_t write_can_frame(buffer_instance * s, uint8_t src_bus, FDCAN_RxHeaderTypeDef * head, uint8_t *data)
{
  uint8_t can_message_length = LengthDecoder( head->DataLength ) + 5; // together with service bytes
  
  write_buffer(s, &can_message_length, 1); // write message length
  
  can_message_length -= 5; // 25 bytes with the service info

  uint32_t id = head->Identifier;
  uint32_t bus_id = encode_bus_id( src_bus, id );

  write_buffer(s, (uint8_t*)&bus_id, 4); // write message length
  write_buffer(s, data, can_message_length); // write message length
  
  return 0;
}

uint8_t read_can_frame(buffer_instance * s, FDCAN_TxHeaderTypeDef * head, uint8_t *data)
{
  uint8_t can_message_length = 0;
  uint32_t bus_id = 0;
  uint8_t bus = 0;
  uint32_t id = 0;
  
  read_buffer(s, &can_message_length, 1);
  can_message_length -= 5;
  read_buffer(s, (uint8_t *)&bus_id, 4);
  read_buffer(s, data, can_message_length);

  head->DataLength = LengthCoder( can_message_length );
  head->Identifier = decode_can_id( bus_id );
  // process can frame
  
  return 0;
}

// allocate dynamic memory of desired size in bytes
static void *memAllocate(CanardInstance *const canard, const size_t amount)
{
  (void)canard;
  return malloc(amount);
}

// free allocated memory
static void memFree(CanardInstance *const canard, void *const pointer)
{
  (void)canard;
  free(pointer);
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30004000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
