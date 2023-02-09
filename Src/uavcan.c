#include "uavcan.h"
#include "main.h"
#include "net.h"
#include "helpers.h"

#include "libcanard/canard.h"
#include "uavcan/primitive/array/Real32_1_0.h"

#include "hl_command_msg.h"
#include "hl_state_msg.h"

extern uint8_t LCM_tx_flag;
extern hl_command_msg rx_lcm_msg;
extern measurment tx_lcm_msg;

void process_canard_TX_queue( uint8_t queue_num );

static void *memAllocate(CanardInstance *const canard, const size_t amount);
static void memFree(CanardInstance *const canard, void *const pointer);

CanardInstance 	canard; // This is the core structure that keeps all of the states and allocated resources of the library instance

// each queue is bound to its dedicated can-bus - 3 on the main and 3 on the companion chip.
CanardTxQueue 	queue0; // Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface
CanardTxQueue 	queue1;
CanardTxQueue 	queue2;
CanardTxQueue 	queue3;
CanardTxQueue 	queue4;
CanardTxQueue 	queue5;

CanardTxQueue * TxQueuesMap[6] = { &queue0, &queue1, &queue2, &queue3, &queue4, &queue5 };

typedef struct Bepis
{
  CanardNodeID node_id;
  uint8_t queue_num;
  CanardPortID rx_port; // driver listens this port
  CanardPortID tx_port; // driver writes into this port
  CanardRxSubscription sub;
} Bepis;

// to disable device set its node_id to zero             
Bepis sukea[12] = 
{ 
  { 0, 0, 1000, 1001}, // 0 drive
  { 0, 1, 1010, 1011}, // 1 drive
  { 0, 1, 1000, 1001}, // 2 drive
  {10, 1, 1000, 1001}, // 3 drive
  { 0, 1, 1000, 1001}, // 4 drive
  { 0, 1, 1000, 1001}, // 5 drive
  { 0, 1, 1000, 1001}, // 6 drive
  { 0, 1, 1030, 1031}, // 7 drive
  { 0, 1, 1000, 1001}, // 8 drive
  { 0, 1, 1000, 1001}, // 9 drive
  { 0, 1, 1000, 1001}, // 10 drive
  { 0, 1, 1000, 1001}, // 11 drive
 };

CanardPortID index_to_port( uint8_t index )
{
  return sukea[index].rx_port;
}

// not sure it is legal
uint8_t sub_to_index( CanardRxSubscription* sub )
{
  return ( (uint32_t)sub - (uint32_t)&sukea) / ((uint32_t)sizeof(Bepis));
}

uint16_t response_mask = 0;
void UAVCAN_setup(void)
{
  canard = canardInit(&memAllocate, &memFree);	// Initialization of a canard instance
  canard.node_id = 40;
  
  for( int i = 0; i < 6; i++ )
  {
    *TxQueuesMap[i] = canardTxInit( 10, CANARD_MTU_CAN_FD); // really we need 1 element only
  }
  
  for( int i = 0; i < 12; i++ )
  {
    if( sukea[i].node_id != 0 )
    {
      if( canardRxSubscribe(    &canard,
                                CanardTransferKindMessage,
                                sukea[i].tx_port,
                                uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &sukea[i].sub ) != 1 ){ Error_Handler(); }
      
      response_mask |= 1UL << i; // set bit corresponding to the subscribed device
    }
  }

  return ;
}

uint16_t response_recorder = 0;
uint8_t message_transfer = 0;
void UAVCAN_send(void)
{
  uint8_t c_serialized[uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_] = {0};
  
  uavcan_primitive_array_Real32_1_0 uavcan_tx_array;
  uavcan_tx_array.value.count = 5;
  
  for( int i = 0; i < 12; i++)
  {
    if( sukea[i].node_id != 0 ) // check if selected drive is ebabled 
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
                                                            .port_id        = sukea[i].rx_port,
                                                            .remote_node_id = CANARD_NODE_ID_UNSET,
                                                            .transfer_id    = message_transfer }; 

      if(canardTxPush(  TxQueuesMap[sukea[i].queue_num],
                        &canard,
                        0,
                        &transfer_metadata,
                        c_serialized_size,
                        c_serialized) < 0 )
                        {
                          Error_Handler();
                        }
      
      process_canard_TX_queue( sukea[i].queue_num );
    }
  }
  
  message_transfer++ ;
  response_recorder = 0;
}

uint8_t parse_canard_frame( uint32_t id, size_t size, void* payload)
{
  CanardFrame rxf;
  rxf.extended_can_id = id;
  rxf.payload_size = size;
  rxf.payload = payload;

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
    if( result == 1 ) // process message if it is the complete transfer
    {
      uint16_t index = sub_to_index( out_subscription );
      
      if( sukea[index].tx_port == transfer.metadata.port_id )
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
        if( response_recorder == response_mask )
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
      return 2;
    }
    else
    {
      // the frame is UAVCAN related, but it does not complete any transfer - do nothing for now. 
      return 1;
    }
  }
  else // we did not subscribe to this subject - pass it up
  {
    return 0;
  }
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