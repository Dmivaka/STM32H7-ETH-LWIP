#include "uavcan.h"
#include "main.h"
#include "net.h"
#include "helpers.h"

#include "libcanard/canard.h"
#include "uavcan/primitive/array/Real32_1_0.h"
#include "uavcan/primitive/Empty_1_0.h"

#include "servo_state_msg.h"
#include "servo_cmd_msg.h"

extern uint8_t LCM_tx_flag;
extern servo_cmd_msg rx_lcm_msg;
extern servo_state_msg tx_lcm_msg;

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

typedef struct driver_struct
{
  CanardNodeID node_id;
  uint8_t queue_num;
  CanardPortID rx_port; // driver listens this port
  CanardPortID tx_port; // driver writes into this port
  CanardRxSubscription sub;
} driver_struct;

// to disable device set its node_id to zero             
driver_struct drivers_map[12] = 
{ 
  { 12, 0, 1120, 1121}, // 0 drive
  { 11, 0, 1110, 1111}, // 1 drive
  { 10, 0, 1100, 1101}, // 2 drive
  { 22, 1, 1220, 1221}, // 3 drive
  { 21, 1, 1210, 1211}, // 4 drive
  { 20, 1, 1200, 1201}, // 5 drive
  { 32, 3, 1320, 1321}, // 6 drive
  { 31, 3, 1310, 1311}, // 7 drive
  { 30, 3, 1300, 1301}, // 8 drive
  { 42, 4, 1420, 1421}, // 9 drive
  { 41, 4, 1410, 1411}, // 10 drive
  { 40, 4, 1400, 1401}, // 11 drive
 };

CanardPortID index_to_port( uint8_t index )
{
  return drivers_map[index].rx_port;
}

// not sure if it's legal
uint8_t sub_to_index( CanardRxSubscription* sub )
{
  return ( (uint32_t)sub - (uint32_t)&drivers_map) / ((uint32_t)sizeof(driver_struct));
}

uint16_t response_mask = 0;
void UAVCAN_setup(void)
{
  canard = canardInit(&memAllocate, &memFree);	// Initialization of a canard instance
  canard.node_id = 100;

  for( int i = 0; i < 12; i++ )
  {
    if( drivers_map[i].node_id != 0 )
    {
      if( canardRxSubscribe(    &canard,
                                CanardTransferKindMessage,
                                drivers_map[i].tx_port,
                                uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &drivers_map[i].sub ) != 1 ){ Error_Handler(); }
      
      response_mask |= 1UL << i; // set bit corresponding to the subscribed device
      
      *TxQueuesMap[ drivers_map[i].queue_num ] = canardTxInit( 10, CANARD_MTU_CAN_FD); // really we need 1 element only
    }
  }

  return ;
}

uint8_t my_transfer = 0;
void UAVCAN_request(void)
{
  uavcan_primitive_Empty_1_0 request;
  
  uint8_t c_serialized;
  size_t c_serialized_size = uavcan_primitive_Empty_1_0_EXTENT_BYTES_;

  if ( uavcan_primitive_Empty_1_0_serialize_( &request, &c_serialized, &c_serialized_size) < 0)
  {
    Error_Handler();
  }
  
  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityHigh,
                                                        .transfer_kind  = CanardTransferKindMessage,
                                                        .port_id        = 900,
                                                        .remote_node_id = CANARD_NODE_ID_UNSET,
                                                        .transfer_id    = my_transfer }; 
  for( int i = 0; i < 6; i++ )
  {
    if( TxQueuesMap[i]->mtu_bytes == CANARD_MTU_CAN_FD )
    {
      if(canardTxPush(  TxQueuesMap[i],
                        &canard,
                        0,
                        &transfer_metadata,
                        c_serialized_size,
                        &c_serialized) < 0 )
                        {
                          Error_Handler();
                        }
      process_canard_TX_queue( i );
    }
  }

  my_transfer++ ;
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
    if( drivers_map[i].node_id != 0 ) // check if selected drive is ebabled 
    {
      uavcan_tx_array.value.elements[0] = rx_lcm_msg.position[i];
      uavcan_tx_array.value.elements[1] = rx_lcm_msg.kp[i];
      uavcan_tx_array.value.elements[2] = rx_lcm_msg.velocity[i];
      uavcan_tx_array.value.elements[3] = rx_lcm_msg.kd[i];
      uavcan_tx_array.value.elements[4] = rx_lcm_msg.torque[i]; 
      
      size_t c_serialized_size = uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_;

      if ( uavcan_primitive_array_Real32_1_0_serialize_( &uavcan_tx_array, c_serialized, &c_serialized_size) < 0)
      {
        Error_Handler();
      }
      
      const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityHigh,
                                                            .transfer_kind  = CanardTransferKindMessage,
                                                            .port_id        = drivers_map[i].rx_port,
                                                            .remote_node_id = CANARD_NODE_ID_UNSET,
                                                            .transfer_id    = message_transfer }; 

      if(canardTxPush(  TxQueuesMap[drivers_map[i].queue_num],
                        &canard,
                        0,
                        &transfer_metadata,
                        c_serialized_size,
                        c_serialized) < 0 )
                        {
                          Error_Handler();
                        }
      
      process_canard_TX_queue( drivers_map[i].queue_num );
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
      
      if( drivers_map[index].tx_port == transfer.metadata.port_id )
      {
        uavcan_primitive_array_Real32_1_0 array;
        size_t array_ser_buf_size = uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_;

        if ( uavcan_primitive_array_Real32_1_0_deserialize_( &array, transfer.payload, &array_ser_buf_size) < 0 )
        {
          Error_Handler();
        }

        tx_lcm_msg.position[index]= array.value.elements[0];
        tx_lcm_msg.velocity[index]= array.value.elements[1];
        tx_lcm_msg.torque[index]= array.value.elements[2];
        
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