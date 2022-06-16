#include "circ_buffer.h"

extern uint8_t dummy_buffer[32];
extern uint8_t more_dummy_buffer[32];

uint8_t write_buffer(buffer_instance * s, uint8_t * local_buffer, uint8_t num_to_write)
{       
  if( buf_size - s->bytes_written > num_to_write )
  {
      int8_t start_index = s->tail; // can be negative
      uint8_t local_tail = s->tail;
      for( int i = 0; i < num_to_write; i++)
      {
          /// check on the end of buffer
          /// if so, the start index is moved to the beginning of the buffer.
          /// normally should be fired no more than once per write operation (otherwise the message is bigger than buffer).
          if( start_index + i > buf_size - 1 )
          {
              start_index -= buf_size;
          }
          s->buffer_body[ start_index + i ] = local_buffer[i];
      }

      s->bytes_written += num_to_write;

      local_tail += num_to_write;
      if( local_tail > buf_size )
      {
          local_tail -= buf_size;
      }
      s->tail = local_tail;
  }
  else
  {
    /// buffer is full!
    return 1;
  }
  return 0;
}

uint8_t read_buffer(buffer_instance * s, uint8_t * local_buffer, uint8_t num_to_read)
{
    if( num_to_read > s->bytes_written )
    {
        /// too much data is required to be read!
        return 1;
    }
    else
    {
        int8_t start_index = s->head; // can be negative
        uint8_t local_head = s->head;
        for( int i = 0; i < num_to_read; i++)
        {
            /// check on the end of buffer
            /// if so, the start index is moved to the beginning of the buffer.
            /// normally should be fired no more than once per write operation (otherwise the message is bigger than buffer).
            if( start_index + i > buf_size - 1 )
            {
                start_index -= buf_size;
            }
            local_buffer[i] = s->buffer_body[ start_index + i ];
        }

        s->bytes_written -= num_to_read;

        local_head += num_to_read;
        if( local_head > buf_size )
        {
            local_head -= buf_size;
        }
        s->head = local_head;
    }
    return 0;
}