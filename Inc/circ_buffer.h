#ifndef CIRC_BUFFER_H
#define CIRC_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#define buf_size 128

typedef struct buffer_instance
{
  uint8_t bytes_written;
  uint8_t head; // pointer to the first byte of the buffer, non-empty. *from where to read*
  uint8_t tail; // pointer to the first empty byte at the buffers end. *where to write*
  uint8_t *buffer_body;
} buffer_instance;

uint8_t read_buffer(buffer_instance * s, uint8_t * local_buffer, uint8_t num_to_read);
uint8_t write_buffer(buffer_instance * s, uint8_t * local_buffer, uint8_t num_to_write);
uint16_t update_index( uint16_t index, uint16_t buffer_size, uint16_t shift);

#ifdef __cplusplus
}
#endif

#endif  // this file will only ever be copied in once to another file