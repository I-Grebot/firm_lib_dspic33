

#include "fifo.h"

// Init, creates a new FIFO and returns a pointer to it.
// Returns 0 if an error occured
fifo_t* fifo_new(uint16_t fifo_depth, size_t data_size) {

  fifo_t* new_fifo;

  // Check for input argumenets
  if(fifo_depth == 0 || data_size == 0) {
    return 0;
  }
  
  // Create a new fifo
  new_fifo = malloc(sizeof(fifo_t));

  // There might be an error during new_fifo alloc
  if(new_fifo == NULL) {
    return 0;
  }

  // Allocate the required space for the fifo
  new_fifo->data = (void*) malloc(fifo_depth * data_size);

  // An error occured during the data array allocation
  // free the newly created fifo and returns NULL
  if(new_fifo->data == NULL) {
    free(new_fifo);
    return 0;
  }

  // Initialize the rest of the fifo structure
  new_fifo->depth = fifo_depth;
  new_fifo->data_size = data_size;
  new_fifo->cnt = 0;
  new_fifo->wr_idx = 0;
  new_fifo->rd_idx = 0;

  return new_fifo;  
}

// Clear a fifo
void fifo_clear(fifo_t* fifo) {
  free(fifo->data);
  free(fifo);
}

// Add a new data to the FIFO
// Return 0 if an error occured
uint8_t fifo_push(fifo_t* fifo, void* data) {

  void* head;
  
  // Ensure that fifo is not full
  if(FIFO_IS_FULL(fifo)) {
    return 0;
  }

  // Pointer to the head (new data to be written)
  head = fifo->data + (fifo->data_size * fifo->wr_idx);

  // Copy the data content to the next available slot (head)
  memcpy(head, data, fifo->data_size);

  // Increment the write index of the circular buffer
  fifo->cnt++;
  if(fifo->wr_idx == fifo->depth - 1) {
    fifo->wr_idx = 0;
  } else {
    fifo->wr_idx++;
  }

  // No error
  return -1;
}

// Get the tail data (but do not pull it)
uint8_t fifo_get_tail(fifo_t* fifo, void* data) {

  void* tail;

  // Ensure that the fifo is not empty
  if(FIFO_IS_EMPTY(fifo)) {
    return 0;
  }

  // Pointer to the tail (old data to be read)
  tail = fifo->data + (fifo->data_size * fifo->rd_idx);

  // Copy the FIFO content to the return value
  memcpy(data, tail, fifo->data_size);

  return 1;
}

// Retrieve a data from the FIFO
// Return 0 if an error occured
uint8_t fifo_pull(fifo_t* fifo, void* data) {  

  // Ensure that the fifo is not empty
  if(FIFO_IS_EMPTY(fifo)) {
    return 0;
  }

  // Get the tail
  fifo_get_tail(fifo, data);

  // Increment the read index of the circular buffer
  fifo->cnt--;
  if(fifo->rd_idx == fifo->depth - 1) {
    fifo->rd_idx = 0;
  } else {
    fifo->rd_idx++;
  }

  // No error
  return -1;  
}

