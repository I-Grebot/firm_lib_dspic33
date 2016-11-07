

#ifndef _FIFO_H_
#define	_FIFO_H_

// Required libraries for datatypes
#include <stdint.h> // for std. types
#include <stdlib.h> // for malloc
#include <string.h>  // for memcpy

// Generic fifo structure
typedef struct {
    size_t data_size;    // Size of an individual data
    uint16_t depth;      // Maximum allowed numbers of elements
    uint16_t cnt;        // Number of elements in the fifo
    void* data;          // Actual data holder
    uint16_t wr_idx;     // Head - Write pointer (next location to write into)
    uint16_t rd_idx;     // Tail - Read pointer (next location to read from)
} fifo_t;

// Some useful macros
#define FIFO_IS_EMPTY(_fifo) ((_fifo)->cnt == 0)
#define FIFO_IS_FULL(_fifo)  ((_fifo)->cnt == (_fifo)->depth-1)

// Prototypes
fifo_t* fifo_new(uint16_t fifo_depth, size_t data_size);
void fifo_clear(fifo_t* fifo);
uint8_t fifo_push(fifo_t* fifo, void* data);
uint8_t fifo_get_tail(fifo_t* fifo, void* data);
uint8_t fifo_pull(fifo_t* fifo, void* data);

#endif	/* _FIFO_H_ */

