//
// Created by zero on 2026/2/22.
//

#ifndef CHASE_LIGHT_OS_RING_BUFFER_H
#define CHASE_LIGHT_OS_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 256
#define BUFFER_MASK (BUFFER_SIZE - 1)

typedef struct
{
    uint8_t buffer[BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer_t;

bool RingBuffer_Pop_Task(RingBuffer_t *rb, uint8_t *data);

#endif //CHASE_LIGHT_OS_RING_BUFFER_H