//
// Created by zero on 2026/2/22.
//

#include "../Inc/ring_buffer.h"
#include "cmsis_compiler.h"

RingBuffer_t rx_buffer = {.head = 0 , .tail = 0 };

bool RingBuffer_Push_ISR(RingBuffer_t *rb,uint8_t data)
{
    //预判下一个写位置
    uint32_t next_head = (rb->head + 1) & BUFFER_MASK;

    //如果下一个位置撞上了读位置，说明缓冲区满了
    if (next_head == rb->tail)
    {
        return false;//爆栈了
    }

    rb->buffer[rb->head] = data;//存入数据
    __DMB();
    rb->head = next_head;       //更新写指针

    return true;
}

bool RingBuffer_Pop_Task(RingBuffer_t *rb, uint8_t *data)
{
    //如果头尾相等说明没有新数据
    if (rb->head == rb->tail)
    {

        return false;
    }

    *data = rb->buffer[rb->tail];               //取出数据
    rb->tail = (rb->tail + 1) & BUFFER_MASK;    //更新读指针

    return true;
}
