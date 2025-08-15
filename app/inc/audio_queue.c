#include <stdint.h>
#include "audio_queue.h"

/**
 * @brief Initialize the audio queue with a buffer and size.
 *
 * @param queue
 * @param buffer
 * @param size
 */
void audio_queue_init(audio_queue_t *queue, uint16_t buffer, uint16_t element_size, uint16_t buffer_size)
{
    if (buffer == 0 || element_size == 0 || buffer_size == 0) {
        // Handle error: invalid parameters
        return;
    }

    // Find maximum elements that can fit on buffer size.
    queue->end = queue->start = buffer;
    while(queue->end + element_size < queue->start + buffer_size){
        queue->end += element_size;
    }

    queue->size = queue->end - queue->start;
    queue->read = queue->start;
    queue->write = queue->start;
}

void audio_queue_flush(audio_queue_t *q)
{
    q->read = q->write = q->start;
}

uint16_t audio_queue_push(audio_queue_t *q, const uint16_t *data, uint16_t len)
{
    uint16_t count = 0;
    while(count < len){
        *q->write++ = *data++;
        if(q->write >= q->end){
            q->write = q->start;
        }
        count++;
        if(q->write == q->read){
            break;
        }
    }
    return count;
}

uint16_t audio_queue_pop(audio_queue_t *q, uint16_t *data, uint16_t len)
{
    uint16_t count = 0;
    while(count < len){
        *data++ = *q->read++;
        if(q->read >= q->end){
            q->read = q->start;
        }
        count++;
        if(q->read == q->write){
            break;
        }
    }
    return count;
}

uint16_t audio_queue_count(audio_queue_t *q)
{
    return (q->write >= q->read)
        ? (q->write - q->read)
        : ((q->end - q->start) - (q->read - q->write));
}

/**
 * @brief Returns maximum number of elements queue can hold.
 *
 * @param q
 * @return uint16_t
 */
static uint16_t audio_queue_size(audio_queue_t *q)
{
    return q->size;
}