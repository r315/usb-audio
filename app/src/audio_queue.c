#include <stdint.h>
#include "audio_queue.h"

/**
 * @brief Initialize the audio queue with a buffer and size.
 *
 * @param queue
 * @param buffer
 * @param size
 */
void audio_queue_init(audio_queue_t *q, uint16_t *buffer, uint16_t element_size, uint16_t buffer_size)
{
    if (q == NULL || buffer == NULL || element_size == 0 || buffer_size == 0) {
        // Handle error: invalid parameters
        return;
    }

    // Find maximum elements that can fit on buffer size.
    q->end = q->start = buffer;
    while(q->end + element_size < q->start + buffer_size){
        q->end += element_size;
    }

    q->size = q->end - q->start;
    q->read = q->start;
    q->write = q->start;
}

void audio_queue_flush(audio_queue_t *q)
{
    q->read = q->write = q->start;
}

uint16_t audio_queue_enqueue(audio_queue_t *q, const uint16_t *data, uint16_t len)
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

uint16_t audio_queue_dequeue(audio_queue_t *q, uint16_t *data, uint16_t len)
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

uint16_t audio_queue_count(const audio_queue_t *q)
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
uint16_t audio_queue_size(const audio_queue_t *q)
{
    return q->size;
}