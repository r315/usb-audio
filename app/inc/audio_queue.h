
#ifndef AUDIO_QUEUE_H
#define AUDIO_QUEUE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t *start;
    uint16_t *end;
    uint16_t size;
    volatile uint16_t *read;
    volatile uint16_t *write;
} audio_queue_t;

/**
 * @brief Initialize the audio queue.
 * @param queue Pointer to the audio queue structure.
 */
void audio_queue_init(audio_queue_t *queue, uint16_t buffer, uint16_t element_size, uint16_t buffer_size);

/**
 * @brief Enqueue a byte into the audio queue.
 * @param queue Pointer to the audio queue structure.
 * @param data Byte to enqueue.
 * @return 0 on success, -1 if the queue is full.
 */
int audio_queue_enqueue(audio_queue_t *queue, uint8_t data);

/**
 * @brief Dequeue a byte from the audio queue.
 * @param queue Pointer to the audio queue structure.
 * @param data Pointer to store the dequeued byte.
 * @return 0 on success, -1 if the queue is empty.
 */
int audio_queue_dequeue(audio_queue_t *queue, uint8_t *data);

/**
 * @brief Check if the audio queue is empty.
 * @param queue Pointer to the audio queue structure.
 * @return 1 if empty, 0 otherwise.
 */
int audio_queue_is_empty(const audio_queue_t *queue);

/**
 * @brief Check if the audio queue is full.
 * @param queue Pointer to the audio queue structure.
 * @return 1 if full, 0 otherwise.
 */
int audio_queue_is_full(const audio_queue_t *queue);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_QUEUE_H