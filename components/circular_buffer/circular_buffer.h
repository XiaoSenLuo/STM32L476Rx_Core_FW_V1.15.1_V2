//
// Created by XIAOSENLUO on 2022/5/28.
//

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "stdint.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

struct circular_buffer_s {
    void * (*lowcpy)(void *, const void *, size_t);
    void * (*fastcpy)(void *, const void *, size_t);
};

typedef struct circular_buffer_s * circular_buffer_handle;

int circular_buffer_static_create(circular_buffer_handle *handle, uint8_t *buffer, int __size);
int circular_buffer_static_delete(circular_buffer_handle handle);

int circular_buffer_create(circular_buffer_handle *handle, int __size);
int circular_buffer_delete(circular_buffer_handle handle);

int circular_buffer_insert(circular_buffer_handle handle, uint8_t *idata, int __size);
int circular_buffer_pop(circular_buffer_handle handle, uint8_t *odata, int __size);

int circular_buffer_length(circular_buffer_handle handle);

int circular_buffer_clean(circular_buffer_handle handle);

#ifdef __cplusplus
}
#endif

#endif //CIRCULAR_BUFFER_H
