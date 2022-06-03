//
// Created by XIAOSENLUO on 2022/5/28.
//

#include "circular_buffer.h"
#include "stdlib.h"


typedef struct circular_buffer_def_s{
    void * (*lowcpy)(void *, const void *, size_t);
    void * (*fastcpy)(void *, const void *, size_t);
    uint8_t *data;
    int capacity;
    int head;
    int tail;
    int length;
    int lock;
}circular_buffer_def_t;


int circular_buffer_static_create(circular_buffer_handle *handle, uint8_t *buffer, int __size){
    circular_buffer_def_t *cbuffer = NULL;
    cbuffer = (circular_buffer_def_t *)malloc(sizeof(circular_buffer_def_t));
    if(!cbuffer) return 1;
    cbuffer->lowcpy = NULL;
    cbuffer->fastcpy = NULL;
    cbuffer->data = NULL;
    cbuffer->head = 0;
    cbuffer->tail = 0;
    cbuffer->capacity = __size;
    cbuffer->length = 0;

    cbuffer->data = buffer;

    *handle = (circular_buffer_handle)cbuffer;
    return 0;
}

int circular_buffer_static_delete(circular_buffer_handle handle){
    circular_buffer_def_t *chandle = (circular_buffer_def_t *)handle;
    free(chandle);
    return 0;
}

int circular_buffer_create(circular_buffer_handle *handle, int __size){
    circular_buffer_def_t *cbuffer = NULL;
    cbuffer = (circular_buffer_def_t *)malloc(sizeof(circular_buffer_def_t));
    if(!cbuffer) return 1;
    cbuffer->lowcpy = NULL;
    cbuffer->fastcpy = NULL;
    cbuffer->data = NULL;
    cbuffer->head = 0;
    cbuffer->tail = 0;
    cbuffer->capacity = __size;
    cbuffer->length = 0;
    cbuffer->lock = 0;

    cbuffer->data = (uint8_t *)malloc(__size);
    if(!cbuffer->data){
        free(cbuffer);
        return 2;
    }

    *handle = (circular_buffer_handle)cbuffer;
    return 0;
}

int circular_buffer_delete(circular_buffer_handle handle){
    circular_buffer_def_t *chandle = (circular_buffer_def_t *)handle;
    free(chandle->data);
    free(chandle);
    return 0;
}

int circular_buffer_insert(circular_buffer_handle handle, uint8_t *idata, int __size){
    circular_buffer_def_t *cbuffer = (circular_buffer_def_t *)handle;
    int is = 0;

    if(cbuffer->lock) return -1;
    cbuffer->lock = 1;
    if(cbuffer->length == cbuffer->capacity) return -1;

    is = (__size < (cbuffer->capacity - cbuffer->length)) ? __size : (cbuffer->capacity - cbuffer->length);

    if(cbuffer->fastcpy){
        int remain = cbuffer->capacity - cbuffer->tail;
        if(is < remain){
        	cbuffer->fastcpy((void*)(cbuffer->data + cbuffer->tail), idata, is);
        	cbuffer->tail += is;
        }else{
        	cbuffer->fastcpy((void*)(cbuffer->data + cbuffer->tail), idata, remain);
        	cbuffer->fastcpy((void*)(cbuffer->data), (void*)(idata + remain), is - remain);
        	cbuffer->tail = is - remain;
        }
    }else{
        for(int i = 0; i < is; i++){
            if((cbuffer->tail) == cbuffer->capacity) cbuffer->tail = 0;
            *(uint8_t*)(cbuffer->data + cbuffer->tail) = idata[i];
            cbuffer->tail++;
        }
    }

    cbuffer->length += is;
    cbuffer->lock = 0;
    return is;
}

int circular_buffer_pop(circular_buffer_handle handle, uint8_t *odata, int __size){
    circular_buffer_def_t *cbuffer = (circular_buffer_def_t *)handle;
    int os = 0;

    if(cbuffer->lock) return -1;
    cbuffer->lock = 1;
    if(cbuffer->length == 0){
        cbuffer->lock = 0;
        return 0;
    }

    os = cbuffer->length;
    if(__size < cbuffer->length) os = __size;

    if(cbuffer->fastcpy){
        int remain = cbuffer->capacity - cbuffer->head;
        if(os < remain){
        	cbuffer->fastcpy(odata, (void*)(cbuffer->data + cbuffer->head), os);
        	cbuffer->head += os;
        }else{
        	cbuffer->fastcpy(odata, (void*)(cbuffer->data + cbuffer->head), remain);
        	cbuffer->fastcpy((void *)(odata + remain), (void*)(cbuffer->data), os - remain);
        	cbuffer->head = os - remain;
        }
    }else{
        for(int i = 0; i < os; i++){
            if(cbuffer->head == cbuffer->capacity){
                cbuffer->head = 0;
            }
            odata[i] = *(uint8_t*)(cbuffer->data + cbuffer->head);
            cbuffer->head++;
        }
    }
    cbuffer->length -= os;

    cbuffer->lock = 0;
    return os;
}


int circular_buffer_length(circular_buffer_handle handle){
    circular_buffer_def_t *cbuffer = (circular_buffer_def_t *)handle;
    return cbuffer->length;
}


int circular_buffer_clean(circular_buffer_handle handle){
    circular_buffer_def_t *cbuffer = (circular_buffer_def_t *)handle;
    cbuffer->head = 0;
    cbuffer->tail = 0;
    cbuffer->length = 0;
    cbuffer->lock = 0;
    return 0;
}
