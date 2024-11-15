#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <mutex>

typedef struct {
    std::mutex float_array_ptr_mutex;
    std::mutex access_gpu_mutex;
} Thread_Control_Flag;

#endif 