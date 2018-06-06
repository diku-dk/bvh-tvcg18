#ifndef __BALANCE_THREADS
#error "__BALANCE_THREADS must be defined"
#endif // __BALANCE_THREADS

#ifndef __GLOBAL_WORK_QUEUE_CAPACITY
#error "__GLOBAL_WORK_QUEUE_CAPACITY must be defined"
#endif // __GLOBAL_WORK_QUEUE_CAPACITY

#ifndef __LOCAL_WORK_QUEUE_INIT_ITEMS
#error "__LOCAL_WORK_QUEUE_INIT_ITEMS must be defined"
#endif // __LOCAL_WORK_QUEUE_INIT_ITEMS

#ifndef __GLOBAL_WORK_QUEUES
#error "__GLOBAL_WORK_QUEUES must be defined"
#endif // __GLOBAL_WORK_QUEUES

#ifndef __TRAVERSAL_THREADS
#error "__TRAVERSAL_THREADS must be defined"
#endif // __TRAVERSAL_THREADS

#include "kdop_cl_error.clh"
#include "kdop_cl_work_item.clh"

__kernel void do_balance_work(
    __global const WorkItem     *restrict global_work_queue_in,
    __global       WorkItem     *restrict global_work_queue_out,
    __global       unsigned int *restrict global_work_queue_counts,
    __global       unsigned int *restrict total_entries,
    __global                int *restrict balance_signal
#ifndef __NDEBUG_DIKUCL
    ,          const    uint               global_work_queue_in_size
    ,          const    uint               global_work_queue_out_size
    ,          const    uint               global_work_queue_counts_size
    , __global volatile uint     *restrict error_codes
    , __global volatile uint     *restrict error_size          // must be initialized to 0
#endif // __NDEBUG_DIKUCL
    )
{
    __local int local_sum[__BALANCE_THREADS];
    __local int local_queue_sizes[__GLOBAL_WORK_QUEUES];
    __local int local_if_balance;
    const int local_id = get_local_id(0);

    int n_splits_left = __GLOBAL_WORK_QUEUES;
    int input_offset = local_id;
    GUARD(local_id, __BALANCE_THREADS, error_codes, error_size);
    local_sum[local_id] = 0;

    if(local_id == 0) {
        local_if_balance = 0;
    }
    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

    while(local_id < n_splits_left) {
        int n_queue_elements = global_work_queue_counts[input_offset];
        GUARD(input_offset, __GLOBAL_WORK_QUEUES, error_codes, error_size);
        local_queue_sizes[input_offset] = n_queue_elements;
        if(n_queue_elements < __LOCAL_WORK_QUEUE_INIT_ITEMS &&
           local_if_balance == 0) {
            (void) atomic_xchg(&local_if_balance, 1);
        } else if(n_queue_elements >=  __GLOBAL_WORK_QUEUE_CAPACITY - __TRAVERSAL_THREADS * 4 - __LOCAL_WORK_QUEUE_CAPACITY &&
                  local_if_balance == 0) {
            (void) atomic_xchg(&local_if_balance, 1);
        }
        GUARD(local_id, __BALANCE_THREADS, error_codes, error_size);
        local_sum[local_id] += n_queue_elements;
        input_offset += __BALANCE_THREADS;
        n_splits_left -= __BALANCE_THREADS;
    }
    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
    
    for(int r = __BALANCE_THREADS / 2; r != 0; r /= 2) {
        if(local_id < r) {
            GUARD(local_id, __BALANCE_THREADS, error_codes, error_size);
            GUARD(local_id + r, __BALANCE_THREADS, error_codes, error_size);
            local_sum[local_id] = local_sum[local_id] + local_sum[local_id + r];
        }
        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
    }

    n_splits_left = local_sum[0];

    if(local_id == 0) {
        *total_entries = n_splits_left;
        *balance_signal = local_if_balance;
    }
    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

    if(local_if_balance > 0) {
        int n_splits_per_queue, n_with_plus_one;
        n_splits_per_queue = n_splits_left / __GLOBAL_WORK_QUEUES;
        n_with_plus_one = n_splits_left - n_splits_per_queue * __GLOBAL_WORK_QUEUES;

        input_offset = 0;
        int output_offset = 0, input_queue = -1, input_queue_count = 0;

        for(int q = 0; q < __GLOBAL_WORK_QUEUES; ++q) {
            int n_splits_local;
            if(q < n_with_plus_one) {
                n_splits_local = n_splits_per_queue + 1;
            } else {
                n_splits_local = n_splits_per_queue;
            }

            output_offset = __GLOBAL_WORK_QUEUE_CAPACITY * q + local_id;

            if(local_id == 0) {
                GUARD(q, global_work_queue_counts_size, error_codes, error_size);
                global_work_queue_counts[q] = n_splits_local;
            }

            while(n_splits_local > 0) {
                if(input_queue_count <= 0) {
                    input_queue++;
                    input_offset = local_id;
                    GUARD(input_queue, __GLOBAL_WORK_QUEUES, error_codes, error_size);
                    input_queue_count = local_queue_sizes[input_queue];
                } else {
                    int splits_to_write = min(n_splits_local, input_queue_count);
                    splits_to_write = min(splits_to_write, __BALANCE_THREADS);

                    if(local_id < splits_to_write) {
                        GUARD(output_offset, global_work_queue_out_size, error_codes, error_size);
                        GUARD(input_queue * __GLOBAL_WORK_QUEUE_CAPACITY + input_offset, global_work_queue_in_size, error_codes, error_size);
                        global_work_queue_out[output_offset] = global_work_queue_in[input_queue * __GLOBAL_WORK_QUEUE_CAPACITY + input_offset];
                    }
                    n_splits_local -= splits_to_write;
                    input_offset += splits_to_write;
                    output_offset += splits_to_write;
                    input_queue_count -= splits_to_write;
                    n_splits_left -= splits_to_write;
                }
                barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
            }
        }
    }
}
// prevent file parsing warning on AMD APP SDK v2.9 by appending this line