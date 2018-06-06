#ifndef __IDLE_THRESHOLD
#error "__IDLE_THRESHOLD must be defined"
#endif // __IDLE_THRESHOLD

#include "kdop_cl_error.clh"
#include "kdop_cl_index_type.clh"
#include "kdop_cl_node.clh"
#include "kdop_cl_work_item.clh"

__kernel void do_tandem_traversal(
    __global const    Node     *restrict nodes,
    __global          WorkItem *restrict global_work_queues,
    __global          uint     *restrict global_work_queues_counts,
    __global volatile uint     *restrict idle_count,
    __global          WorkItem *restrict exact_tests,
    __global volatile uint     *restrict exact_tests_count,
    __local           WorkItem *restrict local_work_queue,
             const    int                local_work_queue_capacity,
             const    uint               global_work_queue_capacity,
             const    uint               global_exact_tests_capacity
#ifndef __NDEBUG_DIKUCL
    ,        const    uint               nodes_size
    ,        const    uint               global_work_queues_size
    , __global volatile uint     *restrict error_codes
    , __global volatile uint     *restrict error_size          // must be initialized to 0
#endif // __NDEBUG_DIKUCL
    )
{

    __local int local_work_queue_size;
    __local int global_work_queue_size;
    __local uint want_to_abort;

    const int group_id = get_group_id(0);
    const int group_size = get_local_size(0);
    const int local_id = get_local_id(0);

    // the actual number of idle threads, as the constant is only a fraction
    const int idle_threshold = __IDLE_THRESHOLD * group_size;

//    if(local_id == 0) {
//        global_work_queue_size = global_work_queues_counts[group_id];
        local_work_queue_size = min(min(group_size * 3,
                                        (int) global_work_queues_counts[group_id]),
                                    local_work_queue_capacity);
        global_work_queue_size = global_work_queues_counts[group_id] - local_work_queue_size;
//    }
    
    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

    if(local_work_queue_size == 0) {
        if(local_id == 0) {
            (void) atomic_inc(idle_count);
        }
        return;
    }

    {
        GUARD(group_id * global_work_queue_capacity + global_work_queue_size, global_work_queues_size, error_codes, error_size);
        __global WorkItem *global_queue = &global_work_queues[group_id * global_work_queue_capacity + global_work_queue_size];
        int queue_offset = local_id;
        while(queue_offset < local_work_queue_size/* * 3*/) {
            GUARD(queue_offset, local_work_queue_capacity, error_codes, error_size);
            GUARD(queue_offset, global_work_queue_capacity, error_codes, error_size);
            local_work_queue[queue_offset] = global_queue[queue_offset];
            queue_offset += group_size;
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
    
    while(local_work_queue_size > 0) {
        WorkItem work_item;
        int n_active = min(group_size, local_work_queue_size);

        bool can_work = local_id < local_work_queue_size;
        if(can_work) {
            GUARD(local_work_queue_size - n_active + local_id, local_work_queue_capacity, error_codes, error_size);
            work_item = local_work_queue[local_work_queue_size - n_active + local_id];
        }
        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

        if(local_id == 0) {
            local_work_queue_size -= n_active;
        }
        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
        
        if(can_work) {
            GUARD(work_item.a, nodes_size, error_codes, error_size);
            Node node_a = nodes[work_item.a];
            GUARD(work_item.b, nodes_size, error_codes, error_size);
            Node node_b = nodes[work_item.b];

            bool overlap = node_overlap(node_a, node_b);
            bool a_is_leaf = node_is_leaf(node_a);
            bool b_is_leaf = node_is_leaf(node_b);

            if(overlap) {
                if(a_is_leaf && b_is_leaf) {
                    uint exact_tests_pos = atomic_inc(exact_tests_count);
                    if(exact_tests_pos >= global_exact_tests_capacity) {
                        // TODO handle
                    }
                    GUARD(exact_tests_pos, global_exact_tests_capacity, error_codes, error_size);
                    exact_tests[exact_tests_pos].a = node_a.start;
                    exact_tests[exact_tests_pos].b = node_b.start;
                    exact_tests[exact_tests_pos].tp = work_item.tp;
                } else if(b_is_leaf || (!a_is_leaf && node_size(node_a) > node_size(node_b))) {
                    int l = atomic_add(&local_work_queue_size, 2);

                    if(l < local_work_queue_capacity) {
                        GUARD(l, local_work_queue_capacity, error_codes, error_size);
                        local_work_queue[l].a = node_a.start;
                        local_work_queue[l].b = work_item.b;
                        local_work_queue[l].tp = work_item.tp;
                        ++l;

                        if(l < local_work_queue_capacity) {
                            GUARD(l, local_work_queue_capacity, error_codes, error_size);
                            local_work_queue[l].a = node_a.end;
                            local_work_queue[l].b = work_item.b;
                            local_work_queue[l].tp = work_item.tp;
                        } else {
                            int g = atomic_inc(&global_work_queue_size);
                            GUARD(group_id * global_work_queue_capacity + g, global_work_queues_size, error_codes, error_size);
                            __global WorkItem *global_queue = &global_work_queues[group_id * global_work_queue_capacity + g];

                            global_queue[0].a = node_a.end;
                            global_queue[0].b = work_item.b;
                            global_queue[0].tp = work_item.tp;
                        }
                    } else {
                        int g = atomic_add(&global_work_queue_size, 2);
                        GUARD(group_id * global_work_queue_capacity + g, global_work_queues_size, error_codes, error_size);
                        __global WorkItem *global_queue = &global_work_queues[group_id * global_work_queue_capacity + g];

                        global_queue[0].a = node_a.start;
                        global_queue[0].b = work_item.b;
                        global_queue[0].tp = work_item.tp;

                        GUARD(group_id * global_work_queue_capacity + g + 1, global_work_queues_size, error_codes, error_size);
                        global_queue[1].a = node_a.end;
                        global_queue[1].b = work_item.b;
                        global_queue[1].tp = work_item.tp;
                    }
                } else {
                    int l = atomic_add(&local_work_queue_size, 2);

                    if(l < local_work_queue_capacity) {
                        GUARD(l, local_work_queue_capacity, error_codes, error_size);
                        local_work_queue[l].a = work_item.a;
                        local_work_queue[l].b = node_b.start;
                        local_work_queue[l].tp = work_item.tp;
                        ++l;

                        if(l < local_work_queue_capacity) {
                            GUARD(l, local_work_queue_capacity, error_codes, error_size);
                            local_work_queue[l].a = work_item.a;
                            local_work_queue[l].b = node_b.end;
                            local_work_queue[l].tp = work_item.tp;
                        } else {
                            int g = atomic_inc(&global_work_queue_size);
                            GUARD(group_id * global_work_queue_capacity + g, global_work_queues_size, error_codes, error_size);
                            __global WorkItem *global_queue = &global_work_queues[group_id * global_work_queue_capacity + g];

                            global_queue[0].a = work_item.a;
                            global_queue[0].b = node_b.end;
                            global_queue[0].tp = work_item.tp;
                        }
                    } else {
                        int g = atomic_add(&global_work_queue_size, 2);
                        GUARD(group_id * global_work_queue_capacity + g, global_work_queues_size, error_codes, error_size);
                        __global WorkItem *global_queue = &global_work_queues[group_id * global_work_queue_capacity + g];

                        global_queue[0].a = work_item.a;
                        global_queue[0].b = node_b.start;
                        global_queue[0].tp = work_item.tp;

                        GUARD(group_id * global_work_queue_capacity + g + 1, global_work_queues_size, error_codes, error_size);
                        global_queue[1].a = work_item.a;
                        global_queue[1].b = node_b.end;
                        global_queue[1].tp = work_item.tp;
                    }
                }
            }
        }
        
        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

        if(local_work_queue_size >= local_work_queue_capacity - group_size ||
           global_work_queue_size >= global_work_queue_capacity - group_size * 2 - local_work_queue_capacity ||
           local_work_queue_size == 0)
        {
            if(local_id == 0) {
                (void) atomic_inc(idle_count);
            }
            break;
        }

        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

        if(local_id == 0) {
            want_to_abort = *idle_count;
        }

        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

        if(want_to_abort >= idle_threshold) {
            if(local_id == 0) {
                (void) atomic_inc(idle_count);
            }
            break;
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
    
//    if(local_id == 0) {
        local_work_queue_size = min(local_work_queue_size, local_work_queue_capacity);
        global_work_queues_counts[group_id] = local_work_queue_size + global_work_queue_size;
//    }

    barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);

    {
        int queue_offset = local_id;
        GUARD(group_id * global_work_queue_capacity + global_work_queue_size, global_work_queues_size, error_codes, error_size);
        __global WorkItem *global_queue = &global_work_queues[group_id * global_work_queue_capacity + global_work_queue_size];
        while(queue_offset < local_work_queue_size/* * 2*/) {
            GUARD(queue_offset, global_work_queue_capacity, error_codes, error_size);
            GUARD(queue_offset, local_work_queue_capacity, error_codes, error_size);
            global_queue[queue_offset] = local_work_queue[queue_offset];
            queue_offset += group_size;
        }
    }
}
// prevent file parsing warning on AMD APP SDK v2.9 by appending this line
