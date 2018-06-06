#ifndef __K
#error "__K must be defined"
#endif // __K

#ifndef __MAX_BVTT_DEGREE
#warning "__MAX_BVTT_DEGREE should be specified, using 4"
#define __MAX_BVTT_DEGREE 4
#endif // __MAX_BVTT_DEGREE

#if __MAX_BVTT_DEGREE < 4
#error "__MAX_BVTT_DEGREE must be at least 4"
#endif // __MAX_BVTT_DEGREE < 4

#include "kdop_cl_error.clh"
#include "kdop_cl_index_type.clh"
#include "kdop_cl_node.clh"
#include "kdop_cl_work_item.clh"

__kernel void do_tandem_traversal(
      __global const    Node     *restrict nodes
    , __global const    WorkItem *restrict work
    ,          const    uint               work_size
    , __global          WorkItem *restrict exact_tests
    , __global volatile uint     *restrict exact_tests_size    // must be initialized to 0
    ,          const    uint               max_exact_tests
    , __global          WorkItem *restrict global_backlog
    , __global volatile uint     *restrict global_backlog_size // must be initialized to 0
    , __local           WorkItem *restrict backlog
    ,          const    uint               backlog_size
#ifndef __NDEBUG_DIKUCL
    ,          const    uint               nodes_size
    ,          const    uint               max_global_backlog_size
    , __global volatile uint     *restrict error_codes
    , __global volatile uint     *restrict error_size          // must be initialized to 0
#endif // __NDEBUG_DIKUCL
    )
{
    // work item ID
    const int local_id = (int) get_local_id(0); // int so we can compare against negative values
    const uint global_id = (uint) get_global_id(0);
            
    // sizes
    const int local_size = (int) get_local_size(0);
    const uint global_size = (uint) get_global_size(0);
        
    // cache until either the backlog is full or there is no more work
    int backlog_pos = local_id;
    uint work_pos = global_id;
    if(work_pos < work_size) {
        GUARD(backlog_pos, backlog_size, error_codes, error_size);
        GUARD(work_pos, work_size, error_codes, error_size);
        backlog[backlog_pos] = work[work_pos];
        work_pos += global_size;
        backlog_pos += local_size;
    }

    // loop step has been executed once too often
    //   and we want backlog_pos to point to the top of the stack
    backlog_pos -= local_size;
    
    // work_pos now points to the next un-cached element
                
    // run until empty or no more children can be pushed
    //  (keeping in mind we reuse the current position)
    bool out_of_space = false;
    while(backlog_pos >= local_id && !out_of_space) {
        GUARD(backlog_pos, backlog_size, error_codes, error_size);
        WorkItem current = backlog[backlog_pos];
        backlog_pos -= local_size;
        
        GUARD(current.a, nodes_size, error_codes, error_size);
        Node a = nodes[current.a];
        GUARD(current.b, nodes_size, error_codes, error_size);
        Node b = nodes[current.b];
        
        bool overlap = node_overlap(a, b);
        bool a_is_leaf = node_is_leaf(a);
        bool b_is_leaf = node_is_leaf(b);
        
        if(overlap && a_is_leaf && b_is_leaf) {
            // extract level from test pair
            uchar level = current.tp >> (sizeof(index_type) * 8 - 3);
            if(level == 0) {
                // push back the tetrahedron information if possible
                uint exact_tests_pos = atomic_inc(exact_tests_size);
                out_of_space = exact_tests_pos >= max_exact_tests;
                if(!out_of_space) {
                    GUARD(exact_tests_pos, max_exact_tests, error_codes, error_size);
                    exact_tests[exact_tests_pos].a = a.start;
                    exact_tests[exact_tests_pos].b = b.start;
                    exact_tests[exact_tests_pos].tp = current.tp;
                } else {
                    (void) atomic_dec(exact_tests_size);
                    backlog_pos += local_size;
                }
            } else {
                // extract proper test pair
                index_type tp = (current.tp << 3) >> 3;
                
                // decrement level
                tp |= (level - 1) << (sizeof(index_type) * 8 - 3);
                
                // declare as leftover work
                uint global_backlog_pos = atomic_inc(global_backlog_size);
                GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                global_backlog[global_backlog_pos].a = a.start;
                global_backlog[global_backlog_pos].b = b.start;
                global_backlog[global_backlog_pos].tp = tp;
            }
        } else if(overlap) {
            // push back children if possible
            out_of_space = (backlog_pos
                    + (a.end - a.start + 1) // number of children of a (or 1 if a is a leaf)
                    * (b.end - b.start + 1) // number of children of b (or 1 if b is a leaf)
                    * local_size) // step size
                    >= backlog_size;
            
            if(!out_of_space) {
                // unroll for binary BVHs
                // 2 or 4 of the following blocks must be executed since
                //   at most one of a or b is a leaf
                
                // increment backlog position once
                backlog_pos += local_size;
                GUARD(backlog_pos, backlog_size, error_codes, error_size);
                backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start;
                backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start + 1;
                backlog[backlog_pos].tp = current.tp;
                
                // don't overwrite if the previous block was correct
                backlog_pos += (b.start + 1 <= b.end) * local_size;
                GUARD(backlog_pos, backlog_size, error_codes, error_size);
                backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start + 1;
                backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start;
                backlog[backlog_pos].tp = current.tp;
                
                // don't overwrite if the previous block was correct
                backlog_pos += (a.start + 1 <= a.end) * local_size;
                GUARD(backlog_pos, backlog_size, error_codes, error_size);
                backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start + 1;
                backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start + 1;
                backlog[backlog_pos].tp = current.tp;
                
                // don't overwrite if the previous block was correct
                // this block will always be written and one or three of the above,
                //   depending on whether one node is a leaf or not
                backlog_pos += (a.start + 1 <= a.end && b.start + 1 <= b.end) * local_size;
                GUARD(backlog_pos, backlog_size, error_codes, error_size);
                backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start;
                backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start;
                backlog[backlog_pos].tp = current.tp;
#if __MAX_BVTT_DEGREE > 4
                // do the rest iteratively
                for(index_type child_a = a.start + 2; child_a <= a.end; ++child_a) {
                    for(index_type child_b = b.start + 2; child_b <= b.end; ++child_b) {
                        backlog_pos += local_size;
                        GUARD(backlog_pos, backlog_size, error_codes, error_size);
                        backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * child_a;
                        backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * child_b;
                        backlog[backlog_pos].tp = current.tp;
                    }
                }
#endif // __MAX_BVTT_DEGREE > 4
            } else {
                uint global_backlog_pos = atomic_add(
                        global_backlog_size,
                        out_of_space * (a.end - a.start + 1) * (b.end - b.start + 1));
                // unroll for binary BVHs
                //   following the same overwrite rationale as above
                GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                global_backlog[global_backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start;
                global_backlog[global_backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start + 1;
                global_backlog[global_backlog_pos].tp = current.tp;
                global_backlog_pos += (b.start + 1 <= b.end);
                GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                global_backlog[global_backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start + 1;
                global_backlog[global_backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start;
                global_backlog[global_backlog_pos].tp = current.tp;
                global_backlog_pos += (a.start + 1 <= a.end);
                GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                global_backlog[global_backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start + 1;
                global_backlog[global_backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start + 1;
                global_backlog[global_backlog_pos].tp = current.tp;
                global_backlog_pos += (a.start + 1 <= a.end && b.start + 1 <= b.end);
                GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                global_backlog[global_backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start;
                global_backlog[global_backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start;
                global_backlog[global_backlog_pos].tp = current.tp;
                ++global_backlog_pos;
#if __MAX_BVTT_DEGREE > 4
                // do the rest iteratively
                for(index_type child_a = a.start + 2; child_a <= a.end; ++child_a) {
                    for(index_type child_b = b.start + 2; child_b <= b.end; ++child_b) {
                        GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                        global_backlog[global_backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * child_a;
                        global_backlog[global_backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * child_b;
                        global_backlog[global_backlog_pos].tp = current.tp;
                        ++global_backlog_pos;
                    }
                }
#endif // __MAX_BVTT_DEGREE > 4
            }
        }
        
        if(backlog_pos < local_id && work_pos < work_size) {
            backlog_pos += local_size;
            GUARD(backlog_pos, backlog_size, error_codes, error_size);
            GUARD(work_pos, work_size, error_codes, error_size);
            backlog[backlog_pos] = work[work_pos];
            work_pos += global_size;
        }
    }
    
    // calculate remaining work size
    uint global_backlog_pos = atomic_add(
            global_backlog_size,
            (backlog_pos >= local_id) * ((backlog_pos - local_id) / local_size + 1) + // no. of items left in the backlog
            (work_pos < work_size) * ((work_size - work_pos) / global_size + (((work_size - work_pos) % global_size) ? 1 : 0)) // no. of items that could not be cached
            );
    // copy back remaining work in the backlog
    for(; backlog_pos >= local_id; backlog_pos -= local_size, ++global_backlog_pos) {
        GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
        GUARD(backlog_pos, backlog_size, error_codes, error_size);
        global_backlog[global_backlog_pos] = backlog[backlog_pos];
    }

    // copy back work that could not be cached
    for(; work_pos < work_size; work_pos += global_size, ++global_backlog_pos) {
        GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
        GUARD(work_pos, work_size, error_codes, error_size);
        global_backlog[global_backlog_pos] = work[work_pos];
    }
}

// CPU kernel that uses global memory as backlog
//   since global memory is cached in L1 anyway
//   and local memory caching just causes overhead since it sits in global memory
// for comments see the above kernel
__kernel void do_tandem_traversal_cpu(
      __global const    Node     *restrict nodes
    , __global const    WorkItem *restrict work
    ,          const    uint               work_size
    , __global          WorkItem *restrict exact_tests
    , __global volatile uint     *restrict exact_tests_size
    ,          const    uint               max_exact_tests
    , __global          WorkItem *restrict global_backlog
    , __global volatile uint     *restrict global_backlog_size    
    , __global          WorkItem *restrict backlog       // guaranteed to have enough
                                                         // space if we access it properly
    ,          const    uint               backlog_size
#ifndef __NDEBUG_DIKUCL
    ,          const    uint               nodes_size
    ,          const    uint               max_global_backlog_size
    , __global volatile uint     *restrict error_codes
    , __global volatile uint     *restrict error_size          // must be initialized to 0
#endif // __NDEBUG_DIKUCL
    )
{    
    const int global_id = (int) get_global_id(0);
    
    // In order to not be wasteful with global memory here (plus not exceed it),
    // set the step size to the number of work items that can actually work.
    // Imagine get_global_size(0) = 1000 but work_size = 1,
    // then backlog will be exceeded if we step through it in strides
    // of 1000 since it only has room for at most all BVTT leaves which are
    // less than 1000 * 4 times the height h of the single BVTT tree we have.
    const int stride = min(work_size, (uint) get_global_size(0));
    
    // Since each work item uses at most 4 * h entries in the backlog at a time
    // and each BVTT tree's h is sufficiently smaller than its last level's size,
    // (which is the amount of memory we have allocated in the backlog)
    // for each work item there is enough room in the backlog to store all its
    // intermediate entries, given careful alignment. If only one work item can
    // do work, that means there is just one BVTT tree and an accordingly small
    // backlog which we can only not exceed if that single work item does not
    // leave space in between its accesses. This is valid for all numbers of
    // work items and BVTT trees, up until there is more work than work items
    // in which case we have enough memory allocated in the backlog anyway.
    // Obviously the above discussion is relevant only when there are mor work
    // items than actual work.
    // See also Robert's thesis for additional information.
        
    uint work_pos = global_id;
    int backlog_pos = global_id;
    if(work_pos < work_size) {
        GUARD(backlog_pos, backlog_size, error_codes, error_size);
        GUARD(work_pos, work_size, error_codes, error_size);
        backlog[backlog_pos] = work[work_pos];
        work_pos += stride;
        backlog_pos += stride;
    }
    backlog_pos -= stride;
    
    bool out_of_space = false;
    while(backlog_pos >= global_id && !out_of_space) {
        GUARD(backlog_pos, backlog_size, error_codes, error_size);
        WorkItem current = backlog[backlog_pos];
        backlog_pos -= stride;

        GUARD(current.a, nodes_size, error_codes, error_size);
        Node a = nodes[current.a];
        GUARD(current.b, nodes_size, error_codes, error_size);
        Node b = nodes[current.b];
                
        bool overlap = node_overlap(a, b);
        bool a_is_leaf = node_is_leaf(a);
        bool b_is_leaf = node_is_leaf(b);

        if(overlap && a_is_leaf && b_is_leaf) {
            uchar level = current.tp >> (sizeof(index_type) * 8 - 3);
            if(level == 0) {
                uint exact_tests_pos = atomic_inc(exact_tests_size);
                out_of_space = exact_tests_pos >= max_exact_tests;
                if(!out_of_space) {
                    GUARD(exact_tests_pos, max_exact_tests, error_codes, error_size);
                    exact_tests[exact_tests_pos].a = a.start;
                    exact_tests[exact_tests_pos].b = b.start;
                    exact_tests[exact_tests_pos].tp = current.tp;
                } else {
                    (void) atomic_dec(exact_tests_size);
                    backlog_pos += stride;
                    GUARD(backlog_pos, backlog_size, error_codes, error_size);
                    backlog[backlog_pos] = current;
                }
            } else {
                index_type tp = (current.tp << 3) >> 3;
                tp |= (level - 1) << (sizeof(index_type) * 8 - 3);
                uint global_backlog_pos = atomic_inc(global_backlog_size);
                GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
                global_backlog[global_backlog_pos].a = a.start;
                global_backlog[global_backlog_pos].b = b.start;
                global_backlog[global_backlog_pos].tp = tp;
            }
        } else if(overlap) {
            backlog_pos += stride;
            GUARD(backlog_pos, backlog_size, error_codes, error_size);
            backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start;
            backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start + 1;
            backlog[backlog_pos].tp = current.tp;
            backlog_pos += (b.start + 1 <= b.end) * stride;
            GUARD(backlog_pos, backlog_size, error_codes, error_size);
            backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start + 1;
            backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start;
            backlog[backlog_pos].tp = current.tp;
            backlog_pos += (a.start + 1 <= a.end) * stride;
            GUARD(backlog_pos, backlog_size, error_codes, error_size);
            backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start + 1;
            backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start + 1;
            backlog[backlog_pos].tp = current.tp;
            backlog_pos += (a.start + 1 <= a.end && b.start + 1 <= b.end) * stride;
            GUARD(backlog_pos, backlog_size, error_codes, error_size);
            backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * a.start;
            backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * b.start;
            backlog[backlog_pos].tp = current.tp;
#if __MAX_BVTT_DEGREE > 4
            for(index_type child_a = a.start + 2; child_a <= a.end; ++child_a) {
                for(index_type child_b = b.start + 2; child_b <= b.end; ++child_b) {
                    backlog_pos += stride;
                    GUARD(backlog_pos, backlog_size, error_codes, error_size);
                    backlog[backlog_pos].a = a_is_leaf * current.a + (!a_is_leaf) * child_a;
                    backlog[backlog_pos].b = b_is_leaf * current.b + (!b_is_leaf) * child_b;
                    backlog[backlog_pos].tp = current.tp;
                }
            }
#endif // __MAX_BVTT_DEGREE > 4
        }
                
        if(backlog_pos < global_id && work_pos < work_size) {
            backlog_pos += stride;
            GUARD(backlog_pos, backlog_size, error_codes, error_size);
            GUARD(work_pos, work_size, error_codes, error_size);
            backlog[backlog_pos] = work[work_pos];
            work_pos += stride;
        }
    }
        
    uint global_backlog_pos = atomic_add(
            global_backlog_size,
            (backlog_pos >= global_id) * ((backlog_pos - global_id) / stride + 1) +
            (work_pos < work_size) * ((work_size - work_pos) / stride + 1)
            );
    for(; backlog_pos >= global_id; backlog_pos -= stride, ++global_backlog_pos) {
        GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
        GUARD(backlog_pos, backlog_size, error_codes, error_size);
        global_backlog[global_backlog_pos] = backlog[backlog_pos];
    }
    for(; work_pos < work_size; work_pos += stride, ++global_backlog_pos) {
        GUARD(global_backlog_pos, max_global_backlog_size, error_codes, error_size);
        GUARD(work_pos, work_size, error_codes, error_size);
        global_backlog[global_backlog_pos] = work[work_pos];
    }
}
// prevent file parsing warning on AMD APP SDK v2.9 by appending this line
