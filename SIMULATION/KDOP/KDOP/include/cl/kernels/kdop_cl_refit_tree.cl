#ifndef __K
#error "__K must be defined"
#endif // __K

#if __K != 8
// See below when the direction table is used.
#error "This kernel currently only supports __K = 8."
#endif // __ K != 8

#include "kdop_cl_direction_table.clh"
#include "kdop_cl_error.clh"
#include "kdop_cl_node.clh"
#include "kdop_cl_real_type.clh"
#include "kdop_cl_tetrahedron.clh"
#include "kdop_cl_vector3_type.clh"

__kernel void do_refit_tree(
    __global const vector3_type *restrict vertices,
    __global const uint         *restrict vertex_offsets,
    __global const Tetrahedron  *restrict tetrahedrons,
    __global const uint         *restrict tetrahedron_offsets,
    __global       Node         *restrict nodes,
    __global const uint         *restrict node_offsets,
    __global const uint         *restrict last_level_offsets
#ifndef __NDEBUG_DIKUCL
    , __global volatile uint     *restrict error_codes
    , __global volatile uint     *restrict error_size          // must be initialized to 0
#endif // __NDEBUG_DIKUCL
    )
{
    // One group per object.
    const size_t group_id   = get_group_id(0);
    const size_t local_id   = get_local_id(0);
    const size_t local_size = get_local_size(0);
    
    // Hold the offsets of BVH nodes for this object in vertices.
    __local uint nodes_start;
    __local uint nodes_end;
    
    // Repeat for tetrahedrons and vertices.
    __local uint tetrahedrons_start;
    __local uint vertices_start;
    
    // Holds the offset of the first chunk in the last level of this object.
    // Needed to identify the type of leaf: either it references a chunk in a
    // lower level, or it references a tetrahedron.
    __local uint last_level_offset;
    
    // Number of threads that sit idle during updating.
    __local uint idle_count;
    
    if(local_id == 0) {
        nodes_start        = group_id == 0 ? 0 : node_offsets[group_id - 1];
        nodes_end          = node_offsets[group_id];
        tetrahedrons_start = group_id == 0 ? 0 : tetrahedron_offsets[group_id - 1];
        vertices_start     = group_id == 0 ? 0 : vertex_offsets[group_id - 1];
        last_level_offset  = last_level_offsets[group_id];
        idle_count         = 0;
    }
    barrier(CLK_LOCAL_MEM_FENCE);
    
    /* Walk through all nodes from the back. */
    
    // In case a child KDOP was not computed, a thread needs to try again.
    bool try_again = false;
    
    // Indicates whether this thread cannot do any more work.
    bool idle = false;
    
    // Helper variable to identify the first iteration.
    bool first_run = true;
    
    size_t offset = nodes_end - local_id - 1;
    while(true)
    {
        // Make all threads have a consistent view on global and local memory.
        // This barrier needs to be encountered by all threads.
        barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
        
        // If all threads are idle, make them all exit at the same time.
        if(idle_count == local_size) {
            break;
        }
        
        // Compute new offset if possible.
        bool abort = (!first_run) && // do not abort on the first run
                (local_size > offset) && // abort if offset would overflow
                (!try_again); // do not abort when retrying the same offset
        offset -= local_size *
                (!first_run) * // do not advance on first run
                (local_size <= offset) * // advance if offset does not overflow
                (!try_again); // do not advance when retrying same offset
        first_run = false;
        
        if(idle || abort || offset < nodes_start || offset >= nodes_end) {
            // Thread cannot do more work, so increment the number of idle
            // threads exactly once and don't continue working.
            if(!idle) {
                (void) atomic_inc(&idle_count);
            };
            idle = true;
            continue;
        }
        
        // Reset retrying state.
        try_again = false;
        
        GUARD(offset, nodes_end, error_codes, error_size);
        Node node = nodes[offset];
        if(node_is_leaf(node)) {
            if(offset >= nodes_start + last_level_offset) {
                // Regular leaf because the offset is larger than/equal to the
                // offset of the first chunk in the last level.
                Tetrahedron tetrahedron = tetrahedrons[tetrahedrons_start + node.start];
                
                // Update each slab with projection for each of the 4 points
                // of a tetrahedron.
                for(uchar n = 0; n < 4; ++n) {
                    vector3_type p = vertices[vertices_start + tetrahedron.vertex_idx[n]];
                    for(uchar k = 0; k < __K / 2; ++k) {
                        // TODO FIXME this does not work with other __K than 8
                        const real_type projection = dot(DIRECTION_TABLE_4[k], p);
                        node.slabs[k].lower = fmin(node.slabs[k].lower, projection);
                        node.slabs[k].upper = fmax(node.slabs[k].upper, projection);
                    }
                }
                
                // Copy back the volume.
                for(uchar k = 0; k < __K / 2; ++k) {
                    nodes[offset].slabs[k].lower = node.slabs[k].lower;
                    nodes[offset].slabs[k].upper = node.slabs[k].upper;
                }
            } else {
                // Super leaf, reuse already computed child volume, if valid.
                // Otherwise, try again in the next iteration.
                GUARD(nodes_start + node.start, nodes_end, error_codes, error_size);
                Node n = nodes[nodes_start + node.start];
                if(!node_valid_kdop(n)) {
                    try_again = true;
                } else {
                    for(uchar k = 0; k < __K / 2; ++k) {
                        nodes[offset].slabs[k].lower = n.slabs[k].lower;
                        nodes[offset].slabs[k].upper = n.slabs[k].upper;
                    }
                }
            }
        } else {
            // Internal node, merge volumes of the two children, if valid.
            // Otherwise, try again in the next iteration.
            GUARD(nodes_start + node.start, nodes_end, error_codes, error_size);
            Node left  = nodes[nodes_start + node.start];
            if(!node_valid_kdop(left)) {
                try_again = true;
            } else {
                GUARD(nodes_start + node.end, nodes_end, error_codes, error_size);
                Node right = nodes[nodes_start + node.end];
                if(!node_valid_kdop(right)) {
                    try_again = true;
                } else {
                    for(uchar k = 0; k < __K / 2; ++k) {
                        nodes[offset].slabs[k].lower = min(left.slabs[k].lower, right.slabs[k].lower);
                        nodes[offset].slabs[k].upper = max(left.slabs[k].upper, right.slabs[k].upper);
                    }
                }
            }
        }
    }
}
// prevent file parsing warning on AMD APP SDK v2.9 by appending this line
