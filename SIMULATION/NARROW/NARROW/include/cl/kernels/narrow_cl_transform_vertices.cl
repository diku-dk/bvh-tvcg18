#include "narrow_cl_error.clh"
#include "narrow_cl_quaternion_type.clh"
#include "narrow_cl_vector3_type.clh"

__kernel void do_transform_vertices(
    __global       vector3_type    *restrict vertices,
    __global const uint            *restrict vertex_offsets, // holds index of last vertex of that object + 1
    __global const vector3_type    *restrict positions,
    __global const quaternion_type *restrict rotations
#ifndef __NDEBUG_DIKUCL
    , __global volatile uint     *restrict error_codes
    , __global volatile uint     *restrict error_size          // must be initialized to 0
#endif // __NDEBUG_DIKUCL
    )
{
  // one group per object
  const size_t group_id   = get_group_id(0);
  const size_t local_id   = get_local_id(0);
  const size_t local_size = get_local_size(0);
  
  __local uint            vertices_start;
  __local uint            vertices_end;
  __local vector3_type    position;
  __local quaternion_type rotation;
  
  if (local_id == 0) {
    // first object's offset is 0
    vertices_start = group_id == 0 ? 0 : vertex_offsets[group_id - 1];
    vertices_end   = vertex_offsets[group_id]; // exclusive
    position       = positions[group_id];
    rotation       = rotations[group_id];
  }
  barrier(CLK_LOCAL_MEM_FENCE);
  
  for (  size_t offset = vertices_start + local_id
       ; offset < vertices_end
       ; offset += local_size
       )
  {
    RAISE(isnan(position.x) == 1 || isnan(position.y) == 1 || isnan(position.z) == 1,
      ((uint[]) { __LINE__ }), 1, error_codes, error_size);
    // high pressure situation, need *a lot* of concurrency here
    vertices[offset] = q_rotate(rotation, vertices[offset]) + position;
  }
}
// prevent file parsing warning on AMD APP SDK v2.9 by appending this line
