#include "kdop_cl_compute_contacts.clh"
#include "kdop_cl_contact_point.clh"
#include "kdop_cl_SAT_tetrahedrons.clh"
#include "kdop_cl_vector3_type.clh"
#include "kdop_cl_vector3_type_64.clh"
#include "kdop_cl_tetrahedron.clh"
#include "kdop_cl_tetrahedron_surface_info.clh"
#include "kdop_cl_work_item.clh"

__kernel void do_exact_tests(
    __global          Tetrahedron            *restrict tetrahedrons,
    __global          TetrahedronSurfaceInfo *restrict tetrahedron_surface_info,
    __global          vector3_type           *restrict vertices,
    __global          WorkItem               *restrict work,
             const    uint                             work_size,
    __global          ContactPoint           *restrict contact_points,
    __global volatile uint                   *restrict contact_points_size       // must be initialized to 0
    )
{
    const uint global_id = (uint) get_global_id(0);
    const uint global_size = (uint) get_global_size(0);
    
    for(uint work_pos = global_id; work_pos < work_size; work_pos += global_size) {
        WorkItem current = work[work_pos];
        
        Tetrahedron tetrahedron_a = tetrahedrons[current.a];
        Tetrahedron tetrahedron_b = tetrahedrons[current.b];
                
        vector3_type_64 A[4];
        A[0] = convert_vector3_type_64(vertices[tetrahedron_a.vertex_idx[0]]);
        A[1] = convert_vector3_type_64(vertices[tetrahedron_a.vertex_idx[1]]);
        A[2] = convert_vector3_type_64(vertices[tetrahedron_a.vertex_idx[2]]);
        A[3] = convert_vector3_type_64(vertices[tetrahedron_a.vertex_idx[3]]);
        
        vector3_type_64 B[4];
        B[0] = convert_vector3_type_64(vertices[tetrahedron_b.vertex_idx[0]]);
        B[1] = convert_vector3_type_64(vertices[tetrahedron_b.vertex_idx[1]]);
        B[2] = convert_vector3_type_64(vertices[tetrahedron_b.vertex_idx[2]]);
        B[3] = convert_vector3_type_64(vertices[tetrahedron_b.vertex_idx[3]]);
        
        TetrahedronSurfaceInfo tsi_a = tetrahedron_surface_info[current.a];
        TetrahedronSurfaceInfo tsi_b = tetrahedron_surface_info[current.b];
        
#ifdef __EARLY_REJECTION
        if( (!(tsi_a.vertex_on_surface[0]) && !(tsi_a.vertex_on_surface[1]) &&
             !(tsi_a.vertex_on_surface[2]) && !(tsi_a.vertex_on_surface[3])) ||
            (!(tsi_b.vertex_on_surface[0]) && !(tsi_b.vertex_on_surface[1]) &&
             !(tsi_b.vertex_on_surface[2]) && !(tsi_b.vertex_on_surface[3])) ||
            SAT_tetrahedrons(A, B))
        {
            // either all faces of one of the tetrahedrons are internal
            // or they are SAT separated
            continue;
        }
#endif // __EARLY_REJECTION
        
        compute_contacts(
                A, tsi_a,
                B, tsi_b,
                contact_points, contact_points_size,
                current.tp);
    }
}
// prevent file parsing warning on AMD APP SDK v2.9 by appending this line