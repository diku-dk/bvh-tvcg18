#ifndef KDOP_CL_TREE_H
#define	KDOP_CL_TREE_H

#include <dikucl.hpp>

namespace kdop
{
    namespace details
    {
        namespace cl
        {
            
            template< typename KernelT >
            struct KernelInterval {
                KernelT lower;
                KernelT upper;
            };

            template< typename KernelI, typename KernelT, size_t K >
            struct KernelNode {
                KernelInterval<KernelT> slabs[K / 2];
                KernelI start;
                KernelI end;
            };

            template< typename KernelI >
            struct KernelWorkItem {
                KernelI a;
                KernelI b;
                KernelI tp;
            };

            template< typename KernelI >
            struct KernelTetrahedron {
                KernelI vertex_idx[4];
            };

            struct KernelTetrahedronSurfaceInfo {
                cl_uchar vertex_on_surface[4];
            };
            
        } // namespace cl
        
    } // namespace details
    
} // namespace kdop

#endif // KDOP_CL_TREE_H

