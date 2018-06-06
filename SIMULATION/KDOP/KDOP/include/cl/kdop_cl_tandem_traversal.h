#ifndef KDOP_CL_TANDEM_TRAVERSAL_H
#define KDOP_CL_TANDEM_TRAVERSAL_H

#include <cl/kdop_cl_kernels_path.h>
#include <cl/kdop_cl_tree.h>

// OpenCL specifics
#include <dikucl.hpp>
#include <dikucl_command_queue_manager.hpp>
#include <dikucl_context_manager.hpp>
#include <dikucl_kernel_manager.hpp>
#include <dikucl_platform_manager.hpp>
#include <dikucl_util.h>

#include <contacts/geometry_contacts_callback.h>
#include <types/geometry_dop.h>

#include <kdop_tags.h>
#include <kdop_test_pair.h>
#include <kdop_tree.h>

#include <mesh_array.h>
#include <mesh_array_t4mesh.h>
#include <mesh_array_vertex_attribute.h>

#include <util_profiling.h>
#include <util_log.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <stdlib.h>
#include <string>
#include <utility>

#ifndef NDEBUG_DIKUCL
#define MAX_ERROR_CODES 1024
#else // NDEBUG_DIKUCL
#define MAX_ERROR_CODES 0
#endif // NDEBUG_DIKUCL

// If a test pair's chunks n^2 test results in more than this amount of tests,
// then issue superchunk pair tests for this device type instead.
#define KDOP_CL_MAX_ROOT_PAIRS_CPU 10000
#define KDOP_CL_MAX_ROOT_PAIRS_GPU 10000000

namespace kdop {
    namespace details {
        
        namespace cl {

            template< typename KernelV, typename KernelT, typename KernelI >
            struct KernelContactPoint {
                KernelV p;
                KernelV n;
                KernelT d;
                KernelI tp;
            };
            
            template< typename KernelV >
            int compare_vectors(KernelV *a, KernelV *b) {
                if(a->s[0] < b->s[0]) {
                    return -1;
                } else if(a->s[0] > b->s[0]) {
                    return 1;
                }
                if(a->s[1] < b->s[1]) {
                    return -1;
                } else if(a->s[1] > b->s[1]) {
                    return 1;
                }
                if(a->s[2] < b->s[2]) {
                    return -1;
                } else if(a->s[2] > b->s[2]) {
                    return 1;
                }
                return 0;
            }
            
            template< typename KernelV, typename KernelT, typename KernelI >
            int compare_kernel_contact_points(const void *_a, const void *_b) {
                KernelContactPoint<KernelV, KernelT, KernelI> a = *((KernelContactPoint<KernelV, KernelT, KernelI> *) _a);
                KernelContactPoint<KernelV, KernelT, KernelI> b = *((KernelContactPoint<KernelV, KernelT, KernelI> *) _b);
                if(a.tp < b.tp) {
                    return -1;
                } else if(a.tp > b.tp) {
                    return 1;
                }
                int c = compare_vectors(&(a.p), &(b.p));
                if(c != 0) {
                    return c;
                }
                c = compare_vectors(&(a.n), &(b.n));
                if(c != 0) {
                    return c;
                }
                if(a.d < b.d) {
                    return -1;
                } else if(a.d > b.d) {
                    return 1;
                }
                return 0;
            }

            // describes offsets of objects parts into memory
            //   this allows us to re-use duplicate objects
            struct object_offset {
                size_t node;
                size_t tet;
                size_t vert;
            };
            
            template< typename KernelI >
            class KernelWorkItemGenerator {
                
            protected:
                
                struct RootPairs {
                    // root indices for the first object in the test pair
                    std::vector<KernelI> a;
                    // root indices for the second object in the test pair
                    std::vector<KernelI> b;
                    // index of the test pair
                    KernelI tp;
                };
                
                // all test pairs
                std::queue< RootPairs > m_q;
                // the current position in m_q
                size_t m_a, m_b;
                // remember the max level currently in the queue
                KernelI max_level;
                
            public:
                
                KernelWorkItemGenerator() : m_a(0), m_b(0), max_level(0) {
                }
                
                // adds the root indices for two chunks
                virtual void add_roots(std::vector<KernelI> a, std::vector<KernelI> b, KernelI tp, KernelI level) {
                    m_q.push(RootPairs());
                    m_q.back().a = std::vector<KernelI>(a);
                    m_q.back().b = std::vector<KernelI>(b);
                    
                    assert(level < (1 << 3) || !"add_roots: level is too high");
                    max_level = std::max(max_level, level);
                    
                    // put the level information inside the top 3 bits
                    const size_t bits = sizeof(KernelI) * 8;
                    m_q.back().tp = level << (bits - 3);
                    
                    assert(tp < (1 << (bits - 3)) || !"add_roots: tp is too high");
                    m_q.back().tp |= tp;
                }
                
                // generates n out of all kernel work items that it has been provided with before
                // generates new kernel work items when invoked again
                // returns 0 if there are no more kernel work items to be processed
                virtual size_t generate_kernel_work_items(
                    size_t n, size_t weight_super_chunk_tp,
                    KernelWorkItem<KernelI> **out_kernel_work_items,
                    cl_device_type device_type,
                    size_t global_mem_cacheline_size,
                    size_t max_bvtt_degree,
                    size_t max_bvtt_height,
                    size_t max_global_work_size,
                    size_t * out_num_super_chunks)
                {
                    if(m_q.empty()) {
                        *out_kernel_work_items = NULL;
                        *out_num_super_chunks = 0;
                        return 0;
                    }
                    
                    // TODO decrease max_level appropriately, e.g. by not having
                    // just one max_level, but a map counting the test pairs per level
                    
                    // reduce maximum number of work items by the amount we might
                    // exceed it by through our way of counting further down
                    size_t possible_exceed = pow(weight_super_chunk_tp, max_level);
                    assert(n > possible_exceed || !"generate_kernel_work_items: n too small");
                    n -= possible_exceed;
                    
                    if(device_type & CL_DEVICE_TYPE_CPU) {
                        size_t size = std::max(
                                n,
                                max_bvtt_degree * max_bvtt_height * max_global_work_size)
                                * sizeof(KernelWorkItem<KernelI>);
                        size += size % global_mem_cacheline_size;
                        *out_kernel_work_items = (KernelWorkItem<KernelI>*) ALIGNED_ALLOC(4096, size);
                    } else {
                        *out_kernel_work_items = new KernelWorkItem<KernelI>[
                            std::max(
                                n,
                                max_bvtt_degree * max_bvtt_height * max_global_work_size)
                        ];
                    }
                    
                    size_t actual = 0;
                    size_t num_super_chunks = 0;
                    // collect work items while we don't have enough
                    // and there are still some to be generated
                    // subtract the number of super chunks from actual since we count them twice
                    while((actual + num_super_chunks * weight_super_chunk_tp) < n
                            && !m_q.empty())
                    {
                        // continue where we might have left off in an earlier
                        // invocation
                        for(; m_a < m_q.front().a.size()
                                && (actual + num_super_chunks * weight_super_chunk_tp) < n;
                                ++m_a)
                        {
                            // continue where we might have left off in an earlier
                            // invocation
                            for(; m_b < m_q.front().b.size()
                                    && (actual + num_super_chunks * weight_super_chunk_tp) < n;
                                    ++m_b)
                            {
                                (*out_kernel_work_items)[actual].a = (KernelI) m_q.front().a[m_a];
                                (*out_kernel_work_items)[actual].b = (KernelI) m_q.front().b[m_b];
                                (*out_kernel_work_items)[actual].tp = (KernelI) m_q.front().tp;
                                
                                // do we have a super chunk?
                                size_t level = (*out_kernel_work_items)[actual].tp >> (sizeof(KernelI) * 8 - 3);
                                if(level > 0) {
                                    // add the number of level 1 super chunks
                                    // by expanding the higher levels
                                    num_super_chunks += pow(weight_super_chunk_tp, (level - 1));
                                }
                                
                                ++actual;
                            }
                            if((actual + num_super_chunks * weight_super_chunk_tp) < n) {
                                // we finished the loop because m_b became too large
                                // so allow for another execution of the inner loop
                                m_b = 0;
                            } else {
                                // we collected enough work items, prevent the outer loop
                                // from incrementing m_a
                                break;
                            }
                        }
                        if((actual + num_super_chunks * weight_super_chunk_tp) < n) {
                            // we broke the loop because m_a became too large
                            // so allow for another execution of the outer loop
                            // with the next test pair
                            m_a = 0;
                            m_q.pop();
                        }
                    }
                    
                    *out_num_super_chunks = num_super_chunks;
                    return actual;
                }
                
              // frees the memory that was allocated during the generation of work items
              virtual void cleanup_generated_work_items(
                                                        KernelWorkItem<KernelI> *generated_kernel_work_items,
                                                        cl_device_type device_type
                                                        )
              {
                if(generated_kernel_work_items != NULL)
                {
                  if(device_type & CL_DEVICE_TYPE_CPU)
                  {
                    free(generated_kernel_work_items);
                  }
                  else
                  {
                    delete[] generated_kernel_work_items;
                  }
                }
              }

              bool empty() const
              {
                return m_q.empty();
              }

            };

            template< typename V, size_t K, typename T,
            typename KernelI, typename KernelT, typename KernelV,
            typename test_pair_container >
            inline void prepare_input(
                    test_pair_container test_pairs,
                    KernelNode<KernelI, KernelT, K> **out_nodes,
                    size_t *out_nodes_size,
                    KernelTetrahedron<KernelI> **out_tets,
                    size_t *out_tets_size,
                    KernelTetrahedronSurfaceInfo **out_tsi,
                    KernelV **out_verts,
                    size_t *out_verts_size,
                    KernelWorkItemGenerator<KernelI> *out_kernel_work_item_generator,
                    geometry::ContactsCallback<V> ***out_callbacks,
                    size_t *out_max_bvtt_degree,
                    size_t *out_max_bvtt_height,
                    cl_device_type device_type,
                    size_t global_mem_cacheline_size,
                    size_t max_kernel_work_items)
            {
                // maintain object offsets in memory to void duplicates
                std::map< Tree<T, K> const*, object_offset > object_offsets;

                size_t nodes_size = 0, tets_size = 0, verts_size = 0;
                size_t max_bvtt_degree = 0, max_bvh_height = 0;

                for (typename test_pair_container::iterator it = test_pairs.begin(); it != test_pairs.end(); std::advance(it, 1)) {
                    kdop::TestPair<V, K, T> tp = *it;
                    Tree<T, K> const* trees[2] = {tp.m_tree_a, tp.m_tree_b};
                    mesh_array::T4Mesh const* meshes[2] = {tp.m_mesh_a, tp.m_mesh_b};

                    // keep track of the degrees of the BVHs
                    size_t max_degrees[2] = {0, 0};
                    for (size_t t = 0; t < 2; ++t) {
                        // figure out object offsets into memory
                        //   based on whether we encounter this object for the first time
                        bool object_seen = object_offsets.count(trees[t]) == 1;
                        if (!object_seen) {
                            object_offset oo;
                            oo.node = nodes_size;
                            oo.tet = tets_size;
                            oo.vert = verts_size;
                            object_offsets.insert(std::pair< Tree<T, K> const*, object_offset >(
                                    trees[t],
                                    oo));

                            // keep track of the total number of tetrahedrons
                            //   and vertices processed so far
                            //   don't count already encountered objects twice
                            tets_size += meshes[t]->tetrahedron_size();
                            verts_size += meshes[t]->vertex_size();
                        }

                        // count total number of nodes
                        for(size_t h = trees[t]->number_of_levels(); h >= 1; --h) {
                            for (size_t b = 0; b < trees[t]->super_chunks(h - 1).size(); ++b) {
                                SubTree<T, K> branch = trees[t]->super_chunks(h - 1)[b];
                                max_bvh_height = std::max(max_bvh_height, branch.m_height);
                                for (size_t n = 0; n < trees[t]->super_chunks(h - 1)[b].m_nodes.size(); ++n) {
                                    if (!(branch.m_nodes[n].is_undefined())) {
                                        // don't count already encountered nodes twice
                                        nodes_size += !object_seen;
                                        max_degrees[t] = std::max(
                                                max_degrees[t],
                                                branch.m_nodes[n].m_end - branch.m_nodes[n].m_start + 1);
                                    }
                                }
                            }
                        }
                    }
                    
                    max_bvtt_degree = std::max(max_bvtt_degree, max_degrees[0] * max_degrees[1]);
                }

                // make room for all output
                *out_max_bvtt_degree = max_bvtt_degree;
                *out_max_bvtt_height = max_bvh_height;
                *out_nodes_size = nodes_size;
                *out_tets_size = tets_size;
                *out_verts_size = verts_size;
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    // align with cache line and page size on CPU
                    size_t size = *out_nodes_size * sizeof(KernelNode<KernelI, KernelT, K>);
                    size += size % global_mem_cacheline_size;
                    *out_nodes = (KernelNode<KernelI, KernelT, K>*) ALIGNED_ALLOC(4096, size);
                    
                    size = *out_tets_size * sizeof(KernelTetrahedron<KernelI>);
                    size += size % global_mem_cacheline_size;
                    *out_tets = (KernelTetrahedron<KernelI>*) ALIGNED_ALLOC(4096, size);
                    
                    size = *out_tets_size * sizeof(KernelTetrahedronSurfaceInfo);
                    size += size % global_mem_cacheline_size;
                    *out_tsi = (KernelTetrahedronSurfaceInfo*) ALIGNED_ALLOC(4096, size);
                    
                    size = *out_verts_size * sizeof(KernelV);
                    size += size % global_mem_cacheline_size;
                    *out_verts = (KernelV*) ALIGNED_ALLOC(4096, size);
                } else {
                    *out_nodes = new KernelNode<KernelI, KernelT, K>[*out_nodes_size];
                    *out_tets = new KernelTetrahedron<KernelI>[*out_tets_size];
                    *out_tsi = new KernelTetrahedronSurfaceInfo[*out_tets_size];
                    *out_verts = new KernelV[*out_verts_size];
                }
                *out_callbacks = new geometry::ContactsCallback<V>*[test_pairs.size()];
                
                // keep track of which objects have been copied already
                std::map < Tree<T, K> const*, bool > objects_copied;
                for (typename test_pair_container::iterator it = test_pairs.begin(); it != test_pairs.end(); std::advance(it, 1)) {
                    kdop::TestPair<V, K, T> tp = *it;
                    Tree<T, K> const* trees[2] = {tp.m_tree_a, tp.m_tree_b};
                    mesh_array::T4Mesh const* meshes[2] = {tp.m_mesh_a, tp.m_mesh_b};
                    mesh_array::TetrahedronAttribute<mesh_array::TetrahedronSurfaceInfo, mesh_array::T4Mesh> const* surface_maps[2]
                            = {tp.m_surface_map_a, tp.m_surface_map_b};

                    mesh_array::VertexAttribute<T, mesh_array::T4Mesh> const* Xs[2] = {tp.m_x_a, tp.m_x_b};
                    mesh_array::VertexAttribute<T, mesh_array::T4Mesh> const* Ys[2] = {tp.m_y_a, tp.m_y_b};
                    mesh_array::VertexAttribute<T, mesh_array::T4Mesh> const* Zs[2] = {tp.m_z_a, tp.m_z_b};

                    size_t test_pair_index = std::distance(test_pairs.begin(), it);
                    (*out_callbacks)[test_pair_index] = tp.m_callback;

                    // remember indices into root nodes for work item generation
                    std::vector< std::vector<KernelI> > chunk_root_indices[2];
                    chunk_root_indices[0].resize(trees[0]->number_of_levels());
                    chunk_root_indices[1].resize(trees[1]->number_of_levels());

                  for (size_t t = 0; t < 2; ++t)
                    {
                        bool object_copied = objects_copied.count(trees[t]) == 1;
                        // get the offsets for this tree into memory
                        object_offset oo = object_offsets[trees[t]];

                        // keep track of the branches as well
                        size_t node_branch_offset = 0, nodes_processed = 0;

                        // copy over the nodes for the chunks
                        for (size_t b = 0; b < trees[t]->branches().size(); ++b) {
                            SubTree<T, K> branch = trees[t]->branches()[b];
                            for (size_t n = 0; n < branch.m_nodes.size(); ++n) {
                                Node<T, K> node = branch.m_nodes[n];
                                size_t out_index = oo.node + node_branch_offset + n;
                                if (!node.is_undefined()) {
                                    if (!object_copied) { // only copy nodes of new objects ...
                                        if (node.is_leaf()) {
                                            // offset chunk leaves by tetrahedrons since they index into them
                                            (*out_nodes)[out_index].start
                                                    = (KernelI) (node.m_start + oo.tet);
                                            (*out_nodes)[out_index].end
                                                    = (KernelI) (node.m_end + oo.tet);
                                        } else {
                                            // offset internal nodes by nodes
                                            (*out_nodes)[out_index].start
                                                    = (KernelI) (node.m_start + oo.node + node_branch_offset);
                                            (*out_nodes)[out_index].end
                                                    = (KernelI) (node.m_end + oo.node + node_branch_offset);
                                        }
                                        for (size_t k = 0; k < K / 2; ++k) {
                                            (*out_nodes)[out_index].slabs[k].lower = (KernelT) node.m_volume(k).lower();
                                            (*out_nodes)[out_index].slabs[k].upper = (KernelT) node.m_volume(k).upper();
                                        }
                                    }
                                    ++nodes_processed;
                                    if (node.is_root()) { // ... but always generate root indices
                                        chunk_root_indices[t][trees[t]->number_of_levels() - 1].push_back((KernelI) out_index);
                                    }
                                }
                            }

                            // remember the number of nodes in the branches processed so far
                            node_branch_offset = nodes_processed;
                        }
                        
                        size_t node_previous_level_offset = 0;
                        
                        // repeat for all super chunk layers
                        for(size_t h = trees[t]->number_of_levels() - 1; h >= 1; --h) {
                            size_t node_current_level_offset = nodes_processed;
                            size_t current_chunk = 0;
                            for (size_t b = 0; b < trees[t]->super_chunks(h - 1).size(); ++b) {
                                SubTree<T, K> branch = trees[t]->super_chunks(h - 1)[b];
                                for (size_t n = 0; n < branch.m_nodes.size(); ++n) {
                                    Node<T, K> node = branch.m_nodes[n];
                                    size_t out_index = oo.node + node_branch_offset + n;
                                    if (!node.is_undefined()) {
                                        if (!object_copied) { // only copy nodes of new objects ...
                                            if (node.is_leaf()) {
                                                // offset super chunk leaves by levels since they index into the previous
                                                // assuming they all have the same number of nodes
                                                size_t chunk_size = trees[t]->super_chunks(h)[0].m_nodes.size();
                                                (*out_nodes)[out_index].start
                                                        = (KernelI) (node.m_start * chunk_size + oo.node + node_previous_level_offset);
                                                (*out_nodes)[out_index].end
                                                        = (KernelI) (node.m_end * chunk_size + oo.node + node_previous_level_offset);
                                                ++current_chunk;
                                            } else {
                                                // offset internal nodes by nodes
                                                (*out_nodes)[out_index].start
                                                        = (KernelI) (node.m_start + oo.node + node_branch_offset);
                                                (*out_nodes)[out_index].end
                                                        = (KernelI) (node.m_end + oo.node + node_branch_offset);
                                            }
                                            for (size_t k = 0; k < K / 2; ++k) {
                                                (*out_nodes)[out_index].slabs[k].lower = (KernelT) node.m_volume(k).lower();
                                                (*out_nodes)[out_index].slabs[k].upper = (KernelT) node.m_volume(k).upper();
                                            }
                                        }
                                        ++nodes_processed;
                                        if (node.is_root()) { // ... but always generate root indices
                                            chunk_root_indices[t][h - 1].push_back((KernelI) out_index);
                                        }
                                    }
                                }

                                // remember the number of nodes in the branches processed so far
                                node_branch_offset = nodes_processed;
                            }
                            
                            // remember the number of nodes at the beginning of this level
                            node_previous_level_offset = node_current_level_offset;
                        }

                        // copy over the meshes of new objects
                        if (!object_copied)
                        {
                            for (size_t tt = 0; tt < meshes[t]->tetrahedron_size(); ++tt)
                            {
                                mesh_array::Tetrahedron tet = meshes[t]->tetrahedron(tt);
                                size_t out_index = oo.tet + tet.idx();

                                (*out_tets)[out_index].vertex_idx[0] = (KernelI) (oo.vert + tet.i());
                                (*out_tets)[out_index].vertex_idx[1] = (KernelI) (oo.vert + tet.j());
                                (*out_tets)[out_index].vertex_idx[2] = (KernelI) (oo.vert + tet.k());
                                (*out_tets)[out_index].vertex_idx[3] = (KernelI) (oo.vert + tet.m());

                                mesh_array::TetrahedronSurfaceInfo vert_on_surf = surface_maps[t]->operator()(tet);
                                (*out_tsi)[out_index].vertex_on_surface[0] = (cl_uchar) vert_on_surf.m_i;
                                (*out_tsi)[out_index].vertex_on_surface[1] = (cl_uchar) vert_on_surf.m_j;
                                (*out_tsi)[out_index].vertex_on_surface[2] = (cl_uchar) vert_on_surf.m_k;
                                (*out_tsi)[out_index].vertex_on_surface[3] = (cl_uchar) vert_on_surf.m_m;

                                (*out_verts)[oo.vert + tet.i()].s[0] = (KernelT) Xs[t]->operator()(tet.i());
                                (*out_verts)[oo.vert + tet.i()].s[1] = (KernelT) Ys[t]->operator()(tet.i());
                                (*out_verts)[oo.vert + tet.i()].s[2] = (KernelT) Zs[t]->operator()(tet.i());
                                (*out_verts)[oo.vert + tet.j()].s[0] = (KernelT) Xs[t]->operator()(tet.j());
                                (*out_verts)[oo.vert + tet.j()].s[1] = (KernelT) Ys[t]->operator()(tet.j());
                                (*out_verts)[oo.vert + tet.j()].s[2] = (KernelT) Zs[t]->operator()(tet.j());
                                (*out_verts)[oo.vert + tet.k()].s[0] = (KernelT) Xs[t]->operator()(tet.k());
                                (*out_verts)[oo.vert + tet.k()].s[1] = (KernelT) Ys[t]->operator()(tet.k());
                                (*out_verts)[oo.vert + tet.k()].s[2] = (KernelT) Zs[t]->operator()(tet.k());
                                (*out_verts)[oo.vert + tet.m()].s[0] = (KernelT) Xs[t]->operator()(tet.m());
                                (*out_verts)[oo.vert + tet.m()].s[1] = (KernelT) Ys[t]->operator()(tet.m());
                                (*out_verts)[oo.vert + tet.m()].s[2] = (KernelT) Zs[t]->operator()(tet.m());
                            }

                            objects_copied.insert(std::pair < Tree<T, K> const*, bool>(trees[t], true));
                        }
                    }
                    
                    // cutoff value for work items
                    const size_t max_work_items = device_type & CL_DEVICE_TYPE_CPU ?
                        KDOP_CL_MAX_ROOT_PAIRS_CPU : KDOP_CL_MAX_ROOT_PAIRS_GPU;
                    const size_t min_height = std::min(
                        trees[0]->number_of_levels(), trees[1]->number_of_levels());
                    size_t level = 0;
                    for(; level < min_height; ++level) {
                        std::vector<KernelI> roots_a = chunk_root_indices[0][trees[0]->number_of_levels() - level - 1];
                        std::vector<KernelI> roots_b = chunk_root_indices[1][trees[1]->number_of_levels() - level - 1];
                        if(roots_a.size() * roots_b.size() <= max_work_items) {
                            break;
                        }
                    }

                    // in case no level could be found, take the highest
                    level = std::min(level, min_height - 1);
                    
                    // we cannot have a level higher that would result in an amount
                    // of kernel work items that are too many to allocate
                    level = std::min(level, (size_t) floor(log(max_kernel_work_items) / log(pow(max_bvtt_degree, max_bvh_height))));
                                        
                    // remember the root indices for this test pair so work items can be generated later
                    out_kernel_work_item_generator->add_roots(
                        chunk_root_indices[0][trees[0]->number_of_levels() - level - 1],
                        chunk_root_indices[1][trees[1]->number_of_levels() - level - 1],
                        (KernelI) test_pair_index, (KernelI) level);
                    
                }
            }
            
            template< typename V, size_t K, typename T,
            typename KernelI, typename KernelT, typename KernelV >
            inline void cleanup(
                    KernelNode<KernelI, KernelT, K> *out_nodes,
                    KernelTetrahedron<KernelI> *out_tets,
                    KernelTetrahedronSurfaceInfo *out_tsi,
                    KernelV *out_verts,
                    geometry::ContactsCallback<V> **out_callbacks,
                    cl_device_type device_type)
            {
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    free(out_nodes);
                    free(out_tets);
                    free(out_tsi);
                    free(out_verts);
                } else {
                    delete[] out_nodes;
                    delete[] out_tets;
                    delete[] out_tsi;
                    delete[] out_verts;
                }
                delete[] out_callbacks;
            }
            
            inline void record_kernel_times(  cl_ulong tandem_traversal_time
                                            , cl_ulong exact_test_time)
            {
                RECORD_TIME("tandem_traversal", (double) tandem_traversal_time / 1000000.0);
                RECORD_TIME("exact_test", (double) exact_test_time / 1000000.0);
            }
            
            inline void record_kernel_invocations(  size_t tandem_traversal_invocations
                                                  , size_t exact_test_invocations)
            {
                RECORD("tandem_traversal_invocations", tandem_traversal_invocations);
                RECORD("exact_test_invocations", exact_test_invocations);
            }

            inline void record_opencl_times(  cl_ulong tandem_traversal_read_time, cl_ulong tandem_traversal_write_time
                                            , cl_ulong exact_test_read_time, cl_ulong exact_test_write_time)
            {
                RECORD_TIME("tandem_traversal_opencl_read_time", (double) tandem_traversal_read_time / 1000000.0);
                RECORD_TIME("tandem_traversal_opencl_write_time", (double) tandem_traversal_write_time / 1000000.0);
                RECORD_TIME("exact_test_opencl_read_time", (double) exact_test_read_time / 1000000.0);
                RECORD_TIME("exact_test_opencl_write_time", (double) exact_test_write_time / 1000000.0);
            }

        } // namespace cl
    
    } // namespace details

    template< typename V, size_t K, typename T, typename test_pair_container >
    inline void tandem_traversal(test_pair_container test_pairs,
                                 dikucl const & /* tag */,
                                 size_t open_cl_platform = 0,
                                 size_t open_cl_device = 0) {
        // assert( ! test_pairs.empty() || !"tandem_traversal : test_pairs are empty" );
        if ( test_pairs.empty() ) {
          details::cl::record_kernel_times(0, 0);
          details::cl::record_kernel_invocations(0, 0);
          details::cl::record_opencl_times(0, 0, 0, 0);
          RECORD_TIME("tandem_traversal_opencl_init", (double) 0.0);
          RECORD_TIME("tandem_traversal_opencl_host_time", (double) 0.0);
          RECORD_TIME("tandem_traversal_opencl_prepare_input", (double) 0.0);
          RECORD_TIME("sort_and_report_contact_points", (double) 0.0);
          return;
        }

        START_TIMER("tandem_traversal_opencl_init");
        
        cl_ulong tandem_traversal_kernel_time       = 0;
        cl_ulong exact_tests_kernel_time            = 0;
        cl_ulong tandem_traversal_opencl_write_time = 0;
        cl_ulong tandem_traversal_opencl_read_time  = 0;
        cl_ulong exact_test_opencl_write_time       = 0;
        cl_ulong exact_test_opencl_read_time        = 0;
        
        size_t tandem_traversal_invocations = 0;
        size_t exact_test_invocations       = 0;

        // TODO
        // make the chunk size depend on the available local memory:
        // more available local memory can mean larger work groups and/or
        // larger (= deeper) BVTTs, resulting from deeper BVHs, resulting
        // from larger chunks

        cl_int err = CL_SUCCESS;

        ::dikucl::PlatformHandle *platform_handle = ::dikucl::PlatformManager::get_instance().get_platform(
                &err, open_cl_platform);
        CHECK_CL_ERR(err);

        ::dikucl::DeviceHandle *device_handle = ::dikucl::DeviceManager::get_instance().get_device(
                &err, platform_handle, open_cl_device);
        CHECK_CL_ERR(err);
        cl_device_type device_type = device_handle->device.getInfo<CL_DEVICE_TYPE>(&err);
        CHECK_CL_ERR(err);
        cl_uint max_compute_units = device_handle->device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>(&err);
        CHECK_CL_ERR(err);
        size_t global_mem_cacheline_size =
                (size_t) device_handle->device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>(&err);
        CHECK_CL_ERR(err);
        size_t max_mem_alloc_size =
                (size_t) device_handle->device.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>(&err);
        CHECK_CL_ERR(err);

        // default context
        ::dikucl::ContextHandle *context_handle =
                ::dikucl::ContextManager::get_instance().get_context(  &err
                                                                     , device_handle);
        CHECK_CL_ERR(err);
        ::cl::Context context = context_handle->context;

        // get command queue for that context
        ::dikucl::CommandQueueHandle *command_queue_handle;
#ifdef USE_PROFILING
        command_queue_handle =
                ::dikucl::CommandQueueManager::get_instance().get_command_queue(
                &err,
                context_handle,
                CL_QUEUE_PROFILING_ENABLE);
#else
        command_queue_handle =
                ::dikucl::CommandQueueManager::get_instance().get_command_queue(
                &err,
                context_handle);
#endif // USE_PROFILING
        CHECK_CL_ERR(err);
        ::cl::CommandQueue queue = command_queue_handle->command_queue;

        // number of work groups that we want in parallel per compute unit
        //   this influences the choice of the backlog size
        //   subject to increase in the future
        // only matters go GPUs
        size_t max_num_work_groups_per_compute_unit = 4;

        // number of work items we want per compute unit
        //   subject to increase in the future
        const size_t num_work_items_per_compute_unit = 2048;

        // a couple of types for the kernel
        typedef cl_uint KI; const std::string kernel_index_type = "uint";
        typedef cl_float KT;
        typedef cl_float3 KV;

        // maximum number of kernel work items we can fit onto the GPU in one call
        const size_t max_kernel_work_items =
                (max_mem_alloc_size - global_mem_cacheline_size) / sizeof(details::cl::KernelWorkItem<KI>);

        // check for double precision support
        std::string device_extensions = device_handle->device.getInfo<CL_DEVICE_EXTENSIONS>(&err);
        CHECK_CL_ERR(err);
        const bool use_double_precision = false; // device_extensions.find("cl_khr_fp64") != std::string::npos;

        // figure out whether we're on a NVIDIA GPU
        if( (device_type & CL_DEVICE_TYPE_GPU)
            && device_extensions.find("cl_nv_device_attribute_query") != std::string::npos)
        {
            // if so, check compute capability to figure out how many
            // warp schedulers this device has and adjust the maximum
            // number of parallel work groups per compute unit accordingly
            cl_uint cc_major =
                    device_handle->device.getInfo<CL_DEVICE_COMPUTE_CAPABILITY_MAJOR_NV>(&err);
            CHECK_CL_ERR(err);
            switch(cc_major) {
                case 1:
                    // Tesla, one warp scheduler
                    max_num_work_groups_per_compute_unit = 1;
                    break;
                case 2:
                    // Fermi, two warp schedulers
                    max_num_work_groups_per_compute_unit = 2;
                    break;
                case 3:
                    // Kepler, four warp schedulers, fast global atomics
                    max_num_work_groups_per_compute_unit = 4;
                    break;
                default:
                    break;
            }
        }

        // get maximum local work sizes
        //   we will be using only dimension 0
        std::vector<size_t> max_work_item_sizes = device_handle->device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>(&err);
        CHECK_CL_ERR(err);
        size_t max_global_work_size =
                std::max(num_work_items_per_compute_unit * max_compute_units, max_work_item_sizes[0]);

        // check how much local memory we have available
        const size_t available_local_memory = (size_t) device_handle->device.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>(&err);
        CHECK_CL_ERR(err);

        /* transform input */

        details::cl::KernelNode<KI, KT, K> *kernel_nodes;
        details::cl::KernelTetrahedron<KI> *kernel_tets;
        details::cl::KernelTetrahedronSurfaceInfo *kernel_tsi;
        details::cl::KernelWorkItem<KI> *kernel_work;
        geometry::ContactsCallback<V> **kernel_callbacks;
        KV *kernel_verts;
        size_t kernel_nodes_size, kernel_tets_size, kernel_verts_size;
        size_t max_bvtt_degree, max_bvtt_height;

        details::cl::KernelWorkItemGenerator<KI> kernel_work_item_generator;
        START_TIMER("tandem_traversal_opencl_prepare_input");
        details::cl::prepare_input<V, K, T, KI, KT, KV>(
                test_pairs,
                &kernel_nodes, &kernel_nodes_size,
                &kernel_tets, &kernel_tets_size, &kernel_tsi,
                &kernel_verts, &kernel_verts_size,
                &kernel_work_item_generator,
                &kernel_callbacks,
                &max_bvtt_degree, &max_bvtt_height,
                device_type, global_mem_cacheline_size,
                max_kernel_work_items);
        STOP_TIMER("tandem_traversal_opencl_prepare_input");

        // maximum number of elements that a work item will push onto its stack
        // only the leaf level needs to fully fit, for the others we can
        // reuse one position because one item is popped off immediately
        // because of depth-first traversal
        size_t max_backlog_per_work_item =
                (max_bvtt_degree - 1) * (max_bvtt_height - 1) + max_bvtt_degree;

        // get the tandem traversal kernel
        ::dikucl::KernelInfo tandem_traversal_kernel_info(
                details::cl::kernels_path,
                "kdop_cl_tandem_traversal.cl",
                device_type & CL_DEVICE_TYPE_CPU ? "do_tandem_traversal_cpu" : "do_tandem_traversal");
        tandem_traversal_kernel_info.define("__K", K);
        tandem_traversal_kernel_info.define("__INDEX_TYPE", kernel_index_type);
        tandem_traversal_kernel_info.define("__MAX_BVTT_DEGREE", max_bvtt_degree);
        if(use_double_precision) {
            tandem_traversal_kernel_info.define("__USE_DOUBLE_PRECISION", 1);
        }
        tandem_traversal_kernel_info.include(details::cl::kernels_path).no_signed_zeros(true);
#ifndef NDEBUG_DIKUCL
        // enable verbose compilation in debug builds
        tandem_traversal_kernel_info.nv_verbose(true).debug_intel(true);
        tandem_traversal_kernel_info.define("__MAX_ERROR_CODES", MAX_ERROR_CODES);
#else // NDEBUG_DIKUCL
        tandem_traversal_kernel_info.define("__NDEBUG_DIKUCL", 1);
#endif // NDEBUG_DIKUCL
        ::dikucl::KernelHandle *tandem_traversal_kernel_handle = ::dikucl::KernelManager::get_instance().get_kernel(
                tandem_traversal_kernel_info,
                &err,
                context_handle);
        CHECK_CL_ERR(err);
        ::cl::Kernel tandem_traversal_kernel = tandem_traversal_kernel_handle->kernel;

        /* set static parameters */

#ifdef USE_PROFILING
        // Because we collect profiling data in the kernel invocations loop, we use this flag to indicate whether
        // to include the one-time profiling recordings as well.
        bool record_per_simulation_step_times = true;
#endif

        // the BVH as an array of nodes
        ::cl::Buffer nodes;
#ifdef USE_PROFILING
        ::cl::Event nodes_event;
#endif // USE_PROFILING
        if(device_type & CL_DEVICE_TYPE_CPU) {
            size_t size = sizeof (details::cl::KernelNode<KI, KT, K>) * kernel_nodes_size;
            size += size % global_mem_cacheline_size;
            nodes = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                    size,
                    kernel_nodes, &err);
            CHECK_CL_ERR(err);
        } else {
            nodes = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY,
                    sizeof (details::cl::KernelNode<KI, KT, K>) * kernel_nodes_size,
                    NULL, &err);
            CHECK_CL_ERR(err);
            err = queue.enqueueWriteBuffer(
                    nodes, CL_FALSE,
                    0, sizeof (details::cl::KernelNode<KI, KT, K>) * kernel_nodes_size,
                    kernel_nodes, NULL,
#ifdef USE_PROFILING
                    &nodes_event);
#else // USE_PROFILING
                    NULL);
#endif // USE_PROFILING
            CHECK_CL_ERR(err);
            FINISH_QUEUE(queue);
        }

        err = tandem_traversal_kernel.setArg(0, nodes);
        CHECK_CL_ERR(err);
#ifndef NDEBUG_DIKUCL
        err = tandem_traversal_kernel.setArg(10, (cl_uint) kernel_nodes_size);
        CHECK_CL_ERR(err);
#endif // NDEBUG_DIKUCL

        // needed for figuring out how many work items we want per work group, eventually
        const size_t preferred_work_group_size_multiple =
            tandem_traversal_kernel.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device_handle->device, &err);
        CHECK_CL_ERR(err);

        // get the exact tests kernel
        ::dikucl::KernelInfo exact_tests_kernel_info(
                details::cl::kernels_path,
                "kdop_cl_exact_tests.cl",
                "do_exact_tests");
        exact_tests_kernel_info.define("__INDEX_TYPE", kernel_index_type);
        if(device_type & CL_DEVICE_TYPE_CPU) {
            exact_tests_kernel_info.define("__EARLY_REJECTION", 1);
        }
        if(use_double_precision) {
            exact_tests_kernel_info.define("__USE_DOUBLE_PRECISION", 1);
        }
        exact_tests_kernel_info.include(details::cl::kernels_path).no_signed_zeros(true);
#ifndef NDEBUG_DIKUCL
        exact_tests_kernel_info.nv_verbose(true).debug_intel(true);
#endif // NDEBUG_DIKUCL
        ::dikucl::KernelHandle *exact_tests_kernel_handle = ::dikucl::KernelManager::get_instance().get_kernel(
                exact_tests_kernel_info,
                &err,
                context_handle);
        CHECK_CL_ERR(err);
        ::cl::Kernel exact_tests_kernel = exact_tests_kernel_handle->kernel;

        /* set static parameters */

        // tetrahedrons
        ::cl::Buffer tetrahedrons;
#ifdef USE_PROFILING
        ::cl::Event tetrahedrons_event;
#endif // USE_PROFILING
        if(device_type & CL_DEVICE_TYPE_CPU) {
            size_t size = sizeof (details::cl::KernelTetrahedron<KI>) * kernel_tets_size;
            size += size % global_mem_cacheline_size;
            tetrahedrons = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                    size,
                    kernel_tets, &err);
            CHECK_CL_ERR(err);
        } else {
            tetrahedrons = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY,
                    sizeof (details::cl::KernelTetrahedron<KI>) * kernel_tets_size,
                    NULL, &err);
            CHECK_CL_ERR(err);
            err = queue.enqueueWriteBuffer(
                    tetrahedrons, CL_FALSE,
                    0, sizeof (details::cl::KernelTetrahedron<KI>) * kernel_tets_size,
                    kernel_tets, NULL,
#ifdef USE_PROFILING
                    &tetrahedrons_event);
#else // USE_PROFILING
                    NULL);
#endif
            CHECK_CL_ERR(err);
            FINISH_QUEUE(queue);
        }
        err = exact_tests_kernel.setArg(0, tetrahedrons);
        CHECK_CL_ERR(err);

        // tetrahedron surface info
        ::cl::Buffer tetrahedron_surface_info;
#ifdef USE_PROFILING
        ::cl::Event tetrahedron_surface_info_event;
#endif // USE_PROFILING
        if(device_type & CL_DEVICE_TYPE_CPU) {
            size_t size = sizeof (details::cl::KernelTetrahedronSurfaceInfo) * kernel_tets_size;
            size += size % global_mem_cacheline_size;
            tetrahedron_surface_info = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                    size,
                    kernel_tsi, &err);
            CHECK_CL_ERR(err);
        } else {
            tetrahedron_surface_info = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY,
                    sizeof (details::cl::KernelTetrahedronSurfaceInfo) * kernel_tets_size,
                    NULL, &err);
            CHECK_CL_ERR(err);
            err = queue.enqueueWriteBuffer(
                    tetrahedron_surface_info, CL_FALSE,
                    0, sizeof (details::cl::KernelTetrahedronSurfaceInfo) * kernel_tets_size,
                    kernel_tsi, NULL,
#ifdef USE_PROFILING
                    &tetrahedron_surface_info_event);
#else // USE_PROFILING
                    NULL);
#endif // USE_PROFILING
            CHECK_CL_ERR(err);
            FINISH_QUEUE(queue);
        }
        err = exact_tests_kernel.setArg(1, tetrahedron_surface_info);
        CHECK_CL_ERR(err);

        // X, Y, Z coordinates
        ::cl::Buffer vertices;
#ifdef USE_PROFILING
        ::cl::Event vertices_event;
#endif // USE_PROFILING
        if(device_type & CL_DEVICE_TYPE_CPU) {
            size_t size = sizeof (KV) * kernel_verts_size;
            size += size % global_mem_cacheline_size;
            vertices = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                    size,
                    kernel_verts, &err);
            CHECK_CL_ERR(err);
        } else {
            vertices = ::cl::Buffer(
                    context, CL_MEM_READ_ONLY,
                    sizeof (KV) * kernel_verts_size,
                    NULL, &err);
            CHECK_CL_ERR(err);
            err = queue.enqueueWriteBuffer(vertices, CL_FALSE, 0, sizeof (KV) * kernel_verts_size,
                    kernel_verts, NULL,
#ifdef USE_PROFILING
                    &vertices_event);
#else // USE_PROFILING
                    NULL);
#endif // USE_PROFILING
            CHECK_CL_ERR(err);
            FINISH_QUEUE(queue);
        }
        err = exact_tests_kernel.setArg(2, vertices);
        CHECK_CL_ERR(err);

        // same for exact tests
        ::cl::Buffer exact_tests;
        size_t max_exact_tests = max_mem_alloc_size / (20 * sizeof(details::cl::KernelContactPoint<KV, KT, KI>));
        if(device_type & CL_DEVICE_TYPE_CPU) {
            // for each exact test we can have up to 20 contact points,
            // so don't generate more exact tests than contact points that
            // we can handle.
            exact_tests = ::cl::Buffer(
                    context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                    max_exact_tests * sizeof(details::cl::KernelWorkItem<KI>),
                    NULL, &err);
            CHECK_CL_ERR(err);
        } else {
            exact_tests = ::cl::Buffer(
                    context, CL_MEM_READ_WRITE,
                    max_exact_tests * sizeof(details::cl::KernelWorkItem<KI>),
                    NULL, &err);
            CHECK_CL_ERR(err);
        }
        err = tandem_traversal_kernel.setArg(3, exact_tests);
        CHECK_CL_ERR(err);
        err = tandem_traversal_kernel.setArg(5, (cl_uint) max_exact_tests);
        CHECK_CL_ERR(err);

        // collect contact points first
        details::cl::KernelContactPoint<KV, KT, KI> *contact_point_container = NULL;
        size_t contact_point_container_size = 0;

        STOP_TIMER("tandem_traversal_opencl_init");
        START_TIMER("tandem_traversal_opencl_host_time");

        // as long as we have unfinished work items ...
        while(!kernel_work_item_generator.empty()) {
            // we don't want too many super chunk test pairs because the kernel
            // puts their leaf-leaf tests in the global backlog which is limited in size
            // so count each super chunk pair in terms of how many (super) chunk pairs
            // they can generate so we can set the global backlog accordingly
            size_t weight_super_chunk_tp = pow(max_bvtt_degree, max_bvtt_height);
            size_t num_super_chunks;
            size_t kernel_root_pairs_size = kernel_work_item_generator.generate_kernel_work_items(
                    max_kernel_work_items, weight_super_chunk_tp,
                    &kernel_work,
                    device_type, global_mem_cacheline_size,
                    max_bvtt_degree, max_bvtt_height,
                    max_global_work_size, &num_super_chunks);

            // setup backlog

            // subtract what the kernel takes up
            size_t tandem_traversal_local_memory =
                    available_local_memory - (size_t) tandem_traversal_kernel.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(device_handle->device, &err);
            CHECK_CL_ERR(err);

            // number of KernelWorkItems that each work group can cache
            //   each work item can then cache backlog_size / local_work_size KernelWorkItems
            size_t kernel_work_size = kernel_root_pairs_size;
            size_t backlog_size = 0;
            ::cl::Buffer backlog;
            if(device_type & CL_DEVICE_TYPE_CPU) {
                backlog_size = std::max(
                                kernel_work_size,
                                max_backlog_per_work_item * max_global_work_size);
                err = tandem_traversal_kernel.setArg(9, (cl_uint) backlog_size);
                CHECK_CL_ERR(err);
                backlog = ::cl::Buffer(
                        context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                        backlog_size * sizeof(details::cl::KernelWorkItem<KI>),
                        NULL, &err);
                CHECK_CL_ERR(err);
                err = tandem_traversal_kernel.setArg(8, backlog);
                CHECK_CL_ERR(err);
            } else {
                // no local memory backlog on CPUs
                // on GPUs we have the problem that work items
                // push to local memory
                backlog_size =
                        tandem_traversal_local_memory / (sizeof(details::cl::KernelWorkItem<KI>) * max_num_work_groups_per_compute_unit);
                err = tandem_traversal_kernel.setArg(8, sizeof(details::cl::KernelWorkItem<KI>) * backlog_size, NULL);
                CHECK_CL_ERR(err);
                err = tandem_traversal_kernel.setArg(9, (cl_uint) backlog_size);
                CHECK_CL_ERR(err);
            }

#ifdef USE_PROFILING
            // similar to record_per_simulation_step_times (see above)
            bool record_per_work_item_generation_times = true;
#endif

            // setup two work buffers
            //   one of them holds the work of the current iteration
            //   the other receives the work for the next iteration
            // see details::cl::prepare_input for an explanation about the sizes
            //   of the work buffers
            size_t current_work_buffer = 0;
            ::cl::Buffer work[2];
#ifdef USE_PROFILING
            ::cl::Event work_event;
#endif // USE_PROFILING
            size_t kernel_work_buffer_size = std::max(
                    std::max(
                            kernel_work_size,
                            max_backlog_per_work_item * max_global_work_size),
                    kernel_work_size + num_super_chunks * weight_super_chunk_tp);
            if(device_type & CL_DEVICE_TYPE_CPU) {
                // keep in mind that super chunk test pairs fill up the global backlog
                size_t size = kernel_work_buffer_size * sizeof(details::cl::KernelWorkItem<KI>);
                size += size % global_mem_cacheline_size;
                work[current_work_buffer] = ::cl::Buffer(
                        context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                        size,
                        kernel_work, &err);
                CHECK_CL_ERR(err);
            } else {
                work[current_work_buffer] = ::cl::Buffer(
                        context, CL_MEM_READ_WRITE,
                        kernel_work_buffer_size * sizeof(details::cl::KernelWorkItem<KI>),
                        NULL, &err);
                CHECK_CL_ERR(err);
                err = queue.enqueueWriteBuffer(
                        work[current_work_buffer], CL_FALSE,
                        0, sizeof (details::cl::KernelWorkItem<KI>) * kernel_work_size,
                        kernel_work, NULL,
#ifdef USE_PROFILING
                        &work_event);
#else // USE_PROFILING
                        NULL);
#endif // USE_PROFILING
                CHECK_CL_ERR(err);
                FINISH_QUEUE(queue);
            }
            err = tandem_traversal_kernel.setArg(1, work[current_work_buffer]);
            CHECK_CL_ERR(err);

            if(device_type & CL_DEVICE_TYPE_CPU) {
                work[1 - current_work_buffer] = ::cl::Buffer(
                        context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                        kernel_work_buffer_size * sizeof(details::cl::KernelWorkItem<KI>),
                        NULL, &err);
                CHECK_CL_ERR(err);
            } else {
                work[1 - current_work_buffer] = ::cl::Buffer(
                        context, CL_MEM_READ_WRITE,
                        kernel_work_buffer_size * sizeof(details::cl::KernelWorkItem<KI>),
                        NULL, &err);
                CHECK_CL_ERR(err);
            }
            err = tandem_traversal_kernel.setArg(6, work[1 - current_work_buffer]);
            CHECK_CL_ERR(err);
#ifndef NDEBUG_DIKUCL
            err = tandem_traversal_kernel.setArg(11, (cl_uint) kernel_work_buffer_size);
            CHECK_CL_ERR(err);
#endif // NDEBUG_DIKUCL

            // iterate kernel invocations for work load balancing
            while (kernel_work_size > 0) {
                err = tandem_traversal_kernel.setArg(2, (cl_uint) kernel_work_size);
                CHECK_CL_ERR(err);

                // setup exact tests size counter
                cl_uint kernel_exact_tests_size = 0;
                ::cl::Buffer exact_tests_size;
#ifdef USE_PROFILING
                ::cl::Event exact_tests_size_event;
#endif // USE_PROFILING
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    size_t size = sizeof(cl_uint);
                    size += size % global_mem_cacheline_size;
                    exact_tests_size = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                            size,
                            &kernel_exact_tests_size, &err);
                    CHECK_CL_ERR(err);
                } else {
                    exact_tests_size = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE,
                            sizeof (cl_uint),
                            NULL, &err);
                    CHECK_CL_ERR(err);
                    err = queue.enqueueWriteBuffer(
                            exact_tests_size, CL_FALSE,
                            0, sizeof (cl_uint),
                            &kernel_exact_tests_size, NULL,
#ifdef USE_PROFILING
                            &exact_tests_size_event);
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                }
                err = tandem_traversal_kernel.setArg(4, exact_tests_size);
                CHECK_CL_ERR(err);

                // setup global backlog size counter
                cl_uint kernel_global_backlog_size = 0;
                ::cl::Buffer global_backlog_size;
#ifdef USE_PROFILING
                ::cl::Event global_backlog_size_event;
#endif // USE_PROFILING
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    size_t size = sizeof(cl_uint);
                    size += size % global_mem_cacheline_size;
                    global_backlog_size = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                            size,
                            &kernel_global_backlog_size, &err);
                    CHECK_CL_ERR(err);
                } else {
                    global_backlog_size = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE,
                            sizeof (cl_uint),
                            NULL, &err);
                    CHECK_CL_ERR(err);
                    err = queue.enqueueWriteBuffer(
                            global_backlog_size, CL_FALSE,
                            0, sizeof (cl_uint),
                            &kernel_global_backlog_size, NULL,
#ifdef USE_PROFILING
                            &global_backlog_size_event);
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                }
                err = tandem_traversal_kernel.setArg(7, global_backlog_size);
                CHECK_CL_ERR(err);

                // figure out how many work items we can have per work group such that
                //   each work item can have a stack that can accommodate pushing
                //   of a full path down one BVTT
                size_t local_work_size =
                        tandem_traversal_kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device_handle->device, &err);
                CHECK_CL_ERR(err);
                local_work_size = std::min(local_work_size, max_work_item_sizes[0]);
                if(!(device_type & CL_DEVICE_TYPE_CPU)) {
                    // backlog is not the limiting factor on CPUs
                    for(; local_work_size > preferred_work_group_size_multiple
                        ; local_work_size -= preferred_work_group_size_multiple)
                    {
                        // have at least a local work size of the preferred multiple
                        // of course it'd be better to be able to push a full BVTT down
                        // the backlog for each work item
                        if(backlog_size / local_work_size >= max_backlog_per_work_item) {
                            break;
                        }
                    }
                }

                size_t global_work_size = std::max(num_work_items_per_compute_unit * max_compute_units, local_work_size);
                if(global_work_size % local_work_size > 0) {
                    global_work_size -= global_work_size % local_work_size;
                }

#ifndef NDEBUG_DIKUCL
                cl_uint kernel_error_codes[MAX_ERROR_CODES];
                ::cl::Buffer error_codes;
#ifdef USE_PROFILING
                ::cl::Event error_codes_event;
#endif // USE_PROFILING
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    size_t size = MAX_ERROR_CODES * sizeof(cl_uint);
                    size += size % global_mem_cacheline_size;
                    error_codes = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                            size,
                            &kernel_error_codes, &err);
                    CHECK_CL_ERR(err);
                } else {
                    error_codes = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE,
                            MAX_ERROR_CODES * sizeof (cl_uint),
                            NULL, &err);
                    CHECK_CL_ERR(err);
                    err = queue.enqueueWriteBuffer(
                            error_codes, CL_FALSE,
                            0, MAX_ERROR_CODES * sizeof (cl_uint),
                            &kernel_error_codes, NULL,
#ifdef USE_PROFILING
                            &error_codes_event);
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                }
                err = tandem_traversal_kernel.setArg(12, error_codes);
                CHECK_CL_ERR(err);

                cl_uint kernel_error_size = 0;
                ::cl::Buffer error_size;
#ifdef USE_PROFILING
                ::cl::Event error_size_event;
#endif // USE_PROFILING
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    size_t size = sizeof(cl_uint);
                    size += size % global_mem_cacheline_size;
                    error_size = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                            size,
                            &kernel_error_size, &err);
                    CHECK_CL_ERR(err);
                } else {
                    error_size = ::cl::Buffer(
                            context, CL_MEM_READ_WRITE,
                            sizeof (cl_uint),
                            NULL, &err);
                    CHECK_CL_ERR(err);
                    err = queue.enqueueWriteBuffer(
                            error_size, CL_FALSE,
                            0, sizeof (cl_uint),
                            &kernel_error_size, NULL,
#ifdef USE_PROFILING
                            &error_size_event);
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                }
                err = tandem_traversal_kernel.setArg(13, error_size);
                CHECK_CL_ERR(err);
#endif // NDEBUG_DIKUCL

                // generate exact tests!
#ifdef USE_PROFILING
                {
                    ::cl::Event tt_profiling;
                    err = queue.enqueueNDRangeKernel(
                            tandem_traversal_kernel,
                            ::cl::NullRange,
                            ::cl::NDRange(global_work_size),
                            ::cl::NDRange(local_work_size),
                            NULL,
                            &tt_profiling);
                    CHECK_CL_ERR(err);
                    err = queue.finish();
                    CHECK_CL_ERR(err);
                    cl_ulong start = tt_profiling.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                    CHECK_CL_ERR(err);
                    cl_ulong end = tt_profiling.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                    CHECK_CL_ERR(err);
                    tandem_traversal_kernel_time += end - start;

                    if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
                        // on the CPU we use host memory, no transactions to profile

                        // these are events that happen just once per overall simulation step
                        if (record_per_simulation_step_times) {
                            ::cl::Event events[] = { nodes_event, tetrahedrons_event, tetrahedron_surface_info_event, vertices_event };
                            for (short i = 0; i < 4; ++i) {
                                start = events[i].getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                                CHECK_CL_ERR(err);
                                end = events[i].getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                                CHECK_CL_ERR(err);
                                tandem_traversal_opencl_write_time += end - start;
                            }

                            // don't record these times again in future loop iterations
                            record_per_simulation_step_times = false;
                        }

                        // these are times that happen once per generated set of kernel work items
                        if (record_per_work_item_generation_times) {
                            start = work_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                            CHECK_CL_ERR(err);
                            end = work_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                            CHECK_CL_ERR(err);
                            tandem_traversal_opencl_write_time += end - start;

                            // don't record this time again in future loop iterations
                            record_per_work_item_generation_times = false;
                        }

                        // these are events that happen once every kernel invocation
                        short num_events;
                        ::cl::Event events[] = {
                                exact_tests_size_event, global_backlog_size_event
#ifndef NDEBUG_DIKUCL
                                , error_codes_event, error_size_event
                        };
                        num_events = 4;
#else // NDEBUG_DIKUCL
                        };
                        num_events = 2;
#endif // NDEBUG_DIKUCL
                        for (short i = 0; i < num_events; ++i) {
                            start = events[i].getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                            CHECK_CL_ERR(err);
                            end = events[i].getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                            CHECK_CL_ERR(err);
                            tandem_traversal_opencl_write_time += end - start;
                        }
                    }
                }
#else
                err = queue.enqueueNDRangeKernel(
                        tandem_traversal_kernel,
                        ::cl::NullRange,
                        ::cl::NDRange(global_work_size),
                        ::cl::NDRange(local_work_size));
                CHECK_CL_ERR(err);
                FINISH_QUEUE(queue);
#endif // USE_PROFILING

#ifndef NDEBUG_DIKUCL
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    cl_uint *mapped_kernel_error_size = (cl_uint*) queue.enqueueMapBuffer(
                            error_size, CL_TRUE,
                            CL_MAP_READ,
                            0, sizeof(cl_uint),
                            NULL, NULL, &err);
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                    kernel_error_size = *mapped_kernel_error_size;
                    err = queue.enqueueUnmapMemObject(
                            error_size,
                            (void*) mapped_kernel_error_size);
                    CHECK_CL_ERR(err);
                } else {
                    err = queue.enqueueReadBuffer(
                            error_size, CL_TRUE,
                            0, sizeof (cl_uint),
                            &kernel_error_size, NULL,
#ifdef USE_PROFILING
                            &error_size_event);
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                }
                FINISH_QUEUE(queue);

                if(kernel_error_size != 0) {
                    if(device_type & CL_DEVICE_TYPE_CPU) {
                        cl_uint *mapped_kernel_error_codes = (cl_uint*) queue.enqueueMapBuffer(
                                error_codes, CL_TRUE,
                                CL_MAP_READ,
                                0, kernel_error_size * sizeof(cl_uint),
                                NULL, NULL, &err);
                        CHECK_CL_ERR(err);
                        FINISH_QUEUE(queue);
                        memcpy(kernel_error_codes, mapped_kernel_error_codes, (size_t) kernel_error_size * sizeof(cl_uint));
                        err = queue.enqueueUnmapMemObject(
                                error_codes,
                                (void*) mapped_kernel_error_codes);
                        CHECK_CL_ERR(err);
                    } else {
                        err = queue.enqueueReadBuffer(
                                error_codes, CL_TRUE,
                                0, (size_t) kernel_error_size * sizeof (cl_uint),
                                kernel_error_codes, NULL,
#ifdef USE_PROFILING
                                &error_codes_event);
#else // USE_PROFILING
                                NULL);
#endif // USE_PROFILING
                        CHECK_CL_ERR(err);
                    }
                    FINISH_QUEUE(queue);

                  {
                    util::Log logging;

                    logging << "Tandem traversal kernel generated the following "
                            << kernel_error_size
                            << " error codes:"
                            << util::Log::newline();


                    for(size_t i = 0; i < kernel_error_size; ++i)
                    {
                      logging << kernel_error_codes[i] << util::Log::newline();
                    }

                  }

                  exit(1);
                }
#endif // NDEBUG_DIKUCL

                ++tandem_traversal_invocations;

                // perform any exact tests
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    cl_uint *mapped_kernel_exact_tests_size = (cl_uint*) queue.enqueueMapBuffer(
                            exact_tests_size, CL_TRUE,
                            CL_MAP_READ,
                            0, sizeof(cl_uint),
                            NULL, NULL, &err);
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                    kernel_exact_tests_size = *mapped_kernel_exact_tests_size;
                    err = queue.enqueueUnmapMemObject(
                            exact_tests_size,
                            (void*) mapped_kernel_exact_tests_size);
                    CHECK_CL_ERR(err);
                } else {
                    err = queue.enqueueReadBuffer(
                            exact_tests_size, CL_TRUE,
                            0, sizeof (cl_uint),
                            &kernel_exact_tests_size, NULL,
#ifdef USE_PROFILING
                            &exact_tests_size_event);
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                }
                FINISH_QUEUE(queue);

                if (kernel_exact_tests_size > 0) {                    
                    // set exact tests as input to exact tests kernel
                    err = exact_tests_kernel.setArg(3, exact_tests);
                    CHECK_CL_ERR(err);
                    err = exact_tests_kernel.setArg(4, kernel_exact_tests_size);
                    CHECK_CL_ERR(err);

                    // make room for generated contact points
                    ::cl::Buffer contact_points;
                    if(device_type & CL_DEVICE_TYPE_CPU) {
                        contact_points = ::cl::Buffer(
                            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
                            sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * kernel_exact_tests_size * 20,
                            NULL, &err);
                        CHECK_CL_ERR(err);
                    } else {
                        contact_points = ::cl::Buffer(
                            context, CL_MEM_WRITE_ONLY,
                            sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * kernel_exact_tests_size * 20,
                            NULL, &err);
                        CHECK_CL_ERR(err);
                    }
                    err = exact_tests_kernel.setArg(5, contact_points);
                    CHECK_CL_ERR(err);

                    cl_uint kernel_contact_points_size = 0;
                    ::cl::Buffer contact_points_size;
#ifdef USE_PROFILING
                    ::cl::Event contact_points_size_event;
#endif // USE_PROFILING
                    if(device_type & CL_DEVICE_TYPE_CPU) {
                        size_t size = sizeof (cl_uint);
                        size += size % global_mem_cacheline_size;
                        contact_points_size = ::cl::Buffer(
                                context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                size,
                                &kernel_contact_points_size, &err);
                        CHECK_CL_ERR(err);
                    } else {
                        contact_points_size = ::cl::Buffer(
                                context, CL_MEM_READ_WRITE,
                                sizeof (cl_uint),
                                NULL, &err);
                        CHECK_CL_ERR(err);
                        err = queue.enqueueWriteBuffer(
                                contact_points_size, CL_FALSE,
                                0, sizeof (cl_uint),
                                &kernel_contact_points_size, NULL,
#ifdef USE_PROFILING
                                &contact_points_size_event);
#else // USE_PROFILING
                                NULL);
#endif // USE_PROFILING
                    }
                    CHECK_CL_ERR(err);
                    err = exact_tests_kernel.setArg(6, contact_points_size);
                    CHECK_CL_ERR(err);

                    local_work_size =
                        exact_tests_kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device_handle->device, &err);
                    CHECK_CL_ERR(err);
                    local_work_size = std::min(local_work_size, max_work_item_sizes[0]);

                    global_work_size = std::max(num_work_items_per_compute_unit * max_compute_units, local_work_size);
                    if(global_work_size % local_work_size > 0) {
                        global_work_size -= global_work_size % local_work_size;
                    }

                    // generate contact points!
#ifdef USE_PROFILING
                    {
                        ::cl::Event et_profiling;
                        err = queue.enqueueNDRangeKernel(
                                exact_tests_kernel,
                                ::cl::NullRange,
                                ::cl::NDRange(global_work_size),
                                ::cl::NDRange(local_work_size),
                                NULL,
                                &et_profiling);
                        CHECK_CL_ERR(err);
                        err = queue.finish();
                        CHECK_CL_ERR(err);
                        cl_ulong start = et_profiling.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                        CHECK_CL_ERR(err);
                        cl_ulong end = et_profiling.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                        CHECK_CL_ERR(err);
                        exact_tests_kernel_time += end - start;

                        if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
                            // on the CPU we use host memory, no transactions to profile
                            ::cl::Event write_events[] = { contact_points_size_event };
                            for (short i = 0; i < 1; ++i) {
                                start = write_events[i].getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                                CHECK_CL_ERR(err);
                                end = write_events[i].getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                                CHECK_CL_ERR(err);
                                exact_test_opencl_write_time += end - start;
                            }

                            short num_events;
                            ::cl::Event read_events[] = { exact_tests_size_event
#ifndef NDEBUG_DIKUCL
                                , error_codes_event, error_size_event
                            };
                            num_events = 3;
#else // NDEBUG_DIKUCL
                            };
                            num_events = 1;
#endif // NDEBUG_DIKUCL
                            for (short i = 0; i < num_events; ++i) {
                                start = read_events[i].getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                                CHECK_CL_ERR(err);
                                end = read_events[i].getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                                CHECK_CL_ERR(err);
                                tandem_traversal_opencl_read_time += end - start;
                            }
                        }
                    }
#else
                    err = queue.enqueueNDRangeKernel(
                            exact_tests_kernel,
                            ::cl::NullRange,
                            ::cl::NDRange(global_work_size),
                            ::cl::NDRange(local_work_size));
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
#endif // USE_PROFILING
                    ++exact_test_invocations;

                    // blocking read because we need the number on the host
                    if(device_type & CL_DEVICE_TYPE_CPU) {
                        cl_uint *mapped_kernel_contact_points_size = (cl_uint*) queue.enqueueMapBuffer(
                                contact_points_size, CL_TRUE,
                                CL_MAP_READ,
                                0, sizeof(cl_uint),
                                NULL, NULL, &err);
                        CHECK_CL_ERR(err);
                        FINISH_QUEUE(queue);
                        kernel_contact_points_size = *mapped_kernel_contact_points_size;
                        err = queue.enqueueUnmapMemObject(
                                contact_points_size,
                                (void*) mapped_kernel_contact_points_size);
                        CHECK_CL_ERR(err);
                    } else {
                        err = queue.enqueueReadBuffer(
                                contact_points_size, CL_TRUE,
                                0, sizeof (cl_uint),
                                &kernel_contact_points_size, NULL,
#ifdef USE_PROFILING
                                &contact_points_size_event);
                        cl_ulong start = contact_points_size_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                        CHECK_CL_ERR(err);
                        cl_ulong end = contact_points_size_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                        CHECK_CL_ERR(err);
                        exact_test_opencl_read_time += end - start;
#else // USE_PROFILING
                                NULL);
#endif // USE_PROFILING
                        CHECK_CL_ERR(err);
                    }
                    FINISH_QUEUE(queue);

                    // collect contact points if any
                    if (kernel_contact_points_size > 0) {
                        contact_point_container = (details::cl::KernelContactPoint<KV, KT, KI>*) realloc(
                                contact_point_container,
                                sizeof (details::cl::KernelContactPoint<KV, KT, KI>) *
                                    (contact_point_container_size + (size_t) kernel_contact_points_size));
                        if(device_type & CL_DEVICE_TYPE_CPU) {
                            details::cl::KernelContactPoint<KV, KT, KI> *mapped_kernel_contact_points =
                                    (details::cl::KernelContactPoint<KV, KT, KI>*) queue.enqueueMapBuffer(
                                    contact_points, CL_TRUE, CL_MAP_READ,
                                    0, sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * (size_t) kernel_contact_points_size,
                                    NULL, NULL, &err);
                            CHECK_CL_ERR(err);
                            std::memcpy(
                                    contact_point_container + (size_t) contact_point_container_size,
                                    mapped_kernel_contact_points,
                                    sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * (size_t) kernel_contact_points_size);
                            err = queue.enqueueUnmapMemObject(
                                    contact_points,
                                    mapped_kernel_contact_points);
                            CHECK_CL_ERR(err);
                        } else {
#ifdef USE_PROFILING
                            ::cl::Event contact_points_event;
#endif // USE_PROFILING
                            err = queue.enqueueReadBuffer(
                                    contact_points, CL_TRUE,
                                    0, sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * (size_t) kernel_contact_points_size,
                                    contact_point_container + (size_t) contact_point_container_size, NULL,
#ifdef USE_PROFILING
                                    &contact_points_event);
                            cl_ulong start = contact_points_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                            CHECK_CL_ERR(err);
                            cl_ulong end = contact_points_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                            CHECK_CL_ERR(err);
                            exact_test_opencl_read_time += end - start;
#else // USE_PROFILING
                                    NULL);
#endif // USE_PROFILING
                            CHECK_CL_ERR(err);
                        }
                        FINISH_QUEUE(queue);
                        contact_point_container_size += (size_t) kernel_contact_points_size;
                    }

                }

                // get leftover work amount
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    cl_uint *mapped_kernel_global_backlog_size = (cl_uint*) queue.enqueueMapBuffer(
                            global_backlog_size, CL_TRUE,
                            CL_MAP_READ,
                            0, sizeof(cl_uint),
                            NULL, NULL, &err);
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(queue);
                    kernel_global_backlog_size = *mapped_kernel_global_backlog_size;
                    err = queue.enqueueUnmapMemObject(
                            global_backlog_size,
                            (void*) mapped_kernel_global_backlog_size);
                    CHECK_CL_ERR(err);
                } else {
                    err = queue.enqueueReadBuffer(
                            global_backlog_size, CL_TRUE,
                            0, sizeof (cl_uint),
                            &kernel_global_backlog_size, NULL,
#ifdef USE_PROFILING
                            &global_backlog_size_event);
                    cl_ulong start = global_backlog_size_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                    CHECK_CL_ERR(err);
                    cl_ulong end = global_backlog_size_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                    CHECK_CL_ERR(err);
                    tandem_traversal_opencl_read_time += end - start;
#else // USE_PROFILING
                            NULL);
#endif // USE_PROFILING
                    CHECK_CL_ERR(err);
                }
                FINISH_QUEUE(queue);

                // swap work buffers for next iteration
                if (kernel_global_backlog_size > 0) {
                    err = tandem_traversal_kernel.setArg(1, work[1 - current_work_buffer]);
                    CHECK_CL_ERR(err);
                    err = tandem_traversal_kernel.setArg(6, work[current_work_buffer]);
                    CHECK_CL_ERR(err);
                    current_work_buffer = 1 - current_work_buffer;
                }
                kernel_work_size = (size_t) kernel_global_backlog_size;
            }

            kernel_work_item_generator.cleanup_generated_work_items(
                    kernel_work, device_type);
        }

        err = queue.finish();
        CHECK_CL_ERR(err);

        STOP_TIMER("tandem_traversal_opencl_host_time");

        // sort and report contact points
        START_TIMER("sort_and_report_contact_points");
        if(contact_point_container != NULL) {
            qsort(  contact_point_container,
                    contact_point_container_size,
                    sizeof(details::cl::KernelContactPoint<KV, KT, KI>),
                    details::cl::compare_kernel_contact_points<KV, KT, KI>);
            for(size_t i = 0; i < contact_point_container_size; ++i) {
                details::cl::KernelContactPoint<KV, KT, KI> contact_point = contact_point_container[i];
                V p = V::make((T) contact_point.p.s[0], (T) contact_point.p.s[1], (T) contact_point.p.s[2]);
                V n = V::make((T) contact_point.n.s[0], (T) contact_point.n.s[1], (T) contact_point.n.s[2]);
                T d = (T) contact_point.d;
                kernel_callbacks[(size_t) contact_point.tp]->operator()(p, n, d);
            }
            free(contact_point_container);
        }
        STOP_TIMER("sort_and_report_contact_points");

        details::cl::cleanup<V, K, T, KI, KT, KV>(
                kernel_nodes,
                kernel_tets,
                kernel_tsi,
                kernel_verts,
                kernel_callbacks,
                device_type);
        
        details::cl::record_kernel_times(tandem_traversal_kernel_time, exact_tests_kernel_time);
        details::cl::record_kernel_invocations(tandem_traversal_invocations, exact_test_invocations);
        details::cl::record_opencl_times(tandem_traversal_opencl_read_time, tandem_traversal_opencl_write_time,
                exact_test_opencl_read_time, exact_test_opencl_write_time);
    }

}// namespace kdop

// KDOP_CL_TANDEM_TRAVERSAL_H
#endif
