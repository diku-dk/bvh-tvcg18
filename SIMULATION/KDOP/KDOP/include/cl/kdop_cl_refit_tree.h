#ifndef KDOP_CL_REFIT_TREE_H
#define KDOP_CL_REFIT_TREE_H

#include <cassert>

#include <dikucl.hpp>
#include <dikucl_context_manager.hpp>
#include <dikucl_device_manager.hpp>
#include <dikucl_kernel_manager.hpp>
#include <dikucl_util.h>

#include <kdop_tree.h>

#include <mesh_array_t4mesh.h>
#include <mesh_array_vertex_attribute.h>

#include <util_log.h>
#include <util_profiling.h>

#include "kdop_cl_kernels_path.h"

#ifndef NDEBUG_DIKUCL
#define MAX_ERROR_CODES 1024
#else // NDEBUG_DIKUCL
#define MAX_ERROR_CODES 0
#endif // NDEBUG_DIKUCL

namespace kdop
{
    namespace details
    {
        
        namespace cl
        {
    
            /**
             * Starts the refitting BVH kernel, assuming all data is scheduled
             * to be copied to the device in queue. Does not finish the queue.
             * 
             * @param context_handle
             * @param device_handle
             * @param queue
             * @param tetrahedrons
             * @param tetrahedron_offsets
             * @param vertices
             * @param vertex_offsets
             * @param nodes
             * @param node_offsets
             * @param last_level_offsets
             * @param total_objects
             */
            template<typename V, size_t K, typename T>
            inline cl_ulong refit_tree(  ::dikucl::ContextHandle *context_handle
                                       , ::dikucl::DeviceHandle *device_handle
                                       , ::cl::CommandQueue *queue
                                       , ::cl::Buffer *tetrahedrons
                                       , ::cl::Buffer *tetrahedron_offsets
                                       , ::cl::Buffer *vertices
                                       , ::cl::Buffer *vertex_offsets
                                       , ::cl::Buffer *nodes /* out */
                                       , ::cl::Buffer *node_offsets
                                       , ::cl::Buffer *last_level_offsets
                                       , size_t total_objects
                                   )
            {
                START_TIMER("refit_tree_opencl_init");

                cl_int err = CL_SUCCESS;

                cl_device_type device_type = device_handle->device.getInfo<CL_DEVICE_TYPE>(&err);
                CHECK_CL_ERR(err);
                size_t global_mem_cacheline_size =
                        (size_t) device_handle->device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>(&err);
                
                // Get the BVH refitting kernel.
                ::dikucl::KernelInfo refit_tree_kernel_info(  details::cl::kernels_path
                                                          , "kdop_cl_refit_tree.cl"
                                                          , "do_refit_tree");
                refit_tree_kernel_info.include(details::cl::kernels_path).no_signed_zeros(true);
                refit_tree_kernel_info.define("__K", K);
                refit_tree_kernel_info.define("__INDEX_TYPE", "uint");
#ifndef NDEBUG_DIKUCL
                refit_tree_kernel_info.nv_verbose(true).debug_intel(true);
                refit_tree_kernel_info.define("__MAX_ERROR_CODES", MAX_ERROR_CODES);
#else // NDEBUG_DIKUCL
                refit_tree_kernel_info.define("__NDEBUG_DIKUCL", 1);
#endif // NDEBUG_DIKUCL
                ::dikucl::KernelHandle *refit_tree_kernel_handle =
                        ::dikucl::KernelManager::get_instance().get_kernel(  refit_tree_kernel_info
                                                                           , &err, context_handle);
                CHECK_CL_ERR(err);
                ::cl::Kernel refit_tree_kernel = refit_tree_kernel_handle->kernel;

                STOP_TIMER("refit_tree_opencl_init");
                START_TIMER("refit_tree_opencl_host_time");
                
                // Simply set all arguments.
                err = refit_tree_kernel.setArg(0, *vertices);
                CHECK_CL_ERR(err);
                err = refit_tree_kernel.setArg(1, *vertex_offsets);
                CHECK_CL_ERR(err);
                err = refit_tree_kernel.setArg(2, *tetrahedrons);
                CHECK_CL_ERR(err);
                err = refit_tree_kernel.setArg(3, *tetrahedron_offsets);
                CHECK_CL_ERR(err);
                err = refit_tree_kernel.setArg(4, *nodes);
                CHECK_CL_ERR(err);
                err = refit_tree_kernel.setArg(5, *node_offsets);
                CHECK_CL_ERR(err);
                err = refit_tree_kernel.setArg(6, *last_level_offsets);
                CHECK_CL_ERR(err);
                
                // Use system suggested local work size.
                const size_t rt_local_work_size =
                        refit_tree_kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(  device_handle->device
                                                                                      , &err);
                CHECK_CL_ERR(err);

#ifndef NDEBUG_DIKUCL
                cl_uint kernel_error_codes[MAX_ERROR_CODES];
                ::cl::Buffer error_codes;
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    size_t size = MAX_ERROR_CODES * sizeof(cl_uint);
                    size += size % global_mem_cacheline_size;
                    error_codes = ::cl::Buffer(
                            context_handle->context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                            size,
                            &kernel_error_codes, &err);
                    CHECK_CL_ERR(err);
                } else {
                    error_codes = ::cl::Buffer(
                            context_handle->context, CL_MEM_READ_WRITE,
                            MAX_ERROR_CODES * sizeof (cl_uint),
                            NULL, &err);
                    CHECK_CL_ERR(err);
                    err = queue->enqueueWriteBuffer(
                            error_codes, CL_FALSE,
                            0, MAX_ERROR_CODES * sizeof (cl_uint),
                            &kernel_error_codes, NULL, NULL);
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(*queue);
                }
                err = refit_tree_kernel.setArg(7, error_codes);
                CHECK_CL_ERR(err);

                cl_uint kernel_error_size = 0;
                ::cl::Buffer error_size;
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    size_t size = sizeof(cl_uint);
                    size += size % global_mem_cacheline_size;
                    error_size = ::cl::Buffer(
                            context_handle->context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                            size,
                            &kernel_error_size, &err);
                    CHECK_CL_ERR(err);
                } else {
                    error_size = ::cl::Buffer(
                            context_handle->context, CL_MEM_READ_WRITE,
                            sizeof (cl_uint),
                            NULL, &err);
                    CHECK_CL_ERR(err);
                    err = queue->enqueueWriteBuffer(
                            error_size, CL_FALSE,
                            0, sizeof (cl_uint),
                            &kernel_error_size, NULL, NULL);
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(*queue);
                }
                err = refit_tree_kernel.setArg(8, error_size);
                CHECK_CL_ERR(err);
#endif // NDEBUG_DIKUCL

                // Start the kernel.
                cl_ulong duration = 0;
#ifdef USE_PROFILING
                {
                    ::cl::Event rt_profiling;
                    err = queue->enqueueNDRangeKernel(  refit_tree_kernel
                                                     , ::cl::NullRange
                                                     , ::cl::NDRange(total_objects * rt_local_work_size)
                                                     , ::cl::NDRange(rt_local_work_size)
                                                     , NULL
                                                     , &rt_profiling
                                                     );
                    CHECK_CL_ERR(err);
                    err = queue->finish();
                    CHECK_CL_ERR(err);
                    cl_ulong start = rt_profiling.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                    CHECK_CL_ERR(err);
                    cl_ulong end = rt_profiling.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                    CHECK_CL_ERR(err);
                    duration = end - start;
                }
#else
                err = queue->enqueueNDRangeKernel(  refit_tree_kernel
                                                  , ::cl::NullRange
                                                  , ::cl::NDRange(total_objects * rt_local_work_size)
                                                  , ::cl::NDRange(rt_local_work_size)
                                                  );
                CHECK_CL_ERR(err);
#endif // USE_PROFILING

#ifndef NDEBUG_DIKUCL
                if(device_type & CL_DEVICE_TYPE_CPU) {
                    cl_uint *mapped_kernel_error_size = (cl_uint*) queue->enqueueMapBuffer(
                            error_size, CL_TRUE,
                            CL_MAP_READ,
                            0, sizeof(cl_uint),
                            NULL, NULL, &err);
                    CHECK_CL_ERR(err);
                    FINISH_QUEUE(*queue);
                    kernel_error_size = *mapped_kernel_error_size;
                    err = queue->enqueueUnmapMemObject(
                            error_size,
                            (void*) mapped_kernel_error_size);
                    CHECK_CL_ERR(err);
                } else {
                    err = queue->enqueueReadBuffer(
                            error_size, CL_TRUE,
                            0, sizeof (cl_uint),
                            &kernel_error_size, NULL, NULL);
                    CHECK_CL_ERR(err);
                }
                FINISH_QUEUE(*queue);

                if(kernel_error_size != 0) {
                    if(device_type & CL_DEVICE_TYPE_CPU) {
                        cl_uint *mapped_kernel_error_codes = (cl_uint*) queue->enqueueMapBuffer(
                                error_codes, CL_TRUE,
                                CL_MAP_READ,
                                0, kernel_error_size * sizeof(cl_uint),
                                NULL, NULL, &err);
                        CHECK_CL_ERR(err);
                        FINISH_QUEUE(*queue);
                        memcpy(kernel_error_codes, mapped_kernel_error_codes, (size_t) kernel_error_size * sizeof(cl_uint));
                        err = queue->enqueueUnmapMemObject(
                                error_codes,
                                (void*) mapped_kernel_error_codes);
                        CHECK_CL_ERR(err);
                    } else {
                        err = queue->enqueueReadBuffer(
                                error_codes, CL_TRUE,
                                0, (size_t) kernel_error_size * sizeof (cl_uint),
                                kernel_error_codes, NULL, NULL);
                        CHECK_CL_ERR(err);
                    }
                    FINISH_QUEUE(*queue);

                    {
                        util::Log logging;
                        logging << "Refit tree kernel generated the following "
                                << kernel_error_size
                                << " error codes:"
                                << util::Log::newline();
                        for(size_t i = 0; i < kernel_error_size; ++i) {
                            logging << kernel_error_codes[i] << util::Log::newline();
                        }
                    }
                    exit(1);
                }
#endif // NDEBUG_DIKUCL
                STOP_TIMER("refit_tree_opencl_host_time");
                return duration;
            }
            
        } // namespace cl
        
    } // namespace details
    
    template<typename V, size_t K, typename T>
    inline void refit_tree(  Tree<T,K> & tree
                             , mesh_array::T4Mesh const & mesh
                             , mesh_array::VertexAttribute<T,mesh_array::T4Mesh> const & X
                             , mesh_array::VertexAttribute<T,mesh_array::T4Mesh> const & Y
                             , mesh_array::VertexAttribute<T,mesh_array::T4Mesh> const & Z
                             , const dikucl & /* tag */
                             )
    {
        assert( 0 || !"refit_tree using DIKUCL is not yet implemented.");
        
        // TODO Batch refitting of trees, copy to device, call above routine
        // and copy back from device.
    }
    
} // namespace kdop

// KDOP_CL_REFIT_TREE_H
#endif
