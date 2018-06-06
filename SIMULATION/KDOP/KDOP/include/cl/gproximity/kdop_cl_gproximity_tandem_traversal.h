#ifndef KDOP_CL_GPROXIMITY_TANDEM_TRAVERSAL_H
#define	KDOP_CL_GPROXIMITY_TANDEM_TRAVERSAL_H

#include <cl/kdop_cl_kernels_path.h>
#include <cl/kdop_cl_tandem_traversal.h>

#include <dikucl.hpp>
#include <dikucl_command_queue_manager.hpp>
#include <dikucl_context_manager.hpp>
#include <dikucl_kernel_manager.hpp>
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
#include <limits>
#include <queue>
#include <stdlib.h>
#include <string>
#include <utility>

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

      template< typename KernelI >
      class gProximityKernelWorkItemGenerator
      : public KernelWorkItemGenerator<KernelI>
      {

      public:

        gProximityKernelWorkItemGenerator()
        {
        }

        void add_roots(std::vector<KernelI> a, std::vector<KernelI> b, KernelI tp, KernelI /* level */) {
          this->m_q.push(typename KernelWorkItemGenerator<KernelI>::RootPairs());
          this->m_q.back().a = std::vector<KernelI>(a);
          this->m_q.back().b = std::vector<KernelI>(b);
          this->m_q.back().tp = tp;
        }

        // 2016-06-22 Kenny code reivew:  'generate_kernel_work_items' hides
        // overloaded virtual function. Declared here: different number
        // of parameters (9 vs 7)
        size_t generate_kernel_work_items(
                                          size_t n,
                                          KernelWorkItem<KernelI> **out_kernel_work_items,
                                          size_t global_mem_cacheline_size,
                                          size_t /* max_bvtt_degree */,
                                          size_t /* max_bvtt_height */,
                                          size_t /* max_global_work_size */,
                                          size_t stride = 1
                                          )
        {
          if(this->m_q.empty()) {
            *out_kernel_work_items = NULL;
            return 0;
          }

          *out_kernel_work_items = new KernelWorkItem<KernelI>[n * stride];

          size_t actual = 0;
          while(actual < n && !(this->m_q.empty())) {
            for(; this->m_a < this->m_q.front().a.size() && actual < n; ++(this->m_a)) {
              for(; this->m_b < this->m_q.front().b.size() && actual < n; ++(this->m_b)) {
                (*out_kernel_work_items)[actual * stride].a = (KernelI) this->m_q.front().a[this->m_a];
                (*out_kernel_work_items)[actual * stride].b = (KernelI) this->m_q.front().b[this->m_b];
                (*out_kernel_work_items)[actual * stride].tp = (KernelI) this->m_q.front().tp;
                ++actual;
              }
              if(actual < n) {
                this->m_b = 0;
              } else {
                break;
              }
            }
            if(actual < n) {
              this->m_a = 0;
              this->m_q.pop();
            }
          }

          return actual;
        }



        // 2016-06-22 Kenny code reivew:  'cleanup_generated_work_items' hides
        // overloaded virtual function, declared here: different number of
        // parameters (2 vs 1)
        void cleanup_generated_work_items(
                                          KernelWorkItem<KernelI> * generated_kernel_work_items
                                          )
        {
          if(generated_kernel_work_items != NULL)
          {
            delete[] generated_kernel_work_items;
          }
        }

      };

      inline void record_kernel_times(  cl_ulong tandem_traversal_time
                                      , cl_ulong balance_work_time
                                      , cl_ulong exact_test_time)
      {
          RECORD_TIME("tandem_traversal", (double) tandem_traversal_time / 1000000.0);
          RECORD_TIME("balance_work", (double) balance_work_time / 1000000.0);
          RECORD_TIME("exact_test", (double) exact_test_time / 1000000.0);
      }
      
      inline void record_kernel_invocations(  size_t tandem_traversal_invocations
                                            , size_t balance_work_invocations
                                            , size_t exact_test_invocations)
      {
          RECORD("tandem_traversal_invocations", tandem_traversal_invocations);
          RECORD("balance_work_invocations", balance_work_invocations);
          RECORD("exact_test_invocations", exact_test_invocations);
      }

      inline void record_opencl_times(  cl_ulong tandem_traversal_read_time, cl_ulong tandem_traversal_write_time
                                      , cl_ulong balance_work_read_time, cl_ulong balance_work_write_time
                                      , cl_ulong exact_test_read_time, cl_ulong exact_test_write_time)
      {
          RECORD_TIME("tandem_traversal_opencl_read_time", (double) tandem_traversal_read_time / 1000000.0);
          RECORD_TIME("tandem_traversal_opencl_write_time", (double) tandem_traversal_write_time / 1000000.0);
          RECORD_TIME("balance_work_opencl_read_time", (double) balance_work_read_time / 1000000.0);
          RECORD_TIME("balance_work_opencl_write_time", (double) balance_work_write_time / 1000000.0);
          RECORD_TIME("exact_test_opencl_read_time", (double) exact_test_read_time / 1000000.0);
          RECORD_TIME("exact_test_opencl_write_time", (double) exact_test_write_time / 1000000.0);
      }

    } // namespace cl

  } // namespace details

  template< typename V, size_t K, typename T, typename test_pair_container >
  inline void tandem_traversal(test_pair_container test_pairs,
                               dikucl::gproximity const & /* tag */,
                               size_t open_cl_platform = 0,
                               size_t open_cl_device = 0) {
    // assert( ! test_pairs.empty() || !"tandem_traversal : test_pairs are empty" );
    if ( test_pairs.empty() ) {
      details::cl::record_kernel_times(0, 0, 0);
      details::cl::record_kernel_invocations(0, 0, 0);
      details::cl::record_opencl_times(0, 0, 0, 0, 0, 0);
      RECORD_TIME("tandem_traversal_opencl_prepare_input", (double) 0.0);
      RECORD_TIME("sort_and_report_contact_points", (double) 0.0);
      return;
    }

    cl_ulong tandem_traversal_kernel_time = 0;
    cl_ulong balance_work_kernel_time     = 0;
    cl_ulong exact_tests_kernel_time      = 0;

    cl_ulong tandem_traversal_opencl_write_time = 0;
    cl_ulong tandem_traversal_opencl_read_time  = 0;
    cl_ulong balance_work_opencl_write_time     = 0;
    cl_ulong balance_work_opencl_read_time      = 0;
    cl_ulong exact_test_opencl_write_time       = 0;
    cl_ulong exact_test_opencl_read_time        = 0;

    size_t tandem_traversal_invocations = 0;
    size_t balance_work_invocations     = 0;
    size_t exact_test_invocations       = 0;

    cl_int err = CL_SUCCESS;

    ::dikucl::PlatformHandle *platform_handle = ::dikucl::PlatformManager::get_instance().get_platform(
                                                                                                       &err, open_cl_platform);
    CHECK_CL_ERR(err);

    ::dikucl::DeviceHandle *device_handle = ::dikucl::DeviceManager::get_instance().get_device(
                                                                                               &err, platform_handle, open_cl_device);
    CHECK_CL_ERR(err);
    size_t global_mem_cacheline_size =
    (size_t) device_handle->device.getInfo<CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE>(&err);
    CHECK_CL_ERR(err);

    cl_device_type device_type = device_handle->device.getInfo<CL_DEVICE_TYPE>(&err);
    CHECK_CL_ERR(err);

    cl_uint max_compute_units = device_handle->device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>(&err);
    CHECK_CL_ERR(err);

    size_t global_mem_size = (size_t) device_handle->device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>(&err);
    CHECK_CL_ERR(err);

    // easy on the memory allocation, if the maximum allocation size is more than 25% of the total available memory,
    // we are getting into trouble further down the pipeline, because we allocate 3x this maximum allocation size
    size_t max_mem_alloc_size = std::min((size_t) device_handle->device.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>(&err), global_mem_size / 4);
    CHECK_CL_ERR(err);

    ::dikucl::ContextHandle *context_handle = ::dikucl::ContextManager::get_instance().get_context(&err, device_handle);
    CHECK_CL_ERR(err);
    ::cl::Context context = context_handle->context;

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

    // figure out whether we're on a NVIDIA GPU
    std::string device_extensions = device_handle->device.getInfo<CL_DEVICE_EXTENSIONS>(&err);
    CHECK_CL_ERR(err);
    if(device_extensions.find("cl_nv_device_attribute_query") != std::string::npos)
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
    size_t max_global_work_size = std::max(num_work_items_per_compute_unit * max_compute_units, max_work_item_sizes[0]);

    // check how much local memory we have available
    const size_t available_local_memory = (size_t) device_handle->device.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>(&err);
    CHECK_CL_ERR(err);

    typedef cl_uint KI; const std::string kernel_index_type = "uint";
    typedef cl_float KT; const std::string kernel_real_type = "float";
    typedef cl_float3 KV; const std::string kernel_vector3_type = "float3";

    details::cl::KernelNode<KI, KT, K> *kernel_nodes;
    details::cl::KernelTetrahedron<KI> *kernel_tets;
    details::cl::KernelTetrahedronSurfaceInfo *kernel_tsi;
    details::cl::KernelWorkItem<KI> *kernel_work;
    geometry::ContactsCallback<V> **kernel_callbacks;
    KV *kernel_verts;
    size_t kernel_nodes_size, kernel_tets_size, kernel_verts_size;
    size_t max_bvtt_degree, max_bvtt_height;

    details::cl::gProximityKernelWorkItemGenerator<KI> kernel_work_item_generator;
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
                                                    (size_t) std::numeric_limits<size_t>::max());
    STOP_TIMER("tandem_traversal_opencl_prepare_input");

    /* Get Tandem Traversal Kernel and set static parameters */

    ::dikucl::KernelInfo tandem_traversal_kernel_info(
                                                      details::cl::kernels_path + "gproximity/",
                                                      "kdop_cl_gproximity_tandem_traversal.cl",
                                                      "do_tandem_traversal");
    tandem_traversal_kernel_info.define("__K", K);
    tandem_traversal_kernel_info.define("__INDEX_TYPE", kernel_index_type).define("__REAL_TYPE", kernel_real_type);
    tandem_traversal_kernel_info.define("__MAX_BVTT_DEGREE", max_bvtt_degree);
    tandem_traversal_kernel_info.define("__IDLE_THRESHOLD", 0.67f);

    tandem_traversal_kernel_info.include(details::cl::kernels_path).no_signed_zeros(true);
    tandem_traversal_kernel_info.include(details::cl::kernels_path + "gproximity/");
#ifndef NDEBUG_DIKUCL
    tandem_traversal_kernel_info.nv_verbose(true);
    tandem_traversal_kernel_info.define("__MAX_ERROR_CODES", MAX_ERROR_CODES);
#else
    tandem_traversal_kernel_info.define("__NDEBUG_DIKUCL", 1);
#endif // NDEBUG
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

    ::cl::Buffer nodes = ::cl::Buffer(
                                      context, CL_MEM_READ_ONLY,
                                      sizeof (details::cl::KernelNode<KI, KT, K>) * kernel_nodes_size,
                                      NULL, &err);
    CHECK_CL_ERR(err);
#ifdef USE_PROFILING
    ::cl::Event nodes_event;
#endif // USE_PROFILING
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

    err = tandem_traversal_kernel.setArg(0, nodes);
    CHECK_CL_ERR(err);

    size_t global_exact_tests_capacity = max_mem_alloc_size / sizeof(details::cl::KernelWorkItem<KI>);
    ::cl::Buffer exact_tests = ::cl::Buffer(
                                            context, CL_MEM_READ_WRITE,
                                            global_exact_tests_capacity * sizeof(details::cl::KernelWorkItem<KI>),
                                            NULL, &err);
    CHECK_CL_ERR(err);
    err = tandem_traversal_kernel.setArg(4, exact_tests);
    CHECK_CL_ERR(err);

    // find out how much memory the kernel takes up to determine the remaining amount of local memory available to it
    size_t tandem_traversal_local_memory =
            available_local_memory - (size_t) tandem_traversal_kernel.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(device_handle->device, &err);
    CHECK_CL_ERR(err);

    // determine the number of work items per local work queue per work group
    const size_t local_work_queue_capacity = tandem_traversal_local_memory / (sizeof(details::cl::KernelWorkItem<KI>) * max_num_work_groups_per_compute_unit);

    // allocate the local memory
    err = tandem_traversal_kernel.setArg(6, sizeof(details::cl::KernelWorkItem<KI>) * local_work_queue_capacity, NULL);
    CHECK_CL_ERR(err);
    err = tandem_traversal_kernel.setArg(7, (cl_int) local_work_queue_capacity);
    CHECK_CL_ERR(err);

    // get suggested local work size from the kernel
    size_t tandem_traversal_local_work_size = tandem_traversal_kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device_handle->device, &err);
    CHECK_CL_ERR(err);

    // however, we do not want more work items than space in the queue
    tandem_traversal_local_work_size = std::min(tandem_traversal_local_work_size, local_work_queue_capacity);

    // now make sure the local work size is a preferred multiple
    const size_t preferred_work_group_size_multiple =
        tandem_traversal_kernel.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device_handle->device, &err);
    CHECK_CL_ERR(err);
    tandem_traversal_local_work_size -= tandem_traversal_local_work_size % preferred_work_group_size_multiple;

    // compute global work size for the kernel
    size_t tandem_traversal_global_work_size = std::max(num_work_items_per_compute_unit * max_compute_units, tandem_traversal_local_work_size);
    tandem_traversal_global_work_size -= tandem_traversal_global_work_size % tandem_traversal_local_work_size;

    // compute global work queue capacity for each work group
    size_t tandem_traversal_work_groups = tandem_traversal_global_work_size / tandem_traversal_local_work_size;
    size_t global_work_queue_capacity = max_mem_alloc_size / (sizeof(details::cl::KernelWorkItem<KI>) * tandem_traversal_work_groups);
    err = tandem_traversal_kernel.setArg(8, (cl_uint) global_work_queue_capacity);
    CHECK_CL_ERR(err);

    // set limit for global exact tests
    err = tandem_traversal_kernel.setArg(9, (cl_uint) global_exact_tests_capacity);
    CHECK_CL_ERR(err);

    /* Get Exact Tests Kernel and set static parameters */

    ::dikucl::KernelInfo exact_tests_kernel_info(
                                                 details::cl::kernels_path,
                                                 "kdop_cl_exact_tests.cl",
                                                 "do_exact_tests");
    exact_tests_kernel_info.define("__INDEX_TYPE", kernel_index_type).define("__REAL_TYPE", kernel_real_type);
    exact_tests_kernel_info.define("__VECTOR3_TYPE", kernel_vector3_type);

    exact_tests_kernel_info.include(details::cl::kernels_path).no_signed_zeros(true);
#ifndef NDEBUG_DIKUCL
    exact_tests_kernel_info.nv_verbose(true);
    exact_tests_kernel_info.define("__MAX_ERROR_CODES", MAX_ERROR_CODES);
#else
    exact_tests_kernel_info.define("__NDEBUG_DIKUCL", 1);
#endif // NDEBUG
    ::dikucl::KernelHandle *exact_tests_kernel_handle = ::dikucl::KernelManager::get_instance().get_kernel(
                                                                                                           exact_tests_kernel_info,
                                                                                                           &err,
                                                                                                           context_handle);
    CHECK_CL_ERR(err);
    ::cl::Kernel exact_tests_kernel = exact_tests_kernel_handle->kernel;

    ::cl::Buffer tetrahedrons = ::cl::Buffer(
                                             context, CL_MEM_READ_ONLY,
                                             sizeof (details::cl::KernelTetrahedron<KI>) * kernel_tets_size,
                                             NULL, &err);
    CHECK_CL_ERR(err);

#ifdef USE_PROFILING
    ::cl::Event tetrahedrons_event;
#endif // USE_PROFILING
    err = queue.enqueueWriteBuffer(
                                   tetrahedrons, CL_FALSE,
                                   0, sizeof (details::cl::KernelTetrahedron<KI>) * kernel_tets_size,
                                   kernel_tets, NULL,
#ifdef USE_PROFILING
                                   &tetrahedrons_event);
#else // USE_PROFILING
                                   NULL);
#endif // USE_PROFILING
    CHECK_CL_ERR(err);
    FINISH_QUEUE(queue);

    err = exact_tests_kernel.setArg(0, tetrahedrons);
    CHECK_CL_ERR(err);

    ::cl::Buffer tetrahedron_surface_info = ::cl::Buffer(
                                                         context, CL_MEM_READ_ONLY,
                                                         sizeof (details::cl::KernelTetrahedronSurfaceInfo) * kernel_tets_size,
                                                         NULL, &err);
    CHECK_CL_ERR(err);

#ifdef USE_PROFILING
    ::cl::Event tetrahedron_surface_info_event;
#endif // USE_PROFILING
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

    err = exact_tests_kernel.setArg(1, tetrahedron_surface_info);
    CHECK_CL_ERR(err);

    ::cl::Buffer vertices = ::cl::Buffer(
                                         context, CL_MEM_READ_ONLY,
                                         sizeof (KV) * kernel_verts_size,
                                         NULL, &err);
    CHECK_CL_ERR(err);

#ifdef USE_PROFILING
    ::cl::Event vertices_event;
#endif // USE_PROFILING
    err = queue.enqueueWriteBuffer(vertices, CL_FALSE, 0, sizeof (KV) * kernel_verts_size,
                                   kernel_verts, NULL,
#ifdef USE_PROFILING
                                   &vertices_event);
#else // USE_PROFILING
                                   NULL);
#endif // USE_PROFILING
    CHECK_CL_ERR(err);
    FINISH_QUEUE(queue);

    err = exact_tests_kernel.setArg(2, vertices);
    CHECK_CL_ERR(err);

    // suggested local work size again
    size_t exact_tests_local_work_size = exact_tests_kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device_handle->device, &err);
    CHECK_CL_ERR(err);

    // compute global work size based on that
    size_t exact_tests_global_work_size = std::max(num_work_items_per_compute_unit * max_compute_units, exact_tests_local_work_size);
    if(exact_tests_global_work_size % exact_tests_local_work_size > 0) {
        exact_tests_global_work_size -= exact_tests_global_work_size % exact_tests_local_work_size;
    }

    /* Get Balance Work Kernel and set static parameters */

    ::dikucl::KernelInfo balance_work_kernel_info(
                                                  details::cl::kernels_path + "gproximity/",
                                                  "kdop_cl_gproximity_balance_work.cl",
                                                  "do_balance_work");
    balance_work_kernel_info.define("__INDEX_TYPE", kernel_index_type);
    balance_work_kernel_info.define("__BALANCE_THREADS", max_work_item_sizes[0]);
    balance_work_kernel_info.define("__GLOBAL_WORK_QUEUE_CAPACITY", global_work_queue_capacity);
    balance_work_kernel_info.define("__GLOBAL_WORK_QUEUES", tandem_traversal_work_groups);
    balance_work_kernel_info.define("__LOCAL_WORK_QUEUE_INIT_ITEMS", tandem_traversal_local_work_size * 3);
    balance_work_kernel_info.define("__LOCAL_WORK_QUEUE_CAPACITY", local_work_queue_capacity);
    balance_work_kernel_info.define("__TRAVERSAL_THREADS", tandem_traversal_local_work_size);

    balance_work_kernel_info.include(details::cl::kernels_path).no_signed_zeros(true);
    balance_work_kernel_info.include(details::cl::kernels_path + "gproximity/");
#ifndef NDEBUG_DIKUCL
    balance_work_kernel_info.nv_verbose(true);
    balance_work_kernel_info.define("__MAX_ERROR_CODES", MAX_ERROR_CODES);
#else
    balance_work_kernel_info.define("__NDEBUG_DIKUCL", 1);
#endif // NDEBUG
    ::dikucl::KernelHandle *balance_work_kernel_handle = ::dikucl::KernelManager::get_instance().get_kernel(
                                                                                                            balance_work_kernel_info,
                                                                                                            &err,
                                                                                                            context_handle);
    CHECK_CL_ERR(err);
    ::cl::Kernel balance_work_kernel = balance_work_kernel_handle->kernel;

    details::cl::KernelContactPoint<KV, KT, KI> *contact_point_container = NULL;
    size_t contact_point_container_size = 0;

    ::cl::Buffer active_splits = ::cl::Buffer(
                                              context, CL_MEM_READ_WRITE,
                                              sizeof(cl_int),
                                              NULL, &err);
    CHECK_CL_ERR(err);
    err = balance_work_kernel.setArg(3, active_splits);
    CHECK_CL_ERR(err);

    ::cl::Buffer balance_signal = ::cl::Buffer(
                                               context, CL_MEM_READ_WRITE,
                                               sizeof(cl_int),
                                               NULL, &err);
    CHECK_CL_ERR(err);
    err = balance_work_kernel.setArg(4, balance_signal);
    CHECK_CL_ERR(err);

#ifndef NDEBUG_DIKUCL
    // set error handling parameters for tandem traversal kernel
    err = tandem_traversal_kernel.setArg(10, (cl_uint) kernel_nodes_size);
    CHECK_CL_ERR(err);
    err = tandem_traversal_kernel.setArg(11, (cl_uint) (tandem_traversal_work_groups * global_work_queue_capacity));
    CHECK_CL_ERR(err);

    cl_uint kernel_error_codes[MAX_ERROR_CODES];
    ::cl::Buffer error_codes;
    error_codes = ::cl::Buffer(
            context, CL_MEM_READ_WRITE,
            MAX_ERROR_CODES * sizeof (cl_uint),
            NULL, &err);
    CHECK_CL_ERR(err);
    err = queue.enqueueWriteBuffer(
            error_codes, CL_FALSE,
            0, MAX_ERROR_CODES * sizeof (cl_uint),
            &kernel_error_codes, NULL,
            NULL);
    CHECK_CL_ERR(err);
    FINISH_QUEUE(queue);
    err = tandem_traversal_kernel.setArg(12, error_codes);
    CHECK_CL_ERR(err);

    cl_uint kernel_error_size = 0;
    ::cl::Buffer error_size;

    error_size = ::cl::Buffer(
            context, CL_MEM_READ_WRITE,
            sizeof (cl_uint),
            NULL, &err);
    CHECK_CL_ERR(err);
    err = queue.enqueueWriteBuffer(
            error_size, CL_FALSE,
            0, sizeof (cl_uint),
            &kernel_error_size, NULL,
            NULL);
    CHECK_CL_ERR(err);
    FINISH_QUEUE(queue);
    err = tandem_traversal_kernel.setArg(13, error_size);
    CHECK_CL_ERR(err);

    // repeat for balance work kernel
    err = balance_work_kernel.setArg(5, (cl_uint) (tandem_traversal_work_groups * global_work_queue_capacity));
    CHECK_CL_ERR(err);
    err = balance_work_kernel.setArg(6, (cl_uint) (tandem_traversal_work_groups * global_work_queue_capacity));
    CHECK_CL_ERR(err);
    err = balance_work_kernel.setArg(7, (cl_uint) tandem_traversal_work_groups);
    CHECK_CL_ERR(err);

    err = balance_work_kernel.setArg(8, error_codes);
    CHECK_CL_ERR(err);
    err = balance_work_kernel.setArg(9, error_size);
    CHECK_CL_ERR(err);
#endif // NDEBUG_DIKUCL

    while(!kernel_work_item_generator.empty()) {
#ifdef USE_PROFILING
      // similar to record_per_simulation_step_times (see above)
      bool record_per_work_item_generation_times = true;
#endif

      size_t kernel_root_pairs_size = kernel_work_item_generator.generate_kernel_work_items(
                                                                                            tandem_traversal_work_groups, &kernel_work,
                                                                                            global_mem_cacheline_size,
                                                                                            max_bvtt_degree, max_bvtt_height,
                                                                                            tandem_traversal_work_groups,
                                                                                            global_work_queue_capacity);

      if(kernel_root_pairs_size > 0) {
        ::cl::Buffer work[2];
        work[0] = ::cl::Buffer(
                               context, CL_MEM_READ_WRITE,
                               tandem_traversal_work_groups * global_work_queue_capacity * sizeof(details::cl::KernelWorkItem<KI>),
                               NULL, &err);
        CHECK_CL_ERR(err);

#ifdef USE_PROFILING
        ::cl::Event work_event;
#endif
        err = queue.enqueueWriteBuffer(
                                       work[0], CL_FALSE,
                                       0, tandem_traversal_work_groups * global_work_queue_capacity * sizeof(details::cl::KernelWorkItem<KI>),
                                       kernel_work, NULL,
#ifdef USE_PROFILING
                                       &work_event);
#else // USE_PROFILING
                                       NULL);
#endif // USE_PROFILING
        CHECK_CL_ERR(err);
        FINISH_QUEUE(queue);

        work[1] = ::cl::Buffer(
                               context, CL_MEM_READ_WRITE,
                               tandem_traversal_work_groups * global_work_queue_capacity * sizeof(details::cl::KernelWorkItem<KI>),
                               NULL, &err);
        CHECK_CL_ERR(err);

        cl_uint *kernel_work_counts = new cl_uint[tandem_traversal_work_groups];
        memset(kernel_work_counts, 0, tandem_traversal_work_groups * sizeof(cl_uint));
        assert (kernel_root_pairs_size <= tandem_traversal_work_groups || !"More root pairs than work groups");
        for(size_t i = 0; i < kernel_root_pairs_size; ++i) {
          kernel_work_counts[i] = 1;
        }
        ::cl::Buffer work_counts = ::cl::Buffer(
                                                context, CL_MEM_READ_WRITE,
                                                tandem_traversal_work_groups * sizeof(cl_uint),
                                                NULL, &err);
        CHECK_CL_ERR(err);

#ifdef USE_PROFILING
        ::cl::Event work_counts_event;
#endif // USE_PROFILING
        err = queue.enqueueWriteBuffer(
                                       work_counts, CL_FALSE,
                                       0, tandem_traversal_work_groups * sizeof(cl_uint),
                                       kernel_work_counts, NULL,
#ifdef USE_PROFILING
                                       &work_counts_event);
#else // USE_PROFILING
                                       NULL);
#endif // USE_PROFILING
        CHECK_CL_ERR(err);
        FINISH_QUEUE(queue);

        err = tandem_traversal_kernel.setArg(2, work_counts);
        CHECK_CL_ERR(err);

        cl_uint kernel_exact_tests_size = 0;
        ::cl::Buffer exact_tests_size = ::cl::Buffer(
                                                     context, CL_MEM_READ_WRITE,
                                                     sizeof (cl_uint),
                                                     NULL, &err);
        CHECK_CL_ERR(err);

#ifdef USE_PROFILING
        ::cl::Event exact_tests_size_event;
#endif // USE_PROFILING
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

        err = tandem_traversal_kernel.setArg(5, exact_tests_size);
        CHECK_CL_ERR(err);

        cl_int kernel_active_splits = 1;
        bool global_work_queue_overflow = false;
        size_t current_work_buffer = 0;
        while (kernel_active_splits > 0 && !global_work_queue_overflow) {
          err = tandem_traversal_kernel.setArg(1, work[current_work_buffer]);
          CHECK_CL_ERR(err);

          cl_uint kernel_idle_count = 0;
          ::cl::Buffer idle_count = ::cl::Buffer(
                                                 context, CL_MEM_READ_WRITE,
                                                 sizeof(cl_uint),
                                                 NULL, &err);
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          ::cl::Event idle_count_event;
#endif // USE_PROFILING
          err = queue.enqueueWriteBuffer(
                                         idle_count, CL_FALSE,
                                         0, sizeof(cl_uint),
                                         &kernel_idle_count, NULL,
#ifdef USE_PROFILING
                                         &idle_count_event);
#else // USE_PROFILING
                                         NULL);
#endif // USE_PROFILING
          CHECK_CL_ERR(err);
          FINISH_QUEUE(queue);

          err = tandem_traversal_kernel.setArg(3, idle_count);
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          {
            ::cl::Event tt_profiling;
            err = queue.enqueueNDRangeKernel(
                                             tandem_traversal_kernel,
                                             ::cl::NullRange,
                                             ::cl::NDRange(tandem_traversal_global_work_size),
                                             ::cl::NDRange(tandem_traversal_local_work_size),
                                             NULL,
                                             &tt_profiling);
            CHECK_CL_ERR(err);
            err = queue.finish();
            CHECK_CL_ERR(err);
            cl_ulong start =
            tt_profiling.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
            CHECK_CL_ERR(err);
            cl_ulong end =
            tt_profiling.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
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
                  ::cl::Event events[] = { work_event, work_counts_event, exact_tests_size_event };
                  for (short i = 0; i < 3; ++i) {
                      start = events[i].getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                      CHECK_CL_ERR(err);
                      end = events[i].getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                      CHECK_CL_ERR(err);
                      tandem_traversal_opencl_write_time += end - start;
                  }

                  // don't record this time again in future loop iterations
                  record_per_work_item_generation_times = false;
              }

              // these are events that happen once every kernel invocation
              start = idle_count_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
              CHECK_CL_ERR(err);
              end = idle_count_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
              CHECK_CL_ERR(err);
              tandem_traversal_opencl_write_time += end - start;
            }
          }
#else
          err = queue.enqueueNDRangeKernel(
                                           tandem_traversal_kernel,
                                           ::cl::NullRange,
                                           ::cl::NDRange(tandem_traversal_global_work_size),
                                           ::cl::NDRange(tandem_traversal_local_work_size));
          CHECK_CL_ERR(err);
          FINISH_QUEUE(queue);
#endif // USE_PROFILING

#ifndef NDEBUG_DIKUCL
          err = queue.enqueueReadBuffer(
              error_size, CL_TRUE,
              0, sizeof (cl_uint),
              &kernel_error_size, NULL,
              NULL);
          CHECK_CL_ERR(err);

          if(kernel_error_size != 0) {
              err = queue.enqueueReadBuffer(
                  error_codes, CL_TRUE,
                  0, (size_t) kernel_error_size * sizeof (cl_uint),
                  kernel_error_codes, NULL,
                  NULL);
              CHECK_CL_ERR(err);

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

          err = queue.enqueueReadBuffer(
                                        work_counts, CL_TRUE,
                                        0, tandem_traversal_work_groups * sizeof(cl_uint),
                                        kernel_work_counts, NULL,
#ifdef USE_PROFILING
                                        &work_counts_event);
#else // USE_PROFILING
                                        NULL);
#endif // USE_PROFILING
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          {
            if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
              // on the CPU we use host memory, no transactions to profile
              cl_ulong start = work_counts_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
              CHECK_CL_ERR(err);
              cl_ulong end = work_counts_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
              CHECK_CL_ERR(err);
              tandem_traversal_opencl_read_time += end - start;
            }
          }
#endif // USE_PROFILING

          for(size_t i = 0; i < tandem_traversal_work_groups; ++i) {
            if(kernel_work_counts[i] >= global_work_queue_capacity)
            {
              util::Log logging;

              global_work_queue_overflow = true;

              logging << "Work Queue "
              << i
              << " did overflow ("
              << kernel_work_counts[i]
              << ")"
              << util::Log::newline();

              logging.flush();
            }
          }

          err = balance_work_kernel.setArg(0, work[current_work_buffer]);
          CHECK_CL_ERR(err);
          err = balance_work_kernel.setArg(1, work[1 - current_work_buffer]);
          CHECK_CL_ERR(err);
          err = balance_work_kernel.setArg(2, work_counts);
          CHECK_CL_ERR(err);
#ifdef USE_PROFILING
          {
            ::cl::Event bw_profiling;
            err = queue.enqueueNDRangeKernel(
                                             balance_work_kernel,
                                             ::cl::NullRange,
                                             ::cl::NDRange(max_work_item_sizes[0]),
                                             ::cl::NDRange(max_work_item_sizes[0]),
                                             NULL,
                                             &bw_profiling);
            CHECK_CL_ERR(err);
            err = queue.finish();
            CHECK_CL_ERR(err);
            cl_ulong bw_start =
            bw_profiling.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
            CHECK_CL_ERR(err);
            cl_ulong bw_end =
            bw_profiling.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
            CHECK_CL_ERR(err);
            balance_work_kernel_time += bw_end - bw_start;
          }
#else
          err = queue.enqueueNDRangeKernel(
                                           balance_work_kernel,
                                           ::cl::NullRange,
                                           ::cl::NDRange(max_work_item_sizes[0]),
                                           ::cl::NDRange(max_work_item_sizes[0]));
          CHECK_CL_ERR(err);
#endif // USE_PROFILING

#ifndef NDEBUG_DIKUCL
          err = queue.enqueueReadBuffer(
              error_size, CL_TRUE,
              0, sizeof (cl_uint),
              &kernel_error_size, NULL,
              NULL);
          CHECK_CL_ERR(err);

          if(kernel_error_size != 0) {
              err = queue.enqueueReadBuffer(
                  error_codes, CL_TRUE,
                  0, (size_t) kernel_error_size * sizeof (cl_uint),
                  kernel_error_codes, NULL,
                  NULL);
              CHECK_CL_ERR(err);

            {
              util::Log logging;

              logging << "Balance work kernel generated the following "
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
          ++balance_work_invocations;

#ifdef USE_PROFILING
          ::cl::Event active_splits_event;
#endif // USE_PROFILING
          err = queue.enqueueReadBuffer(
                                        active_splits, CL_FALSE,
                                        0, sizeof (cl_int),
                                        &kernel_active_splits, NULL,
#ifdef USE_PROFILING
                                        &active_splits_event);
#else // USE_PROFILING
                                        NULL);
#endif // USE_PROFILING
          CHECK_CL_ERR(err);
          FINISH_QUEUE(queue);

          cl_int kernel_balance_signal = 0;
#ifdef USE_PROFILING
          ::cl::Event balance_signal_event;
#endif // USE_PROFILING
          err = queue.enqueueReadBuffer(
                                        balance_signal, CL_TRUE,
                                        0, sizeof (cl_int),
                                        &kernel_balance_signal, NULL,
#ifdef USE_PROFILING
                                        &balance_signal_event);
#else // USE_PROFILING
                                        NULL);
#endif // USE_PROFILING
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          {
            if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
              // on the CPU we use host memory, no transactions to profile
              ::cl::Event events[] = { active_splits_event, balance_signal_event };
              for (short i = 0; i < 2; ++i) {
                cl_ulong start = events[i].getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
                CHECK_CL_ERR(err);
                cl_ulong end = events[i].getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
                CHECK_CL_ERR(err);
                balance_work_opencl_read_time += end - start;
              }
            }
          }
#endif // USE_PROFILING

          if(kernel_balance_signal == 1) {
            current_work_buffer = 1 - current_work_buffer;
          }
        }

        delete[] kernel_work_counts;

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

#ifdef USE_PROFILING
        {
          if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
            // on the CPU we use host memory, no transactions to profile
            cl_ulong start = exact_tests_size_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
            CHECK_CL_ERR(err);
            cl_ulong end = exact_tests_size_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
            CHECK_CL_ERR(err);
            tandem_traversal_opencl_read_time += end - start;
          }
        }
#endif // USE_PROFILING

        if (kernel_exact_tests_size > 0) {
          err = exact_tests_kernel.setArg(3, exact_tests);
          CHECK_CL_ERR(err);
          err = exact_tests_kernel.setArg(4, kernel_exact_tests_size);
          CHECK_CL_ERR(err);

          ::cl::Buffer contact_points = ::cl::Buffer(
                                                     context, CL_MEM_WRITE_ONLY,
                                                     sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * kernel_exact_tests_size * 20,
                                                     NULL, &err);
          CHECK_CL_ERR(err);
          err = exact_tests_kernel.setArg(5, contact_points);
          CHECK_CL_ERR(err);

          cl_uint kernel_contact_points_size = 0;
          ::cl::Buffer contact_points_size = ::cl::Buffer(
                                                          context, CL_MEM_READ_WRITE,
                                                          sizeof (cl_uint),
                                                          NULL, &err);
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          ::cl::Event contact_points_size_event;
#endif // USE_PROFILING
          err = queue.enqueueWriteBuffer(
                                         contact_points_size, CL_FALSE,
                                         0, sizeof (cl_uint),
                                         &kernel_contact_points_size, NULL,
#ifdef USE_PROFILING
                                         &contact_points_size_event);
#else // USE_PROFILING
                                         NULL);
#endif // USE_PROFILING
          CHECK_CL_ERR(err);
          FINISH_QUEUE(queue);

          err = exact_tests_kernel.setArg(6, contact_points_size);
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          {
            ::cl::Event et_profiling;
            err = queue.enqueueNDRangeKernel(
                                             exact_tests_kernel,
                                             ::cl::NullRange,
                                             ::cl::NDRange(exact_tests_global_work_size),
                                             ::cl::NDRange(exact_tests_local_work_size),
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
              start = contact_points_size_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
              CHECK_CL_ERR(err);
              end = contact_points_size_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
              CHECK_CL_ERR(err);
              exact_test_opencl_write_time += end - start;
            }
          }
#else
          err = queue.enqueueNDRangeKernel(
                                           exact_tests_kernel,
                                           ::cl::NullRange,
                                           ::cl::NDRange(exact_tests_global_work_size),
                                           ::cl::NDRange(exact_tests_local_work_size));
          CHECK_CL_ERR(err);
          FINISH_QUEUE(queue);
#endif // USE_PROFILING
          ++exact_test_invocations;

          err = queue.enqueueReadBuffer(
                                        contact_points_size, CL_TRUE,
                                        0, sizeof (cl_uint),
                                        &kernel_contact_points_size, NULL,
#ifdef USE_PROFILING
                                        &contact_points_size_event);
#else // USE_PROFILING
                                        NULL);
#endif // USE_PROFILING
          CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          {
            if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
              // on the CPU we use host memory, no transactions to profile
              cl_ulong start = contact_points_size_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
              CHECK_CL_ERR(err);
              cl_ulong end = contact_points_size_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
              CHECK_CL_ERR(err);
              exact_test_opencl_read_time += end - start;
            }
          }
#endif // USE_PROFILING

          if (kernel_contact_points_size > 0) {
            contact_point_container = (details::cl::KernelContactPoint<KV, KT, KI>*) realloc(
                                                                                             contact_point_container,
                                                                                             sizeof (details::cl::KernelContactPoint<KV, KT, KI>) *
                                                                                             (contact_point_container_size + (size_t) kernel_contact_points_size));

#ifdef USE_PROFILING
            ::cl::Event contact_points_event;
#endif // USE_PROFILING
            err = queue.enqueueReadBuffer(
                                          contact_points, CL_TRUE,
                                          0, sizeof (details::cl::KernelContactPoint<KV, KT, KI>) * (size_t) kernel_contact_points_size,
                                          contact_point_container + (size_t) contact_point_container_size, NULL,
#ifdef USE_PROFILING
                                          &contact_points_event);
#else // USE_PROFILING
                                          NULL);
#endif // USE_PROFILING
            CHECK_CL_ERR(err);

#ifdef USE_PROFILING
          {
            if((device_type & CL_DEVICE_TYPE_CPU) == 0) {
              // on the CPU we use host memory, no transactions to profile
              cl_ulong start = contact_points_event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&err);
              CHECK_CL_ERR(err);
              cl_ulong end = contact_points_event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&err);
              CHECK_CL_ERR(err);
              exact_test_opencl_read_time += end - start;
            }
          }
#endif // USE_PROFILING

            contact_point_container_size += (size_t) kernel_contact_points_size;                        
          }
          
        }
        
      }
      
      kernel_work_item_generator.cleanup_generated_work_items(
                                                              kernel_work);
    }
    
    err = queue.finish();
    CHECK_CL_ERR(err);
    
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
    
    details::cl::record_kernel_times(  tandem_traversal_kernel_time
                                     , balance_work_kernel_time
                                     , exact_tests_kernel_time);
    details::cl::record_kernel_invocations(  tandem_traversal_invocations
                                           , balance_work_invocations
                                           , exact_test_invocations);
    details::cl::record_opencl_times(  tandem_traversal_opencl_read_time, tandem_traversal_opencl_write_time
                                     , balance_work_opencl_read_time, balance_work_opencl_write_time
                                     , exact_test_opencl_read_time, exact_test_opencl_write_time);
  }
  
} // namespace kdop

// KDOP_CL_GPROXIMITY_TANDEM_TRAVERSAL_H
#endif
