#ifndef DIKUCL_UTIL_H
#define	DIKUCL_UTIL_H

#include <dikucl.hpp>

#define CHECK_CL_ERR(error)                                           \
  do {                                                                \
    cl_int _error = (error);                                          \
    if(_error != CL_SUCCESS) {                                        \
      std::cerr << "OpenCL error in " << __FILE__ << ":" << __LINE__  \
        << " (" << _error << ")" << std::endl;                        \
      std::cerr.flush();                                              \
      exit(1);                                                        \
    }                                                                 \
  } while (false)

#ifndef NDEBUG_DIKUCL
#define MAX_ERROR_CODES 1024
#define FINISH_QUEUE(q)                        \
  do {                                         \
    cl_uint queue_finish_error = (q).finish(); \
    CHECK_CL_ERR(queue_finish_error);          \
  } while (false)
#else // NDEBUG_DIKUCL
#define MAX_ERROR_CODES 0
#define FINISH_QUEUE(q) do {} while (false)
#endif // NDEBUG_DIKUCL

#ifdef __linux__
#define ALIGNED_ALLOC(alignment, size) aligned_alloc((alignment), (size))
#else
#define ALIGNED_ALLOC(alignment, size) malloc((size))
#endif

#endif	// DIKUCL_UTIL_H

