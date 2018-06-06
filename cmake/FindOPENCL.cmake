
IF(APPLE)
  
  FIND_PATH(
    OPENCL_INCLUDE_DIR
    NAMES
    OpenCL/cl.hpp
    PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/FOUNDATION/DIKUCL/DIKUCL/include  # when specified from CMakeLists.txt
    )
    
ELSE(APPLE)
  
  FIND_PATH(
    OPENCL_INCLUDE_DIR
    NAMES
    CL/cl.hpp
    PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/FOUNDATION/DIKUCL/DIKUCL/include  # when specified from CMakeLists.txt
    )
  
ENDIF(APPLE)

IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
  
  SET(
    OPENCL_LIB_SEARCH_PATH
    ${OPENCL_LIB_SEARCH_PATH}
    /usr/lib32/nvidia-current
    $ENV{AMDAPPSDKROOT}/lib/x86
    $ENV{INTELOCLSDKROOT}/lib/x86
    $ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/lib/Win32
    # Legacy Stream SDK
    $ENV{ATISTREAMSDKROOT}/lib/x86
    )

ELSEIF(CMAKE_SIZEOF_VOID_P EQUAL 8)
  
  SET(
    OPENCL_LIB_SEARCH_PATH
    ${OPENCL_LIB_SEARCH_PATH}
    /usr/lib/nvidia-current
    $ENV{AMDAPPSDKROOT}/lib/x86_64
    $ENV{INTELOCLSDKROOT}/lib/x64
    $ENV{NVSDKCOMPUTE_ROOT}/OpenCL/common/lib/x64
    # Legacy stream SDK
    $ENV{ATISTREAMSDKROOT}/lib/x86_64
    )
  
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 4)

FIND_LIBRARY(
  OPENCL_LIBRARY
  NAMES OpenCL
  PATHS ${OPENCL_LIB_SEARCH_PATH}
  )

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  OpenCL
  DEFAULT_MSG
  OPENCL_LIBRARY OPENCL_INCLUDE_DIR
  )

IF(OPENCL_FOUND)
  SET(OPENCL_LIBRARIES ${OPENCL_LIBRARY})
ELSE(OPENCL_FOUND)
  SET(OPENCL_LIBRARIES)
ENDIF(OPENCL_FOUND)

MARK_AS_ADVANCED( OPENCL_INCLUDE_DIR )
MARK_AS_ADVANCED( OPENCL_LIBRARIES   )
MARK_AS_ADVANCED( OPENCL_FOUND       )
