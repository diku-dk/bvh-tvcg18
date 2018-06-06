IF(COMMAND cmake_policy)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0043 NEW) # QT stuff needs this?
ENDIF(COMMAND cmake_policy)

PROJECT(PROX)

##########################################################
##########################################################
##########################################################

CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0 FATAL_ERROR)

##########################################################
##########################################################
##########################################################

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

##########################################################
##########################################################
##########################################################

IF (MSVC)
  
  ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
  
ELSE (MSVC)
  
  SET(
    CMAKE_CXX_FLAGS
    "-Wall -Wno-unused-local-typedef" CACHE STRING ""
    FORCE
    )
  
  MESSAGE(
    WARNING
    "Forcing to use CMAKE_CXX_FLAGS = " ${CMAKE_CXX_FLAGS} " because Kenny is lazy"
    )
  
ENDIF(MSVC)

IF(CMAKE_GENERATOR MATCHES Xcode)		  

  SET(
    CMAKE_OSX_DEPLOYMENT_TARGET "10.10"
    CACHE STRING "Deployment target for OSX"
    FORCE
    )
  
  MESSAGE(
    WARNING
    "Forcing to use CMAKE_OSX_DEPLOYMENT_TARGET  = " ${CMAKE_OSX_DEPLOYMENT_TARGET} " because Kenny is lazy"
    )
  
ENDIF(CMAKE_GENERATOR MATCHES Xcode)		

##########################################################
##########################################################
##########################################################

#--- Make sure compiler knows we want to use c++11 features if they exist     ----
#--- This works fine with "system" compiler, but NVCC will not work with this  ---

INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

IF(COMPILER_SUPPORTS_CXX11)
  
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  ADD_DEFINITIONS( -DHAS_CXX11 )

  MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
  
ELSEIF(COMPILER_SUPPORTS_CXX0X)
  
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  ADD_DEFINITIONS( -DHAS_CXX0X )
  
  MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has CXX0X support.")

ELSE()
  
  MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  
ENDIF()


##########################################################
##########################################################
##########################################################

SET(
  CMAKE_MODULE_PATH 
  ${PROJECT_SOURCE_DIR}/CMake 
  ${PROJECT_SOURCE_DIR}/cmake 
  ${CMAKE_MODULE_PATH}
  )

##########################################################
##########################################################
##########################################################

SET(Boost_USE_STATIC_LIBS ON)
IF (MSVC)
  SET( BOOST_LIBRARYDIR "${Boost_INCLUDE_DIR}/lib")
ENDIF(MSVC)

FIND_PACKAGE(
  Boost
  COMPONENTS
  system
  filesystem
  REQUIRED
  )

##########################################################
##########################################################
##########################################################

ADD_DEFINITIONS(-D_USE_MATH_DEFINES)

##########################################################
##########################################################
##########################################################

FIND_PACKAGE(LAPACK)

##########################################################
##########################################################
##########################################################

FIND_PACKAGE(OPENCL)

IF(NOT OPENCL_FOUND)

  MESSAGE("OpenCL could not be found on the system")

ENDIF()

##########################################################
##########################################################
##########################################################

SET(
  ENABLE_DIKUCL
  1
  CACHE STRING
  "Set to 1 if DIKUCL should be enabled and 0 otherwise"
  )

IF(ENABLE_DIKUCL)
  
  SET(HAS_DIKUCL ${OPENCL_FOUND})
  
  IF(OPENCL_FOUND)
    
    ADD_DEFINITIONS(-DHAS_DIKUCL)
    
    MESSAGE("HAS_DIKUCL is turned...................ON")
    
  ELSE()
    
    MESSAGE("HAS_DIKUCL is turned...................OFF")
    
  ENDIF()
  
ENDIF()

##########################################################
##########################################################
##########################################################

SET(ENABLE_DEBUG_DIKUCL 1 CACHE STRING "Set to 1 if OpenCL debugging should be enabled and 0 otherwise")

IF(ENABLE_DIKUCL)
  
  IF(HAS_DIKUCL)
  
    IF(NOT ENABLE_DEBUG_DIKUCL)
    
      MESSAGE("OpenCL Debugging is turned.............OFF")
      ADD_DEFINITIONS(-DNDEBUG_DIKUCL)
    
    ELSE()
    
      MESSAGE("OpenCL Debugging is turned.............ON")
    
    ENDIF()
  
  ENDIF()

ENDIF()

##########################################################
##########################################################
##########################################################

FIND_PACKAGE(OpenGL REQUIRED)

##########################################################
##########################################################
##########################################################

FIND_PACKAGE(GLEW REQUIRED)

##########################################################
##########################################################
##########################################################

FIND_PACKAGE(DevIL) 
IF(IL_FOUND)
  
  ADD_DEFINITIONS(-D__DEVILLIB__)
  ADD_DEFINITIONS(-DILUT_USE_OPENGL) # Michael: Otherwise ilutGLScreen is not available
  SET(DEVIL_INCLUDE_DIR ${IL_INCLUDE_DIR})
  SET(DEVIL_LIBRARIES
    ${ILUT_LIBRARIES}
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    )
  
ELSE()
  
  MESSAGE("DevIL is not found")

  SET(DEVIL_INCLUDE_DIR "")
  SET(DEVIL_LIBRARIES   "")
  
ENDIF()

##########################################################
##########################################################
##########################################################

SET( TMP_CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} )

# point to your Qt installation directory
SET( CMAKE_PREFIX_PATH "$ENV{HOME}/Qt/5.3/clang_64/" ${TMP_CMAKE_PREFIX_PATH} )

FIND_PACKAGE(Qt5Widgets  REQUIRED)
FIND_PACKAGE(Qt5Core     REQUIRED)
FIND_PACKAGE(Qt5Gui      REQUIRED)
FIND_PACKAGE(Qt5OpenGL   REQUIRED)

SET( CMAKE_PREFIX_PATH ${TMP_CMAKE_PREFIX_PATH} )


##########################################################
##########################################################
##########################################################
#
# Tell that TinyXML in 3RDPARTY should use
# std::strings over char *
#
# TinyXML can work both with stl strings and char*. The
# functionality is controlled by defining the pre-processor
# directive TIXML_USE_STL or not.
#
# Note that every target that links against TinyXML
# also need to have the TIXML_USE_STL flag defined!
#
ADD_DEFINITIONS(-DTIXML_USE_STL)

##########################################################
##########################################################
##########################################################

SET(ENABLE_PROFILING     1 CACHE STRING "Set to 1 if profiling should be enabled and 0 otherwise"                )

IF(ENABLE_PROFILING)
  
  MESSAGE("Profiling is...........................ON")
  ADD_DEFINITIONS(-DUSE_PROFILING)

ELSE()

  MESSAGE("Profiling is...........................OFF")
    
ENDIF()

##########################################################
##########################################################
##########################################################

ADD_SUBDIRECTORY(3RDPARTY)
ADD_SUBDIRECTORY(FOUNDATION)
ADD_SUBDIRECTORY(SIMULATION)
ADD_SUBDIRECTORY(APPLICATIONS)

##########################################################
##########################################################
##########################################################

