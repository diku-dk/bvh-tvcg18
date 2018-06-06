#ifndef DIKUCL_KERNEL_MANAGER_HPP
#define DIKUCL_KERNEL_MANAGER_HPP

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <dikucl.hpp>
#include <dikucl_context_manager.hpp>

namespace dikucl {
    
    class KernelInfo {
    private:
        
        std::string m_path, m_name;
        
        std::map< std::string, std::string > m_definitions;
        
        std::set< std::string > m_include_directories;
        
        bool m_cl_nv_verbose;
                
        bool m_cl_no_signed_zeros;
        
        bool m_debug_intel;
        
        std::string m_build_options;
        
    public:
        
        KernelInfo(std::string path, std::string name) :
        m_path(path), m_name(name), m_cl_nv_verbose(false), m_cl_no_signed_zeros(false),
        m_debug_intel(false)
        {
        }
        
        KernelInfo(std::string directory, std::string filename, std::string name) :
        m_name(name), m_cl_nv_verbose(false), m_cl_no_signed_zeros(false),
        m_debug_intel(false)
        {
            std::ostringstream path;
            path << directory << filename;
            this->m_path = path.str();
        }
        
        template< typename T >
        KernelInfo& define(std::string name, T definition) {
            std::ostringstream _definition;
            _definition << definition;
            m_definitions[name] = _definition.str();
            set_build_options();
            return *this;
        }
        
        KernelInfo& undefine(std::string name) {
            m_definitions.erase(name);
            set_build_options();
            return *this;
        }
        
        KernelInfo& include(std::string include_directory) {
            m_include_directories.insert(include_directory);
            set_build_options();
            return *this;
        }
        
        KernelInfo& exclude(std::string include_directory) {
            m_include_directories.erase(include_directory);
            set_build_options();
            return *this;
        }
        
        KernelInfo& nv_verbose(bool v) {
            m_cl_nv_verbose = v;
            set_build_options();
            return *this;
        }
        
        KernelInfo& debug_intel(bool di) {
            m_debug_intel = di;
            set_build_options();
            return *this;
        }
        
        KernelInfo& no_signed_zeros(bool nsz) {
            m_cl_no_signed_zeros = nsz;
            set_build_options();
            return *this;
        }
        
        std::string get_path() const {
            return m_path;
        }
        
        std::string get_name() const {
            return m_name;
        }
        
        bool get_nv_verbose() const {
            return m_cl_nv_verbose;
        }
        
        bool get_debug_intel() const {
            return m_debug_intel;
        }
        
        bool get_no_signed_zeros() const {
            return m_cl_no_signed_zeros;
        }
        
        std::string get_build_options() const {
            return m_build_options;
        }
        
        friend bool operator<(const KernelInfo& ki1, const KernelInfo& ki2) {
            if(ki1.m_path.compare(ki2.m_path) < 0) {
                return true;
            } else if(ki1.m_path.compare(ki2.m_path) > 0) {
                return false;
            }
            if(ki1.m_name.compare(ki2.m_name) < 0) {
                return true;
            } else if(ki1.m_name.compare(ki2.m_name) > 0) {
                return false;
            }
            return ki1.m_build_options.compare(ki2.m_build_options) < 0;
        }
        
        friend bool operator>(const KernelInfo& ki1, const KernelInfo& ki2) {
            if(ki1.m_path.compare(ki2.m_path) > 0) {
                return true;
            } else if(ki1.m_path.compare(ki2.m_path) < 0) {
                return false;
            }
            if(ki1.m_name.compare(ki2.m_name) > 0) {
                return true;
            } else if(ki1.m_name.compare(ki2.m_name) < 0) {
                return false;
            }
            return ki1.m_build_options.compare(ki2.m_build_options) > 0;
        }
        
        friend bool operator==(const KernelInfo& ki1, const KernelInfo& ki2) {
            return  ki1.m_path.compare(ki2.m_path) == 0 &&
                    ki1.m_name.compare(ki2.m_name) == 0 &&
                    ki1.m_build_options.compare(ki2.m_build_options) == 0;
        }
        
    private:
        
        void set_build_options() {
            std::ostringstream options;
            
            if(!m_definitions.empty()) {
                for(  std::map< std::string, std::string >::iterator it = m_definitions.begin()
                    ; it != m_definitions.end()
                    ; ++it) {
                    if(it != m_definitions.begin()) {
                        options << " ";
                    }
                    options << "-D" << it->first << "=" << it->second;
                }
            }
            
            if(!m_include_directories.empty()) {
                for(  std::set< std::string >::iterator it = m_include_directories.begin()
                    ; it != m_include_directories.end()
                    ; ++it) {
                    if(it != m_include_directories.begin() || !m_definitions.empty()) {
                        // either this is not the first include or the options have been filled before
                        options << " ";
                    }
                    options << "-I" << (*it);
                }
            }
            
            if(m_cl_nv_verbose) {
                if(!m_definitions.empty() || !m_include_directories.empty()) {
                    options << " ";
                }
                options << "-cl-nv-verbose";
            }
            
            if(m_cl_no_signed_zeros) {
                if(!m_definitions.empty() || !m_include_directories.empty() || !m_cl_nv_verbose) {
                    options << " ";
                }
                options << "-cl-no-signed-zeros";
            }
            
            if(m_debug_intel) {
                if(!m_definitions.empty() || !m_include_directories.empty() || !m_cl_nv_verbose ||
                   !m_cl_no_signed_zeros)
                {
                    options << " ";
                }
                options << "-g -s " << get_path();
            }
                        
            m_build_options = options.str();
        }
    };
    
    class KernelHandle {
    public:
        
        ContextHandle context_handle;
        cl::Kernel kernel;
                
        KernelHandle(
            ContextHandle context_handle,
            cl::Kernel kernel) :
            context_handle(context_handle),
            kernel(kernel)
        {
        }
        
    };

    class KernelManager {
    private:

        std::map< ContextHandle, std::map< KernelInfo, KernelHandle > > kernels;

        KernelManager() {
        }

        KernelManager(KernelManager const&);
        void operator=(KernelManager const&);
        
    public:

        static KernelManager& get_instance() {
            static KernelManager instance;
            return instance;
        }

        KernelHandle* get_kernel(
                KernelInfo kernel_info,
                cl_int *err = NULL,
                ContextHandle *context_handle = ContextManager::get_instance().get_context()) {
            cl_int error = CL_SUCCESS;
            
            if(context_handle == NULL) {
                return NULL;
            }
            
            if (kernels.count(*context_handle) == 0) {
                kernels.insert(std::pair< ContextHandle, std::map< KernelInfo, KernelHandle > >(
                        *context_handle, std::map< KernelInfo, KernelHandle >()));
            }
            
            std::map< ContextHandle, std::map< KernelInfo, KernelHandle > >::iterator it =
                    kernels.find(*context_handle);
            if(it != kernels.end()) {
                if(it->second.count(kernel_info) == 0) {
                    std::string device_name =
                            context_handle->device_handle.device.getInfo<CL_DEVICE_NAME>(&error);
                    if(error != CL_SUCCESS) {
                        if(err != NULL) {
                            *err = error;
                        }
                        return NULL;
                    }
                    
                    cl_uint max_compute_units =
                            context_handle->device_handle.device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>(&error);
                    if(error != CL_SUCCESS) {
                        if(err != NULL) {
                            *err = error;
                        }
                        return NULL;
                    }
                    
                    cl_ulong global_mem_size =
                            context_handle->device_handle.device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>(&error);
                    if(error != CL_SUCCESS) {
                        if(err != NULL) {
                            *err = error;
                        }
                        return NULL;
                    }
                    
                    cl_uint max_clock_frequency =
                            context_handle->device_handle.device.getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>(&error);
                    if(error != CL_SUCCESS) {
                        if(err != NULL) {
                            *err = error;
                        }
                        return NULL;
                    }
                    
                    // read the file
                    std::ifstream kernel_file_stream(kernel_info.get_path().c_str());
                    std::string kernel_file(
                            std::istreambuf_iterator<char>(kernel_file_stream),
                            (std::istreambuf_iterator<char>()));

                    // create the program
                    cl::Program::Sources kernel_sources(
                            1,
                            std::make_pair(kernel_file.c_str(), kernel_file.length() + 1));
                    cl::Program program(context_handle->context, kernel_sources, &error);
                    
                    if(error == CL_SUCCESS) {
                        std::string build_options;
                        bool disable_nv_verbose = false;
                        if(kernel_info.get_nv_verbose()) {
                            // check that the build option is supported
                            std::string device_extensions =
                                context_handle->device_handle.device.getInfo<CL_DEVICE_EXTENSIONS>(&error);
                            if(error == CL_SUCCESS) {
                                if(device_extensions.find("cl_nv_compiler_options") == std::string::npos) {
                                    std::cerr << kernel_info.get_path() << " : " << kernel_info.get_name()
                                            << " on " << device_name << " :" << std::endl
                                            << "\tverbose compilation not supported, disabling." << std::endl;
                                    disable_nv_verbose = true;
                                } else {
                                    build_options = kernel_info.get_build_options();
                                }
                            } else { // get platform extensions
                                if(err != NULL) {
                                    *err = error;
                                }
                                return NULL;
                            }
                        }
                        
                        bool disable_debug_intel = false;
                        if(kernel_info.get_debug_intel()) {
                            cl_device_type device_type = context_handle->device_handle.device.getInfo<CL_DEVICE_TYPE>(&error);
                            if(error == CL_SUCCESS) {
                                std::string vendor = context_handle->device_handle.device.getInfo<CL_DEVICE_VENDOR>(&error);
                                if(error == CL_SUCCESS) {
                                    if(!(device_type & CL_DEVICE_TYPE_CPU) || vendor.find("Intel") == std::string::npos) {
                                        std::cerr << kernel_info.get_path() << " : " << kernel_info.get_name()
                                                << " on " << device_name << " :" << std::endl
                                                << "\tthis does not seem to be an Intel CPU, disabling debugging for Intel CPUs."
                                                << std::endl;
                                        disable_debug_intel = true;
                                    }
                                } else { // get device vendor
                                    if(err != NULL) {
                                        *err = error;
                                    }
                                    return NULL;
                                }
                            } else { // get device type
                                if(err != NULL) {
                                    *err = error;
                                }
                                return NULL;
                            }
                        }
                        
                        // disable some options for compiling
                        if(disable_nv_verbose) {
                            kernel_info.nv_verbose(false);
                        }
                        if(disable_debug_intel) {
                            kernel_info.debug_intel(false);
                        }
                        
                        build_options = kernel_info.get_build_options();
                        
                        if(disable_nv_verbose) {
                            kernel_info.nv_verbose(true);
                        }
                        if(disable_debug_intel) {
                            kernel_info.debug_intel(true);
                        }
                        
                        std::vector< cl::Device > device(1, context_handle->device_handle.device);
                        error = program.build(device, build_options.c_str());
                        cl_int build_log_error = CL_SUCCESS;
                        std::string build_log =
                                program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(context_handle->device_handle.device, &build_log_error);
                        if(build_log_error == CL_SUCCESS) {
                            if(!build_log.empty()) {
                                std::cerr << kernel_info.get_path() << " : " << kernel_info.get_name()
                                        << " on " << device_name << " ("
                                        << (global_mem_size / 1048576) << "MB @ "
                                        << max_compute_units << "x"
                                        << max_clock_frequency << "MHz)" << std::endl
                                        << "\tbuild log :" << std::endl
                                        << build_log << std::endl << std::endl;
                            }
                        } else { // get build log
                            if(err != NULL) {
                                *err = build_log_error;
                            }
                            return NULL;
                        }
                        
                        if (error == CL_SUCCESS) {
                            cl::Kernel kernel(program, kernel_info.get_name().c_str(), &error);
                            if(error == CL_SUCCESS) {
                                it->second.insert(std::pair< KernelInfo, KernelHandle >(
                                        kernel_info,
                                        KernelHandle(*context_handle, kernel)));
                            } else { // kernel constructor
                                if(err != NULL) {
                                    *err = error;
                                }
                                return NULL;
                            }
                        } else { // build program
                            if(err != NULL) {
                                *err = error;
                            }
                            return NULL;
                        }
                    } else { // program constructor
                        if(err != NULL) {
                            *err = error;
                        }
                        return NULL;
                    }
                }
            } else { // get kernel map for context
                return NULL;
            }
            
            std::map< KernelInfo, KernelHandle >::iterator _it = it->second.find(kernel_info);
            if(_it != it->second.end()) {
                return &(_it->second);
            }
            return NULL;
        }
        
        void reset() {
            kernels.clear();
        }

    };

} // namespace dikucl

#endif // DIKUCL_KERNEL_MANAGER_HPP