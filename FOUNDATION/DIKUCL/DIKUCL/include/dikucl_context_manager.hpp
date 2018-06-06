#ifndef DIKUCL_CONTEXT_MANAGER_HPP
#define DIKUCL_CONTEXT_MANAGER_HPP

#include <iostream>
#include <map>
#include <utility>
#include <vector>

#include <dikucl.hpp>
#include <dikucl_device_manager.hpp>

namespace dikucl {
    
    namespace details {
        
        /**
         * Callback for errors in a context, simply prints the error string.
         * 
         * @param error_info error string
         * @param private_info implementation dependent binary data
         * @param cb size of private_info
         * @param user_data user_data from creating the context
         */
        inline void opencl_notify(  const char * error_info
                                  , const void * /* private_info */
                                  , size_t       /* cb           */
                                  , void       * /* user_data    */)
        {
            std::cerr << "OpenCL Error Information: " << error_info << std::endl;
        }
        
    } // namespace details
    
    class ContextHandle {
    public:
        
        DeviceHandle device_handle;
        cl::Context context;
                
        ContextHandle(
            DeviceHandle device_handle,
            cl::Context context) :
            device_handle(device_handle),
            context(context)
        {
        }
        
        friend bool operator<(const ContextHandle& ch1, const ContextHandle& ch2) {
            return ch1.device_handle < ch2.device_handle;
        }
        
        friend bool operator>(const ContextHandle& ch1, const ContextHandle& ch2) {
            return ch1.device_handle > ch2.device_handle;
        }
        
        friend bool operator==(const ContextHandle& ch1, const ContextHandle& ch2) {
            return ch1.device_handle == ch2.device_handle;
        }
        
    };

    class ContextManager {
    private:
        
        std::map< DeviceHandle, ContextHandle > contexts;

        ContextManager() {
        }

        ContextManager(ContextManager const&);
        void operator=(ContextManager const&);
        
    public:

        static ContextManager& get_instance() {
            static ContextManager instance;
            return instance;
        }

        ContextHandle* get_context(
                cl_int *err = NULL,
                DeviceHandle *device_handle = DeviceManager::get_instance().get_device(),
                cl_context_properties *properties = NULL,
                void (CL_CALLBACK * notifyFptr)(const char*, const void*, size_t, void*) = details::opencl_notify,
                void* data = NULL) {
            cl_int error = CL_SUCCESS;
            
            if(device_handle == NULL) {
                return NULL;
            }
            
            if(contexts.count(*device_handle) == 0) {
                std::vector< cl::Device > device_vector(1, device_handle->device);
                cl::Context context(device_vector, properties, notifyFptr, data, &error);
                if(error == CL_SUCCESS) {
                    contexts.insert(std::pair< DeviceHandle, ContextHandle >(
                            *device_handle,
                            ContextHandle(*device_handle, context)));
                } else {
                    if(err != NULL) {
                        *err = error;
                    }
                    return NULL;
                }
            }
            
            std::map< DeviceHandle, ContextHandle >::iterator it = contexts.find(*device_handle);
            if(it != contexts.end()) {
                return &(it->second);
            }
            return NULL;
        }
        
        void reset() {
            contexts.clear();
        }

    };

} // namespace dikucl

#endif // DIKUCL_CONTEXT_MANAGER_HPP