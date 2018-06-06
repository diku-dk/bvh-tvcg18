#ifndef DIKUCL_DEVICE_MANAGER_HPP
#define DIKUCL_DEVICE_MANAGER_HPP

#include <map>
#include <utility>
#include <vector>

#include <dikucl.hpp>
#include <dikucl_platform_manager.hpp>
#include <util_get_environment.h>

namespace dikucl {
    
    class DeviceHandle {
    public:
        
        PlatformHandle platform_handle;
        size_t id;
        cl::Device device;
                
        DeviceHandle(
            PlatformHandle platform_handle,
            size_t id,
            cl::Device device) :
            platform_handle(platform_handle),
            id(id),
            device(device)
        {
        }
        
        friend bool operator<(const DeviceHandle& dh1, const DeviceHandle& dh2) {
            if(dh1.platform_handle < dh2.platform_handle) {
                return true;
            } else if(dh1.platform_handle > dh2.platform_handle) {
                return false;
            }
            return dh1.id < dh2.id;
        }
        
        friend bool operator>(const DeviceHandle& dh1, const DeviceHandle& dh2) {
            if(dh1.platform_handle > dh2.platform_handle) {
                return true;
            } else if(dh1.platform_handle < dh2.platform_handle) {
                return false;
            }
            return dh1.id > dh2.id;
        }
        
        friend bool operator==(const DeviceHandle& dh1, const DeviceHandle& dh2) {
            return  dh1.platform_handle == dh2.platform_handle &&
                    dh1.id == dh2.id;
        }
        
    };

    class DeviceManager {
    private:
        
        std::map< PlatformHandle, std::vector< DeviceHandle > > devices;
        
        DeviceManager() 
        {
        }
        
        static size_t default_device() {
            return util::get_environment<size_t>("DIKUCL_DEFAULT_DEVICE", 0u);
        }

        DeviceManager(DeviceManager const&);
        void operator=(DeviceManager const&);
        
    public:

        static DeviceManager& get_instance() {
            static DeviceManager instance;
            return instance;
        }

        DeviceHandle* get_device(
                cl_int *err = NULL,
                PlatformHandle *platform_handle = PlatformManager::get_instance().get_platform(),
                size_t device_id = default_device()) {
            cl_int error = CL_SUCCESS;
            
            if(platform_handle == NULL) {
                return NULL;
            }
            
            if(this->devices.count(*platform_handle) == 0) {
                std::vector< cl::Device > devices;
                error = platform_handle->platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
                if(error == CL_SUCCESS) {
                    std::vector< DeviceHandle > device_handles;
                    for(size_t i = 0; i < devices.size(); ++i) {
                        device_handles.push_back(DeviceHandle(*platform_handle, i, devices[i]));
                    }
                    this->devices.insert(std::pair< PlatformHandle, std::vector< DeviceHandle > >(
                            *platform_handle,
                            device_handles));
                } else {
                    if(err != NULL) {
                        *err = error;
                    }
                    return NULL;
                }
            }
            
            std::map< PlatformHandle, std::vector< DeviceHandle > >::iterator it = this->devices.find(*platform_handle);
            if(it != this->devices.end()) {
                if(device_id < it->second.size()) {
                    return &(it->second[device_id]);
                } else {
                    return NULL;
                }
            }
            return NULL;
        }
        
        void reset() {
            devices.clear();
        }
        
        size_t get_device_count(PlatformHandle platform_handle) {
            std::map< PlatformHandle, std::vector< DeviceHandle > >::iterator it = this->devices.find(platform_handle);
            if(it != devices.end()) {
                return it->second.size();
            } else {
                return 0;
            }
        }

    };

} // namespace dikucl

#endif // DIKUCL_DEVICE_MANAGER_HPP