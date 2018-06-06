#ifndef DIKUCL_PLATFORM_MANAGER_HPP
#define DIKUCL_PLATFORM_MANAGER_HPP

#include <vector>

#include <dikucl.hpp>

namespace dikucl {
    
    class PlatformHandle {
    public:
        
        size_t id;
        cl::Platform platform;
                
        PlatformHandle(
            size_t id,
            cl::Platform platform) :
            id(id),
            platform(platform)
        {
        }
        
        friend bool operator<(const PlatformHandle& ph1, const PlatformHandle& ph2) {
            return ph1.id < ph2.id;
        }
        
        friend bool operator>(const PlatformHandle& ph1, const PlatformHandle& ph2) {
            return ph1.id > ph2.id;
        }
        
        friend bool operator==(const PlatformHandle& ph1, const PlatformHandle& ph2) {
            return ph1.id == ph2.id;
        }
        
    };

    class PlatformManager {
    private:
        
        std::vector< PlatformHandle > platforms;
        
        PlatformManager()
        {
        }

        PlatformManager(PlatformManager const&);
        void operator=(PlatformManager const&);
        
    public:

        static PlatformManager& get_instance() {
            static PlatformManager instance;
            return instance;
        }
        
        PlatformHandle* get_platform(
                cl_int *err = NULL,
                size_t platform_id = 0) {
            cl_int error = CL_SUCCESS;
            
            if(this->platforms.empty()) {
                std::vector< cl::Platform > platforms;
                error = cl::Platform::get(&platforms);
                if(error == CL_SUCCESS) {
                    for(size_t i = 0; i < platforms.size(); ++i) {
                        this->platforms.push_back(PlatformHandle(i, platforms[i]));
                    }
                } else {
                    if(err != NULL) {
                        *err = error;
                    }
                    return NULL;
                }
            }
            
            if(platform_id < this->platforms.size()) {
                return &(this->platforms[platform_id]);
            }
            return NULL;
        }
        
        void reset() {
            platforms.clear();
        }
        
        size_t get_platform_count() {
            return platforms.size();
        }

    };

} // namespace dikucl

#endif // DIKUCL_PLATFORM_MANAGER_HPP