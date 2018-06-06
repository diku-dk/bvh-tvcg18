#ifndef DIKUCL_COMMAND_QUEUE_MANAGER_HPP
#define DIKUCL_COMMAND_QUEUE_MANAGER_HPP

#include <map>
#include <utility>
#include <vector>

#include <dikucl.hpp>
#include <dikucl_context_manager.hpp>

namespace dikucl {
    
    class CommandQueueHandle {
    public:
        
        ContextHandle context_handle;
        cl::CommandQueue command_queue;
                
        CommandQueueHandle(
            ContextHandle context_handle,
            cl::CommandQueue command_queue) :
            context_handle(context_handle),
            command_queue(command_queue)
        {
        }
        
    };

    class CommandQueueManager {
    private:

        std::map< ContextHandle, CommandQueueHandle > command_queues;

        CommandQueueManager() {
        }

        CommandQueueManager(CommandQueueManager const&);
        void operator=(CommandQueueManager const&);
        
    public:

        static CommandQueueManager& get_instance() {
            static CommandQueueManager instance;
            return instance;
        }

        CommandQueueHandle* get_command_queue(
                cl_int *err = NULL,
                ContextHandle *context_handle = ContextManager::get_instance().get_context(),
                cl_command_queue_properties properties = 0) {
            cl_int error = CL_SUCCESS;
            
            if(context_handle == NULL) {
                return NULL;
            }
            
            if(command_queues.count(*context_handle) == 0) {
                cl::CommandQueue command_queue(
                        context_handle->context,
                        context_handle->device_handle.device,
                        properties,
                        &error);
                if(error == CL_SUCCESS) {
                    command_queues.insert(std::pair< ContextHandle, CommandQueueHandle >(
                            *context_handle,
                            CommandQueueHandle(*context_handle, command_queue)));
                } else {
                    if(err != NULL) {
                        *err = error;
                    }
                    return NULL;
                }
            }
            
            std::map< ContextHandle, CommandQueueHandle >::iterator it = command_queues.find(*context_handle);
            if(it != command_queues.end()) {
                return &(it->second);
            }
            return NULL;
        }
        
        void reset() {
            command_queues.clear();
        }

    };

} // namespace dikucl

#endif // DIKUCL_COMMAND_QUEUE_MANAGER_HPP