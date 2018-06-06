#ifndef DIKUCL_IO_H
#define DIKUCL_IO_H

#include <string>
#include <vector>

#include <dikucl.hpp>

namespace dikucl {

    std::ostream & operator<<(std::ostream & output, cl::Platform const & platform) {
        // http://www.khronos.org/registry/cl/sdk/1.2/docs/man/xhtml/clGetPlatformInfo.html
        output
                << "Profile: "
                << platform.getInfo<CL_PLATFORM_PROFILE>()
                << std::endl
                << "Version: "
                << platform.getInfo<CL_PLATFORM_VERSION>()
                << std::endl
                << "Name: "
                << platform.getInfo<CL_PLATFORM_NAME>()
                << std::endl
                << "Vendor: "
                << platform.getInfo<CL_PLATFORM_VENDOR>()
                << std::endl
                << "Extensions: "
                << platform.getInfo<CL_PLATFORM_EXTENSIONS>()
                << std::endl;
        return output;
    }

    std::ostream & operator<<(std::ostream & output, cl::Device const & device) {
        // http://www.khronos.org/registry/cl/sdk/1.2/docs/man/xhtml/clGetDeviceInfo.html
        output
                << "Name: "
                << device.getInfo<CL_DEVICE_NAME>()
                << std::endl
                << "Version: "
                << device.getInfo<CL_DEVICE_VERSION>()
                << std::endl
                << "Driver Version: "
                << device.getInfo<CL_DRIVER_VERSION>()
                << std::endl
                << "Global Memory Size: "
                << device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>() / 1048576.0
                << "MB" << std::endl
                << "Local Memory Size: "
                << device.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>() / 1024.0
                << "kB" << std::endl
                << "Maximum Memory Allocation Size: "
                << device.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>() / 1048576.0
                << "MB" << std::endl
                << "Maximum Compute Units: "
                << device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>()
                << std::endl
                << "Maximum Work Group Size: "
                << device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>()
                << std::endl
                << "Type: "
                << device.getInfo<CL_DEVICE_TYPE>()
                << std::endl
                << "Maximum Work Item Dimensions: ";
        cl_uint maximum_work_item_dimensions = device.getInfo<CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS>();
        output
                << maximum_work_item_dimensions
                << std::endl
                << "Maximum Work Item Sizes: ";
        std::vector<std::size_t> maximum_work_item_sizes = device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>();
        for (cl_uint i = 0; i < maximum_work_item_dimensions; ++i) {
            output << maximum_work_item_sizes[i] << " ";
        }
        output << std::endl;
        return output;
    }

    struct KernelIO {
        const cl::Kernel& kernel;
        const cl::Device& device;
    };

    std::ostream & operator<<(std::ostream & output, KernelIO const & kernel_io) {
        // http://www.khronos.org/registry/cl/sdk/1.2/docs/man/xhtml/clGetKernelWorkGroupInfo.html
#if defined(CL_VERSION_1_2)
        output << "Global Work Size: ";
        cl::size_t<3> global_work_size = kernel_io.kernel.getWorkGroupInfo<CL_KERNEL_GLOBAL_WORK_SIZE>(kernel_io.device);
        for (cl_uint i = 0; i < 3; ++i) {
            output << global_work_size[i] << " ";
        }
#endif // CL_VERSION_1_2
        output
                << std::endl
                << "Work Group Size: "
                << kernel_io.kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(kernel_io.device)
                << std::endl
                << "Compile Work Group Size: ";
        cl::size_t<3> compile_work_group_size = kernel_io.kernel.getWorkGroupInfo<CL_KERNEL_COMPILE_WORK_GROUP_SIZE>(kernel_io.device);
        for (cl_uint i = 0; i < 3; ++i) {
            output << compile_work_group_size[i] << " ";
        }
        output
                << std::endl
                << "Local Memory Size: "
                << kernel_io.kernel.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(kernel_io.device) / 1024.0
                << "kB" << std::endl
                << "Preferred Work Group Size Multiple: "
                << kernel_io.kernel.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(kernel_io.device)
                << std::endl
                << "Private Memory Size: "
                << kernel_io.kernel.getWorkGroupInfo<CL_KERNEL_PRIVATE_MEM_SIZE>(kernel_io.device)
                << std::endl;
        return output;
    }

} // namespace dikucl

#endif // DIKUCL_IO_H

