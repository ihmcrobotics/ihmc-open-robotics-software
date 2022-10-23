//
// Created by quantum on 7/3/21.
//

#include "opencl_manager.h"
#include "chrono"

OpenCLManager::OpenCLManager(const std::string& packagePath)
{
   printf("Initializing OpenCL\n");

   std::vector<cl::Platform> all_platforms;
   cl::Platform::get(&all_platforms);

   if (all_platforms.size() == 0)
   {
      printf(" No platforms found. Check OpenCL installation!\n");
      exit(1);
   }
   cl::Platform default_platform = all_platforms[0];
   std::vector<cl::Device> all_devices;
   default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
   cl::Device default_device = all_devices[0];
   context = cl::Context({default_device});

   cl::Program::Sources sources;

   FILE *fp;
   char *source_str;
   cl::size_type source_size, program_size;

   fp = fopen((packagePath + "/kernels/fitting_kernel.cpp").c_str(), "rb");
   if (!fp)
   {
      printf("Failed to load kernel\n");
      printf("%s", (packagePath + "/kernels/fitting_kernel.cpp").c_str());
      return;
   }

   fseek(fp, 0, SEEK_END);
   program_size = ftell(fp);
   rewind(fp);
   source_str = (char *) malloc(program_size + 1);
   source_str[program_size] = '\0';
   fread(source_str, sizeof(char), program_size, fp);
   fclose(fp);

   std::string kernel_code(source_str);
   sources.push_back({kernel_code.c_str(), kernel_code.length()});
   cl::Program program(context, sources);
   if (program.build({default_device}) != CL_SUCCESS)
   {
      printf(" Error building: %s", program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device).c_str());
      exit(1);
   }
   commandQueue = cl::CommandQueue(context, default_device);
   filterKernel = cl::Kernel(program, "filterKernel");
   packKernel = cl::Kernel(program, "packKernel");
   mergeKernel = cl::Kernel(program, "mergeKernel");
   correspondenceKernel = cl::Kernel(program, "correspondenceKernel");
   correlationKernel = cl::Kernel(program, "correlationKernel");
   centroidKernel = cl::Kernel(program, "centroidKernel");
   cylinderKernel = cl::Kernel(program, "cylinderKernel");
   planesKernel = cl::Kernel(program, "planesKernel");
   normalsKernel = cl::Kernel(program, "normalsKernel");
   hashKernel = cl::Kernel(program, "hashKernel");
   indexKernel = cl::Kernel(program, "indexKernel");
   diffuseKernel = cl::Kernel(program, "diffuseKernel");

   printf("OpenCL Initialized Successfully\n");

}

uint8_t OpenCLManager::CreateLoadBufferFloat(float *params, uint32_t count)
{
   buffers.emplace_back(cl::Buffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float) * count, params));
   return buffers.size() - 1;
}

uint8_t OpenCLManager::CreateLoadBufferUnsignedInt(uint32_t *params, uint32_t count)
{
   buffers.emplace_back(cl::Buffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(uint32_t) * count, params));
   return buffers.size() - 1;
}

uint8_t OpenCLManager::CreateBufferInt(uint32_t count)
{
   buffers.emplace_back(cl::Buffer(context, CL_MEM_READ_WRITE, sizeof(int) * count));
   return buffers.size() - 1;
}

uint8_t OpenCLManager::CreateBufferFloat(uint32_t count)
{
   buffers.emplace_back(cl::Buffer(context, CL_MEM_READ_WRITE, sizeof(float) * count));
   return buffers.size() - 1;
}

uint8_t OpenCLManager::CreateLoadReadOnlyImage2D_R16(uint16_t *depthBuffer, uint32_t width, uint32_t height)
{
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), width, height, 0, depthBuffer));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateLoadReadOnlyImage2D_RGBA8(uint8_t *colorBuffer, uint32_t width, uint32_t height)
{
   images.emplace_back(
         cl::Image2D(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8), width, height, 0, colorBuffer));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateReadWriteImage2D_R8(uint32_t width, uint32_t height)
{
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT8), width, height));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateReadWriteImage2D_R16(uint32_t width, uint32_t height)
{
   printf("Creating RW Buffer R16: %d %d\n", width, height);
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), width, height));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateReadWriteImage2D_RFloat(uint32_t width, uint32_t height)
{
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), width, height));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateReadWriteImage2D_RGBA8(uint32_t width, uint32_t height)
{
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8), width, height));
   return images.size() - 1;
}

void OpenCLManager::ReadImage(uint8_t image, const std::array<cl::size_type, 3>& region, void *cpuBufferPtr)
{
   commandQueue.enqueueReadImage(images[image], CL_TRUE, {0,0,0}, region, 0, 0, cpuBufferPtr);
}

void OpenCLManager::ReadBufferInt(uint8_t buffer, int *cpuBufferPtr, int size)
{
   commandQueue.enqueueReadBuffer(buffers[buffer], CL_TRUE, 0, sizeof(int) * size, cpuBufferPtr);
}

void OpenCLManager::ReadBufferFloat(uint8_t buffer, float *cpuBufferPtr, int size)
{
   commandQueue.enqueueReadBuffer(buffers[buffer], CL_TRUE, 0, sizeof(float) * size, cpuBufferPtr);
}

void OpenCLManager::Reset()
{
   images.clear();
   buffers.clear();
}

void OpenCLManager::SetArgument(const std::string& kernel, uint8_t argId, uint8_t bufferId, bool image)
{
   if(kernel == "filterKernel") (image) ? filterKernel.setArg(argId, images[bufferId]) : filterKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "packKernel") (image) ? packKernel.setArg(argId, images[bufferId]) : packKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "mergeKernel") (image) ? mergeKernel.setArg(argId, images[bufferId]) : mergeKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "correspondenceKernel") (image) ? correspondenceKernel.setArg(argId, images[bufferId]) : correspondenceKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "correlationKernel") (image) ? correlationKernel.setArg(argId, images[bufferId]) : correlationKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "centroidKernel") (image) ? centroidKernel.setArg(argId, images[bufferId]) : centroidKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "cylinderKernel") (image) ? cylinderKernel.setArg(argId, images[bufferId]) : cylinderKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "planesKernel") (image) ? planesKernel.setArg(argId, images[bufferId]) : planesKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "normalsKernel") (image) ? normalsKernel.setArg(argId, images[bufferId]) : normalsKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "hashKernel") (image) ? hashKernel.setArg(argId, images[bufferId]) : hashKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "indexKernel") (image) ? indexKernel.setArg(argId, images[bufferId]) : indexKernel.setArg(argId, buffers[bufferId]);
   else if(kernel == "diffuseKernel") (image) ? diffuseKernel.setArg(argId, images[bufferId]) : diffuseKernel.setArg(argId, buffers[bufferId]);
}

void OpenCLManager::SetArgumentInt(const std::string& kernel, uint8_t argId, uint32_t value)
{
      if(kernel == "filterKernel") filterKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "mergeKernel") mergeKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "packKernel") packKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "correspondenceKernel") correspondenceKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "correlationKernel") correlationKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "centroidKernel") centroidKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "cylinderKernel") cylinderKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "planesKernel") planesKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "normalsKernel") normalsKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "hashKernel") hashKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "indexKernel") indexKernel.setArg(argId, sizeof(cl_int), &value);
      else if(kernel == "diffuseKernel") diffuseKernel.setArg(argId, sizeof(cl_int), &value);
}

void OpenCLManager::Finish()
{
   commandQueue.finish();
}


