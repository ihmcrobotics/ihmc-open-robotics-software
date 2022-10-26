#pragma once

#include <CL/cl.h>
#include <iostream>
#include <string>
#include <fstream>


class OpenCLManager
{
   public:
      OpenCLManager(const std::string& packagePath);
      ~OpenCLManager() = default;

      int LoadProgramIntoString(const char* filename, std::string& source);

      uint8_t CreateLoadBufferFloat(float* params, uint32_t count);
      uint8_t CreateLoadBufferUnsignedInt(uint32_t *params, uint32_t count);

      uint8_t CreateLoadReadOnlyImage2D_R16(uint16_t *depthBuffer, uint32_t width, uint32_t height);
      uint8_t CreateLoadReadOnlyImage2D_RGBA8(uint8_t *colorBuffer, uint32_t width, uint32_t height);

      uint8_t CreateReadWriteImage2D_R8(uint32_t width, uint32_t height);
      uint8_t CreateReadWriteImage2D_R16(uint32_t width, uint32_t height);
      uint8_t CreateReadWriteImage2D_RFloat(uint32_t width, uint32_t height);
      uint8_t CreateReadWriteImage2D_RGBA8(uint32_t width, uint32_t height);

      uint8_t CreateBufferInt(uint32_t count);
      uint8_t CreateBufferFloat(uint32_t count);

      void ReadImage(uint8_t image, const std::array<cl::size_t, 3>& region, void *cpuBufferPtr);
      void ReadBufferInt(uint8_t buffer, int *cpuBufferPtr, int size);
      void ReadBufferFloat(uint8_t buffer, float *cpuBufferPtr, int size);

      void Reset();
      void Finish();
      void SetArgument(const std::string& kernel, uint8_t argId, uint8_t bufferId, bool image = false);
      void SetArgumentInt(const std::string& kernel, uint8_t argId, uint32_t value);

   public:
      cl::CommandQueue commandQueue;
      cl::Kernel filterKernel, packKernel, mergeKernel, correspondenceKernel, correlationKernel,
                  centroidKernel, cylinderKernel, planesKernel, normalsKernel, hashKernel, indexKernel, diffuseKernel, parallelAddKernel;

   private:
      std::vector<cl::Image2D> images;
      std::vector<cl::Buffer> buffers;

      cl_context context;
      cl_event event;
      // cl::size_type<3> origin;
};
