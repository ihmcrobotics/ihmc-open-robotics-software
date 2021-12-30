package us.ihmc.perception;

import org.bytedeco.javacpp.*;
import org.bytedeco.opencl.*;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;
import java.util.ArrayList;

import static org.bytedeco.opencl.global.OpenCL.*;

/**
 * Reference: https://www.khronos.org/registry/OpenCL/sdk/2.2/docs/man/html/
 * Use `clinfo` to get more info about your setup.
 */
public class OpenCLManager
{
   private _cl_platform_id platforms = new _cl_platform_id(null);
   private _cl_device_id devices = new _cl_device_id(null);
   private _cl_context context = new _cl_context(null);
   private _cl_command_queue commandQueue = new _cl_command_queue(null);
   private final IntPointer numberOfDevices = new IntPointer(1);
   private final IntPointer numberOfPlatforms = new IntPointer(1);
   private final IntPointer returnCode = new IntPointer(1);
   private final ArrayList<_cl_program> programs = new ArrayList<>();
   private final ArrayList<_cl_kernel> kernels = new ArrayList<>();
   private final ArrayList<_cl_mem> bufferObjects = new ArrayList<>();
   private final SizeTPointer globalWorkSize = new SizeTPointer(0, 0, 0);
//   private final SizeTPointer localWorkSize = new SizeTPointer(1024, 0, 0); // TODO: Rethink this

   public void create()
   {
      /* Get platform/device information */
      checkReturnCode(clGetPlatformIDs(1, platforms, numberOfPlatforms));
      checkReturnCode(clGetDeviceIDs(platforms, CL_DEVICE_TYPE_GPU, 1, devices, numberOfDevices));

      // TODO: Print info about the setup here. Looking for CL_DEVICE_IMAGE_SUPPORT

      /* Create OpenCL Context */
      context = clCreateContext(null, 1, devices, null, null, returnCode);
      checkReturnCode();

      /* Create Command Queue */
      IntPointer properties = new IntPointer(new int[] {0});
      commandQueue = clCreateCommandQueueWithProperties(context, devices, properties, returnCode);
      checkReturnCode();
   }

   public _cl_kernel loadSingleFunctionProgramAndCreateKernel(String programName)
   {
      _cl_program program = loadProgram(programName);
      return createKernel(program, programName);
   }

   public _cl_program loadProgram(String programName)
   {
      String sourceAsString = OpenCLTools.readFile(Paths.get("openCL", programName + ".cl"));

      /* Create Kernel program from the read in source */
      int count = 1;
      _cl_program program = clCreateProgramWithSource(context,
                                                      count,
                                                      new PointerPointer(sourceAsString),
                                                      new SizeTPointer(1).put(sourceAsString.length()),
                                                      returnCode);
      checkReturnCode();
      programs.add(program);

      /* Build Kernel Program */
      checkReturnCode(clBuildProgram(program, 1, devices, null, null, null));
      int preallocatedBytes = Conversions.megabytesToBytes(2);
      CharPointer charPointer = new CharPointer(preallocatedBytes);
      SizeTPointer length = new SizeTPointer(1);
      clGetProgramBuildInfo(program, devices.getPointer(), CL_PROGRAM_BUILD_LOG, preallocatedBytes, charPointer, length);
      LogTools.info("OpenCL Build log: openCL/{}.cl", programName);
      ByteBuffer byteBuffer = charPointer.asByteBuffer();
      int logLength = (int) length.get();
      byte[] bytes = new byte[logLength];
      byteBuffer.get(bytes, 0, logLength);
      System.out.println(new String(bytes, StandardCharsets.UTF_8));

      return program;
   }

   public _cl_kernel createKernel(_cl_program program, String kernelName)
   {
      /* Create OpenCL Kernel */
      _cl_kernel kernel = clCreateKernel(program, kernelName, returnCode);
      kernels.add(kernel);
      return kernel;
   }

   public _cl_mem createBufferObject(long sizeInBytes)
   {
      Pointer hostPointer = null;
      return createBufferObject(sizeInBytes, hostPointer);
   }

   public _cl_mem createBufferObject(long sizeInBytes, Pointer hostPointer)
   {
      int flags = CL_MEM_READ_WRITE; // TODO: Provide more options
      if (hostPointer != null)
         flags |= CL_MEM_USE_HOST_PTR;
      _cl_mem bufferObject = clCreateBuffer(context, flags, sizeInBytes, hostPointer, returnCode);
      checkReturnCode();
      bufferObjects.add(bufferObject);
      return bufferObject;
   }

   public _cl_mem createImage(int flags, int width, int height, Pointer hostPointer)
   {
//      flags |= CL_MEM_COPY_HOST_PTR;
      cl_image_format imageFormat = new cl_image_format(new IntPointer(CL_R, CL_UNSIGNED_INT16));
      int rowPitch = 0;
      cl_image_desc imageDescription = new cl_image_desc(new IntPointer(
            CL_MEM_OBJECT_IMAGE2D,
            width,
            height,
            0, 0,
            rowPitch,
            0, 0, 0, 0
      ));
//      imageDescription.position(0).put(new IntPointer(new int[] {CL_MEM_OBJECT_IMAGE2D}));
//      imageDescription.position(1).put(new IntPointer(new int[] {width}));
//      imageDescription.position(2).put(new IntPointer(new int[] {height}));
//      imageDescription.position(3).put(new IntPointer(new int[] {0})); // depth
//      imageDescription.position(4).put(new IntPointer(new int[] {0})); // arraySize
//      imageDescription.position(5).put(new IntPointer(new int[] {rowPitch}));
//      imageDescription.position(6).put(new IntPointer(new int[] {0})); // slicePitch
//      imageDescription.position(7).put(new IntPointer(new int[] {0})); // number of mipmap levels
//      imageDescription.position(8).put(new IntPointer(new int[] {0})); // number of samples
//      imageDescription.position(9).put(new IntPointer(new int[] {0})); // mem_object
      _cl_mem image = clCreateImage(context, flags, imageFormat, imageDescription, hostPointer, returnCode);
      bufferObjects.add(image);
      checkReturnCode();
      return image;
   }

   public void enqueueWriteBuffer(_cl_mem bufferObject, Pointer hostMemoryPointer)
   {
      enqueueWriteBuffer(bufferObject, bufferObject.limit(), hostMemoryPointer);
   }

   public void enqueueWriteBuffer(_cl_mem bufferObject, long sizeInBytes, Pointer hostMemoryPointer)
   {
      /* Transfer data to memory buffer */
      bufferObject.position(0);
      checkReturnCode(clEnqueueWriteBuffer(commandQueue, bufferObject, CL_TRUE, 0, sizeInBytes, hostMemoryPointer, 0, (PointerPointer) null, null));
   }

   public void setKernelArgument(_cl_kernel kernel, int argumentIndex, _cl_mem bufferObject)
   {
      /* Set OpenCL kernel argument */
      int argumentSize = Pointer.sizeof(_cl_mem.class);
      checkReturnCode(clSetKernelArg(kernel, argumentIndex, argumentSize, bufferObject));
   }

   public void execute1D(_cl_kernel kernel, long workSizeX)
   {
      execute(kernel, 1, workSizeX, 0, 0);
   }

   public void execute2D(_cl_kernel kernel, long workSizeX, long workSizeY)
   {
      execute(kernel, 2, workSizeX, workSizeY, 0);
   }

   /**
    * See https://www.khronos.org/registry/OpenCL/sdk/2.2/docs/man/html/clEnqueueNDRangeKernel.html
    */
   public void execute(_cl_kernel kernel, int numberOfWorkDimensions, long workSizeX, long workSizeY, long workSizeZ)
   {
      /* Enqueue OpenCL kernel execution */
      globalWorkSize.put(0, workSizeX);
      globalWorkSize.put(1, workSizeY);
      globalWorkSize.put(2, workSizeZ);
      SizeTPointer globalWorkOffset = null; // starts at (0,0,0)
      SizeTPointer localWorkSize = null; // auto mode?
      int numberOfEventsInWaitList = 0; // no events
      PointerPointer eventWaitList = null; // no events
      PointerPointer event = null; // no events
      checkReturnCode(clEnqueueNDRangeKernel(commandQueue,
                                             kernel,
                                             numberOfWorkDimensions,
                                             globalWorkOffset,
                                             globalWorkSize,
                                             localWorkSize,
                                             numberOfEventsInWaitList,
                                             eventWaitList,
                                             event));
   }

   public void enqueueReadBuffer(_cl_mem bufferObject, Pointer hostMemoryPointer)
   {
      enqueueReadBuffer(bufferObject, bufferObject.limit(), hostMemoryPointer);
   }

   public void enqueueReadBuffer(_cl_mem bufferObject, long sizeInBytes, Pointer hostMemoryPointer)
   {
      /* Transfer result from the memory buffer */
      bufferObject.position(0);
      checkReturnCode(clEnqueueReadBuffer(commandQueue, bufferObject, CL_TRUE, 0, sizeInBytes, hostMemoryPointer, 0, (PointerPointer) null, null));
   }

   public int finish()
   {
      checkReturnCode(clFlush(commandQueue));
      checkReturnCode(clFinish(commandQueue));
      return returnCode.get();
   }

   private void checkReturnCode(int returnCode)
   {
      this.returnCode.put(returnCode);
      if (returnCode != CL_SUCCESS) // yeah it's duplicated but reduces stack trace height
      {
         LogTools.error(1, "OpenCL error code: " + returnCode);
      }
   }

   private void checkReturnCode()
   {
      if (returnCode.get() != CL_SUCCESS)
      {
         LogTools.error(1, "OpenCL error code: " + returnCode.get());
      }
   }

   public void destroy()
   {
      returnCode.put(clFlush(commandQueue));
      returnCode.put(clFinish(commandQueue));
      for (_cl_program program : programs)
         returnCode.put(clReleaseProgram(program));
      for (_cl_kernel kernel : kernels)
         returnCode.put(clReleaseKernel(kernel));
      for (_cl_mem bufferObject : bufferObjects)
         returnCode.put(clReleaseMemObject(bufferObject));
      returnCode.put(clReleaseCommandQueue(commandQueue));
      returnCode.put(clReleaseContext(context));
   }

   public int getReturnCode()
   {
      return returnCode.get();
   }
}
