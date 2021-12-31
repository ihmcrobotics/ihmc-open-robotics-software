package us.ihmc.perception;

import org.apache.commons.lang3.StringUtils;
import org.bytedeco.javacpp.*;
import org.bytedeco.opencl.*;
import us.ihmc.log.LogTools;

import java.nio.file.Paths;
import java.util.ArrayList;

import static org.bytedeco.opencl.global.OpenCL.*;

/**
 * Reference: https://www.khronos.org/registry/OpenCL/sdk/2.2/docs/man/html/
 * Use `clinfo` to get more info about your setup.
 */
public class OpenCLManager
{
   private final int maxNumberOfEntries = 2; // More than 2 results in native crash TODO: Why?
   private _cl_platform_id platforms = new _cl_platform_id();
   private _cl_device_id devices = new _cl_device_id();
   private _cl_context context = new _cl_context();
   private _cl_command_queue commandQueue = new _cl_command_queue();
   private final IntPointer numberOfDevices = new IntPointer(1);
   private final IntPointer numberOfPlatforms = new IntPointer(3);
   private final IntPointer returnCode = new IntPointer(1);
   private final ArrayList<_cl_program> programs = new ArrayList<>();
   private final ArrayList<_cl_kernel> kernels = new ArrayList<>();
   private final ArrayList<_cl_mem> bufferObjects = new ArrayList<>();
   private final SizeTPointer globalWorkSize = new SizeTPointer(0, 0, 0);
//   private final SizeTPointer localWorkSize = new SizeTPointer(1024, 0, 0); // TODO: Rethink this

   public void create()
   {
      /* Get platform/device information */
      checkReturnCode(clGetPlatformIDs(maxNumberOfEntries, platforms, numberOfPlatforms));
      checkReturnCode(clGetDeviceIDs(platforms, CL_DEVICE_TYPE_ALL, maxNumberOfEntries, devices, numberOfDevices));

      int numberOfPlatforms = this.numberOfPlatforms.get();
      LogTools.info("Number of platforms: {}", numberOfPlatforms);
      int numberOfDevices = this.numberOfDevices.get();
      LogTools.info("Number of devices: {}", numberOfDevices);

      for (int i = 0; i < numberOfPlatforms; i++)
      {
         String message = "OpenCL Platform:";
         message += " Name: " + readPlatformInfoParameter(i, CL_PLATFORM_NAME);
         message += " Vendor: " + readPlatformInfoParameter(i, CL_PLATFORM_VENDOR);
         message += " Version: " + readPlatformInfoParameter(i, CL_PLATFORM_VERSION);
         LogTools.info(message);
      }

      for (int i = 0; i < numberOfDevices; i++)
      {
         String message = "OpenCL Device:";
         message += " Name: " + readDeviceInfoParameter(i, CL_DEVICE_NAME);
         message += " Vendor: " + readDeviceInfoParameter(i, CL_DEVICE_VENDOR);
         message += " Driver Version: " + readDeviceInfoParameter(i, CL_DRIVER_VERSION);
         LogTools.info(message);
      }

      /* Create OpenCL Context */
      context = clCreateContext(null, 1, devices, null, null, returnCode);
      checkReturnCode();

      /* Create Command Queue */
      IntPointer properties = new IntPointer(new int[] {0});
      commandQueue = clCreateCommandQueueWithProperties(context, devices, properties, returnCode);
      checkReturnCode();
   }

   private String readPlatformInfoParameter(int i, int parameterName)
   {
      return OpenCLTools.readString((stringSizeByteLimit, stringPointer, resultingStringLengthPointer) ->
      {
         checkReturnCode(clGetPlatformInfo(platforms.position(i).getPointer(),
                                           parameterName,
                                           stringSizeByteLimit,
                                           stringPointer,
                                           resultingStringLengthPointer));
      });
   }

   private String readDeviceInfoParameter(int i, int parameterName)
   {
      return OpenCLTools.readString((stringSizeByteLimit, stringPointer, resultingStringLengthPointer) ->
      {
         checkReturnCode(clGetDeviceInfo(devices.position(i).getPointer(),
                                         parameterName,
                                         stringSizeByteLimit,
                                         stringPointer,
                                         resultingStringLengthPointer));
      });
   }

   public _cl_kernel loadSingleFunctionProgramAndCreateKernel(String programName)
   {
      _cl_program program = loadProgram(programName);
      return createKernel(program, StringUtils.uncapitalize(programName));
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
      int numberOfDevices = 1;
      String options = null;
      Pfn_notify__cl_program_Pointer notificationRoutine = null;
      Pointer userData = null;
      checkReturnCode(clBuildProgram(program, numberOfDevices, devices, options, notificationRoutine, userData));
      LogTools.info("OpenCL Build log: openCL/{}.cl", programName);
      System.out.println(OpenCLTools.readString((stringSizeByteLimit, stringPointer, resultingStringLengthPointer) ->
      {
         clGetProgramBuildInfo(program, devices.getPointer(), CL_PROGRAM_BUILD_LOG, stringSizeByteLimit, stringPointer, resultingStringLengthPointer);
      }));

      return program;
   }

   public _cl_kernel createKernel(_cl_program program, String kernelName)
   {
      /* Create OpenCL Kernel */
      _cl_kernel kernel = clCreateKernel(program, kernelName, returnCode);
      checkReturnCode();
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
      int flags = CL_MEM_READ_WRITE;
      return createBufferObject(flags, sizeInBytes, hostPointer);
   }

   public _cl_mem createBufferObject(int flags, long sizeInBytes, Pointer hostPointer)
   {
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
      if (hostPointer != null)
         flags |= CL_MEM_USE_HOST_PTR;
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
      int blockingWrite = CL_TRUE;
      int offset = 0;
      int numberOfEventsInWaitList = 0; // no events
      PointerPointer eventWaitList = null; // no events
      PointerPointer event = null; // no events
      checkReturnCode(clEnqueueWriteBuffer(commandQueue,
                                           bufferObject,
                                           blockingWrite,
                                           offset,
                                           sizeInBytes,
                                           hostMemoryPointer,
                                           numberOfEventsInWaitList,
                                           eventWaitList,
                                           event));
   }

   public void setKernelArgument(_cl_kernel kernel, int argumentIndex, _cl_mem bufferObject)
   {
      long argumentSize = Pointer.sizeof(PointerPointer.class);
      setKernelArgument(kernel, argumentIndex, argumentSize, new PointerPointer(1).put(bufferObject));
   }

   public void setKernelArgument(_cl_kernel kernel, int argumentIndex, long argumentSize, Pointer bufferObject)
   {
      /* Set OpenCL kernel argument */
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
