package us.ihmc.perception.opencl;

import org.apache.commons.lang3.StringUtils;
import org.bytedeco.javacpp.*;
import org.bytedeco.opencl.*;
import org.bytedeco.opencl.global.OpenCL;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.TreeSet;

import static org.bytedeco.opencl.global.OpenCL.*;

/**
 * Reference: https://www.khronos.org/registry/OpenCL/sdk/2.2/docs/man/html/
 * Use `clinfo` to get more info about your setup.
 */
public class OpenCLManager
{
   // Platform and device stay the same throughout the process
   private static final _cl_platform_id platformId = new _cl_platform_id();
   private static final _cl_device_id deviceId = new _cl_device_id();
   private static volatile boolean initialized = false;

   static
   {
      if (!initialized)
      {
         Loader.load(OpenCL.class);

         /* Get platform/device information */
         IntPointer platformCount = new IntPointer(1);
         IntPointer deviceCount = new IntPointer(1);

         OpenCLTools.checkReturnCode(clGetPlatformIDs(1, platformId, platformCount));
         OpenCLTools.checkReturnCode(clGetDeviceIDs(platformId, CL_DEVICE_TYPE_DEFAULT, 1, deviceId, deviceCount));

         LogTools.info("Number of platforms: {}", platformCount.get());
         LogTools.info("Number of devices: {}", deviceCount.get());

         for (int i = 0; i < platformCount.get(); i++)
         {
            String message = "OpenCL Platform:";
            message += " Name: " + OpenCLTools.readPlatformInfoParameter(platformId, i, CL_PLATFORM_NAME);
            message += " Vendor: " + OpenCLTools.readPlatformInfoParameter(platformId, i, CL_PLATFORM_VENDOR);
            message += " Version: " + OpenCLTools.readPlatformInfoParameter(platformId, i, CL_PLATFORM_VERSION);
            LogTools.info(message);
         }

         for (int i = 0; i < deviceCount.get(); i++)
         {
            String message = "OpenCL Device:";
            message += " Name: " + OpenCLTools.readDeviceInfoParameter(deviceId, i, CL_DEVICE_NAME);
            message += " Vendor: " + OpenCLTools.readDeviceInfoParameter(deviceId, i, CL_DEVICE_VENDOR);
            message += " Driver Version: " + OpenCLTools.readDeviceInfoParameter(deviceId, i, CL_DRIVER_VERSION);
            LogTools.info(message);
         }

         initialized = true;
      }
   }

   // A new context and command queue are created for each OpenCLManager
   private final _cl_context context;
   private final _cl_command_queue commandQueue;

   private final ArrayList<_cl_program> programs = new ArrayList<>();
   private final ArrayList<_cl_kernel> kernels = new ArrayList<>();
   private final TreeSet<_cl_mem> bufferObjects = new TreeSet<>(Comparator.comparing(Pointer::address));
   private final SizeTPointer globalWorkSize = new SizeTPointer(0, 0, 0);
   private final PointerPointer tempPointerPointerForSetKernelArgument = new PointerPointer(1);
   private final long pointerPointerSize = Pointer.sizeof(PointerPointer.class);
   private final long intPointerSize = Pointer.sizeof(IntPointer.class);
   private final SizeTPointer origin = new SizeTPointer(3);
   private final SizeTPointer region = new SizeTPointer(3);
   private final IntPointer returnCode = new IntPointer(1);

   public OpenCLManager()
   {
      /* Create OpenCL Context */
      context = clCreateContext(null, 1, deviceId, null, null, returnCode);
      OpenCLTools.checkReturnCode(returnCode);

      /* Create Command Queue */
      LongPointer properties = null;
      commandQueue = clCreateCommandQueueWithProperties(context, deviceId, properties, returnCode);
      OpenCLTools.checkReturnCode(returnCode);
   }

   public _cl_kernel loadSingleFunctionProgramAndCreateKernel(String programName, String... headerFilesToInclude)
   {
      _cl_program program = loadProgram(programName, headerFilesToInclude);
      return createKernel(program, StringUtils.uncapitalize(programName));
   }

   public _cl_program loadProgram(String programName, String... headerFilesToInclude)
   {
      String sourceAsString = "";

      ArrayList<String> includedHeaders = new ArrayList<>();
      includedHeaders.add("EuclidCommon.cl");
      includedHeaders.addAll(Arrays.asList(headerFilesToInclude));

      for (String includedHeader : includedHeaders)
      {
         Path headerFilePath = Paths.get("openCL", includedHeader);
         LogTools.info("Loading OpenCL program: openCL/{}", includedHeader);
         sourceAsString += OpenCLTools.readFile(headerFilePath) + "\n";
      }

      Path programPath = Paths.get("openCL", programName + ".cl");
      LogTools.info("Loading OpenCL program: {}", programPath);
      sourceAsString += OpenCLTools.readFile(programPath);

      // Support loading from CRLF (Windows) checkouts
      sourceAsString = StringTools.filterOutCRLFLineEndings(sourceAsString);

      /* Create Kernel program from the read in source */
      int count = 1;
      _cl_program program = clCreateProgramWithSource(context,
                                                      count,
                                                      new PointerPointer(sourceAsString),
                                                      new SizeTPointer(1).put(sourceAsString.length()),
                                                      returnCode);
      OpenCLTools.checkReturnCode(returnCode);
      programs.add(program);

      /* Build Kernel Program */
      int numberOfDevices = 1;
      String options = null;
      Pfn_notify__cl_program_Pointer notificationRoutine = null;
      Pointer userData = null;
      int returnCode = clBuildProgram(program, numberOfDevices, deviceId, options, notificationRoutine, userData);
      LogTools.info("OpenCL build info for openCL/{}.cl: \n{}",
                    programName,
                    OpenCLTools.readString((stringSizeByteLimit, stringPointer, resultingStringLengthPointer) ->
                                           {
                                              clGetProgramBuildInfo(program,
                                                                    deviceId.getPointer(),
                                                                    CL_PROGRAM_BUILD_LOG,
                                                                    stringSizeByteLimit,
                                                                    stringPointer,
                                                                    resultingStringLengthPointer);
                                           }));
      OpenCLTools.checkReturnCode(returnCode);

      return program;
   }

   public _cl_kernel createKernel(_cl_program program, String kernelName)
   {
      /* Create OpenCL Kernel */
      _cl_kernel kernel = clCreateKernel(program, kernelName, returnCode);
      OpenCLTools.checkReturnCode(returnCode);
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
      OpenCLTools.checkReturnCode(returnCode);
      bufferObjects.add(bufferObject);
      return bufferObject;
   }

   public _cl_mem createImage(int flags, int imageChannelOrder, int imageChannelDataType, int width, int height, Pointer hostPointer)
   {
      if (hostPointer != null)
         flags |= CL_MEM_USE_HOST_PTR;
      cl_image_format imageFormat = new cl_image_format();
      imageFormat.image_channel_order(imageChannelOrder);
      imageFormat.image_channel_data_type(imageChannelDataType);
      cl_image_desc imageDescription = new cl_image_desc();
      imageDescription.image_type(CL_MEM_OBJECT_IMAGE2D);
      imageDescription.image_width(width);
      imageDescription.image_height(height);
      imageDescription.image_depth(0);
      imageDescription.image_array_size(0);
      imageDescription.image_row_pitch(0);
      imageDescription.image_slice_pitch(0);
      imageDescription.num_mip_levels(0);
      imageDescription.num_samples(0);
      imageDescription.mem_object(null);
      _cl_mem image = clCreateImage(context, flags, imageFormat, imageDescription, hostPointer, returnCode);
      bufferObjects.add(image);
      OpenCLTools.checkReturnCode(returnCode);
      return image;
   }

   public void enqueueWriteBuffer(_cl_mem bufferObject, long sizeInBytes, Pointer hostMemoryPointer)
   {
      /* Transfer data to memory buffer */
      int blockingWrite = CL_TRUE;
      int offset = 0;
      int numberOfEventsInWaitList = 0; // no events
      PointerPointer eventWaitList = null; // no events
      PointerPointer event = null; // no events
      OpenCLTools.checkReturnCode(clEnqueueWriteBuffer(commandQueue,
                                           bufferObject,
                                           blockingWrite,
                                           offset,
                                           sizeInBytes,
                                           hostMemoryPointer,
                                           numberOfEventsInWaitList,
                                           eventWaitList,
                                           event));
   }

   public void enqueueWriteImage(_cl_mem image, long imageWidth, long imageHeight, Pointer hostMemoryPointer)
   {
      /* Transfer data to memory buffer */
      int blockingWrite = CL_TRUE;
      origin.put(0, 0);
      origin.put(1, 0);
      origin.put(2, 0);
      region.put(0, imageWidth);
      region.put(1, imageHeight);
      region.put(2, 1);
      long inputRowPitch = 0;
      long inputSlicePitch = 0;
      int numberOfEventsInWaitList = 0; // no events
      PointerPointer eventWaitList = null; // no events
      PointerPointer event = null; // no events
      OpenCLTools.checkReturnCode(clEnqueueWriteImage(commandQueue,
                                          image,
                                          blockingWrite,
                                          origin,
                                          region,
                                          inputRowPitch,
                                          inputSlicePitch,
                                          hostMemoryPointer,
                                          numberOfEventsInWaitList,
                                          eventWaitList,
                                          event));
   }

   public void setKernelArgument(_cl_kernel kernel, int argumentIndex, _cl_mem bufferObject)
   {
      setKernelArgument(kernel, argumentIndex, pointerPointerSize, bufferObject);
   }

   public void setKernelArgument(_cl_kernel kernel, int argumentIndex, IntPointer intPointer)
   {
      setKernelArgument(kernel, argumentIndex, intPointerSize, intPointer);
   }

   private void setKernelArgument(_cl_kernel kernel, int argumentIndex, long argumentSize, Pointer bufferObject)
   {
      /* Set OpenCL kernel argument */
      tempPointerPointerForSetKernelArgument.put(bufferObject);
      OpenCLTools.checkReturnCode(clSetKernelArg(kernel, argumentIndex, argumentSize, tempPointerPointerForSetKernelArgument));
   }

   public void execute1D(_cl_kernel kernel, long workSizeX)
   {
      execute(kernel, 1, workSizeX, 0, 0);
   }

   public void execute2D(_cl_kernel kernel, long workSizeX, long workSizeY)
   {
      execute(kernel, 2, workSizeX, workSizeY, 0);
   }

   public void execute3D(_cl_kernel kernel, long workSizeX, long workSizeY, long workSizeZ)
   {
      execute(kernel, 3, workSizeX, workSizeY, workSizeZ);
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
      OpenCLTools.checkReturnCode(clEnqueueNDRangeKernel(commandQueue,
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
      OpenCLTools.checkReturnCode(clEnqueueReadBuffer(commandQueue, bufferObject, CL_TRUE, 0, sizeInBytes, hostMemoryPointer, 0, (PointerPointer) null, null));
   }

   public void enqueueReadImage(_cl_mem image, long imageWidth, long imageHeight, Pointer hostMemoryPointer)
   {
      /* Transfer result from the memory buffer */
      int blockingRead = CL_TRUE;
      origin.put(0, 0);
      origin.put(1, 0);
      origin.put(2, 0);
      region.put(0, imageWidth);
      region.put(1, imageHeight);
      region.put(2, 1);
      long inputRowPitch = 0;
      long inputSlicePitch = 0;
      int numberOfEventsInWaitList = 0; // no events
      PointerPointer eventWaitList = null; // no events
      PointerPointer event = null; // no events
      OpenCLTools.checkReturnCode(clEnqueueReadImage(commandQueue,
                                         image,
                                         blockingRead,
                                         origin,
                                         region,
                                         inputRowPitch,
                                         inputSlicePitch,
                                         hostMemoryPointer,
                                         numberOfEventsInWaitList,
                                         eventWaitList,
                                         event));
   }

   public void enqueueFillBuffer(_cl_mem bufferObject, long sizeInBytes, byte value)
   {
      ByteBuffer patternByteBuffer = ByteBuffer.allocateDirect(1);
      patternByteBuffer.put(value);
      Pointer patternPointer = new Pointer(patternByteBuffer);
      PointerPointer eventWaitList = null; // no events
      PointerPointer event = null; // no events
      OpenCLTools.checkReturnCode(clEnqueueFillBuffer(commandQueue, bufferObject, patternPointer, patternPointer.limit(), 0, sizeInBytes, 0, eventWaitList, event));
   }

   public void releaseBufferObject(_cl_mem bufferObject)
   {
      OpenCLTools.checkReturnCode(clReleaseMemObject(bufferObject));
      bufferObjects.remove(bufferObject);
   }

   public void join()
   {
      OpenCLTools.checkReturnCode(clFinish(commandQueue));
   }

   public void destroy()
   {
      for (_cl_program program : programs)
         OpenCLTools.checkReturnCode(clReleaseProgram(program));
      programs.clear();
      for (_cl_kernel kernel : kernels)
         OpenCLTools.checkReturnCode(clReleaseKernel(kernel));
      kernels.clear();
      for (_cl_mem bufferObject : bufferObjects)
         OpenCLTools.checkReturnCode(clReleaseMemObject(bufferObject));
      bufferObjects.clear();

      OpenCLTools.checkReturnCode(clFlush(commandQueue));
      OpenCLTools.checkReturnCode(clFinish(commandQueue));
      OpenCLTools.checkReturnCode(clReleaseCommandQueue(commandQueue));
      OpenCLTools.checkReturnCode(clReleaseContext(context));
   }
}
