package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencl.*;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static org.bytedeco.opencl.global.OpenCL.*;

public class OpenCLManagerTest
{
   @Tag("gui")
   @Test
   public void testLoadingAndPrintingParameters()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLManager.destroy();
   }

   @Tag("gui")
   @Test
   public void testOpenCLContext()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      openCLManager.create();
      _cl_kernel kernel = openCLManager.loadSingleFunctionProgramAndCreateKernel("VectorAddition");
      long numberOfFloats = 128;
      long sizeInBytes = numberOfFloats * Float.BYTES;
      FloatPointer hostMemoryPointer = new FloatPointer(numberOfFloats);
      for (int i = 0; i < numberOfFloats; i++)
      {
         hostMemoryPointer.put(i, i);
      }
      _cl_mem bufferObject = openCLManager.createBufferObject(sizeInBytes, hostMemoryPointer);
      openCLManager.enqueueWriteBuffer(bufferObject, sizeInBytes, hostMemoryPointer);
      openCLManager.setKernelArgument(kernel, 0, bufferObject);
      openCLManager.execute1D(kernel, numberOfFloats);
      openCLManager.enqueueReadBuffer(bufferObject, sizeInBytes, hostMemoryPointer);
      openCLManager.finish();

      /* Display result */
      for (int i = 0; i < numberOfFloats; i++)
      {
         System.out.println("hostMemoryPointer[" + i + "] : " + hostMemoryPointer.get(i));
      }
      openCLManager.destroy();
   }

   @Tag("gui")
   @Test
   public void testOpenCLImageManipulation()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      openCLManager.create();
      _cl_kernel kernel = openCLManager.loadSingleFunctionProgramAndCreateKernel("ManipulateImage");
      int imageWidth = 6;
      int imageHeight = 4;
      int cvMatType = opencv_core.CV_32FC1;
      int bytesPerPixel = 4;
      int sizeInBytes = bytesPerPixel * imageHeight * imageWidth;
      ByteBuffer backingDirectByteBuffer = ByteBuffer.allocateDirect(sizeInBytes);
      backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      BytePointer hostMemoryPointer = new BytePointer(backingDirectByteBuffer);
      Mat openCVMat = new Mat(imageHeight, imageWidth, cvMatType, hostMemoryPointer);

//      hostMemoryPointer = null;
      _cl_mem image = openCLManager.createImage(OpenCL.CL_MEM_READ_WRITE,
                                                OpenCL.CL_R,
                                                OpenCL.CL_FLOAT,
                                                imageWidth,
                                                imageHeight,
                                                hostMemoryPointer);
      openCLManager.enqueueWriteImage(image, imageWidth, imageHeight, hostMemoryPointer);
      openCLManager.setKernelArgument(kernel, 0, image);
      openCLManager.execute2D(kernel, imageWidth, imageHeight);
      openCLManager.flush();
      openCLManager.enqueueReadImage(image, imageWidth, imageHeight, hostMemoryPointer);
      openCLManager.finish();

      /* Display result */
      for (int x = 0; x < imageWidth; x++)
      {
         for (int y = 0; y < imageHeight; y++)
         {
            System.out.println("hostMemoryPointer[" + x + "] : " + openCVMat.ptr(y, x).getFloat());
         }
      }
      openCLManager.destroy();
   }

   @Tag("gui")
   @Test
   public void testOpenCLImageManipulation2()
   {
      final int maxNumberOfEntries = 2; // More than 2 results in native crash TODO: Why?
      _cl_platform_id platforms = new _cl_platform_id();
      _cl_device_id devices = new _cl_device_id();
      _cl_context context;
      _cl_command_queue commandQueue = new _cl_command_queue();
      IntPointer numberOfDevices = new IntPointer(1);
      IntPointer numberOfPlatforms = new IntPointer(3);
      IntPointer returnCode = new IntPointer(1);
      clGetPlatformIDs(maxNumberOfEntries, platforms, numberOfPlatforms);
      clGetDeviceIDs(platforms, CL_DEVICE_TYPE_ALL, maxNumberOfEntries, devices, numberOfDevices);
      context = clCreateContext(null, 1, devices, null, null, returnCode);
      int imageWidth = 6;
      int imageHeight = 4;
      int flags = OpenCL.CL_MEM_READ_WRITE;
      int imageChannelOrder = CL_R;
      int imageChannelDataType = CL_UNSIGNED_INT16;
//      cl_image_format imageFormat = new cl_image_format(new IntPointer(imageChannelOrder, imageChannelDataType));
      cl_image_format imageFormat = new cl_image_format();
      imageFormat.image_channel_order(imageChannelOrder);
      imageFormat.image_channel_data_type(imageChannelDataType);
      int rowPitch = 0;
      int imageDepth = 0;
      int imageArraySize = 0; // image array size
      int imageSlicePitch = 0;
      int numberOfMipmapLevels = 0;
      int numberOfMipmapSamples = 0;
      int memoryObject = 0;
//      cl_image_desc imageDescription = new cl_image_desc(new IntPointer(
//            CL_MEM_OBJECT_IMAGE2D,
//            imageWidth,
//            imageHeight,
//            imageDepth,
//            imageArraySize,
//            rowPitch,
//            imageSlicePitch,
//            numberOfMipmapLevels,
//            numberOfMipmapSamples,
//            memoryObject
//      ));
      cl_image_desc imageDescription = new cl_image_desc();
      imageDescription.image_type(CL_MEM_OBJECT_IMAGE2D);
      imageDescription.image_width(imageWidth);
      imageDescription.image_height(imageHeight);
//      imageDescription.position(0).put(new cl_mem);
//      imageDescription.position(1).put(new IntPointer(new int[] {width}));
//      imageDescription.position(2).put(new IntPointer(new int[] {height}));
//      imageDescription.position(3).put(new IntPointer(new int[] {0})); // depth
//      imageDescription.position(4).put(new IntPointer(new int[] {0})); // arraySize
//      imageDescription.position(5).put(new IntPointer(new int[] {rowPitch}));
//      imageDescription.position(6).put(new IntPointer(new int[] {0})); // slicePitch
//      imageDescription.position(7).put(new IntPointer(new int[] {0})); // number of mipmap levels
//      imageDescription.position(8).put(new IntPointer(new int[] {0})); // number of samples
//      imageDescription.position(9).put(new IntPointer(new int[] {0})); // mem_object
      Pointer hostPointer = null;
      _cl_mem image = clCreateImage(context, flags, imageFormat, imageDescription, hostPointer, returnCode);
      System.out.println("code: " + returnCode.get() + " image: " + image);
   }
}
