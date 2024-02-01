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
import us.ihmc.perception.opencl.OpenCLManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static org.bytedeco.opencl.global.OpenCL.*;

public class OpenCLManagerTest
{
   @Tag("opencl")
   @Test
   public void testLoadingAndPrintingParameters()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      openCLManager.destroy();
   }

   @Tag("opencl")
   @Test
   public void testOpenCLContext()
   {
      OpenCLManager openCLManager = new OpenCLManager();
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

      /* Display result */
      for (int i = 0; i < numberOfFloats; i++)
      {
         System.out.println("hostMemoryPointer[" + i + "] : " + hostMemoryPointer.get(i));
      }
      openCLManager.destroy();
   }

   @Tag("opencl")
   @Test
   public void testOpenCLImageManipulation()
   {
      OpenCLManager openCLManager = new OpenCLManager();
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

      for (int x = 0; x < imageWidth; x++)
      {
         openCVMat.ptr(2, x).putFloat(10.0f);
      }

      _cl_mem image = openCLManager.createImage(OpenCL.CL_MEM_READ_WRITE,
                                                OpenCL.CL_R,
                                                OpenCL.CL_FLOAT,
                                                imageWidth,
                                                imageHeight,
                                                hostMemoryPointer);
//      openCLManager.enqueueWriteImage(image, imageWidth, imageHeight, hostMemoryPointer); // Not actually required
      openCLManager.setKernelArgument(kernel, 0, image);
      openCLManager.execute2D(kernel, imageWidth, imageHeight);
      openCLManager.enqueueReadImage(image, imageWidth, imageHeight, hostMemoryPointer);

      /* Display result */
      for (int x = 0; x < imageWidth; x++)
      {
         for (int y = 0; y < imageHeight; y++)
         {
            System.out.println("image[" + x + ", " + y + "] : " + openCVMat.ptr(y, x).getFloat());
         }
      }
      openCLManager.destroy();
   }

   @Tag("opencl")
   @Test
   public void testOpenCLImageManipulation2()
   {
      _cl_platform_id platforms = new _cl_platform_id();
      _cl_device_id devices = new _cl_device_id();
      _cl_context context;
      IntPointer numberOfDevices = new IntPointer(1);
      IntPointer numberOfPlatforms = new IntPointer(3);
      IntPointer returnCode = new IntPointer(1);
      final int platformCount = 1; // We're just interested in the primary platform (most likely "NVIDIA CUDA")
      clGetPlatformIDs(platformCount, platforms, numberOfPlatforms);
      clGetDeviceIDs(platforms, CL_DEVICE_TYPE_ALL, platformCount, devices, numberOfDevices);
      context = clCreateContext(null, 1, devices, null, null, returnCode);
      int imageWidth = 6;
      int imageHeight = 4;
      int flags = OpenCL.CL_MEM_READ_WRITE;
      int imageChannelOrder = CL_R;
      int imageChannelDataType = CL_UNSIGNED_INT16;
      cl_image_format imageFormat = new cl_image_format();
      imageFormat.image_channel_order(imageChannelOrder);
      imageFormat.image_channel_data_type(imageChannelDataType);
      cl_image_desc imageDescription = new cl_image_desc();
      imageDescription.image_type(CL_MEM_OBJECT_IMAGE2D);
      imageDescription.image_width(imageWidth);
      imageDescription.image_height(imageHeight);
      Pointer hostPointer = null;
      _cl_mem image = clCreateImage(context, flags, imageFormat, imageDescription, hostPointer, returnCode);
      System.out.println("code: " + returnCode.get() + " image: " + image);
   }
}
