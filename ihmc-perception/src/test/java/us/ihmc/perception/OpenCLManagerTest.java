package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

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
      ByteBuffer backingDirectByteBuffer = ByteBuffer.allocateDirect(imageWidth * imageHeight * bytesPerPixel);
      backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      BytePointer hostMemoryPointer = new BytePointer(backingDirectByteBuffer);
      Mat openCVMat = new Mat(imageHeight, imageWidth, cvMatType, hostMemoryPointer);

      _cl_mem bufferObject = openCLManager.createImage(OpenCL.CL_MEM_READ_WRITE, imageWidth, imageHeight, hostMemoryPointer);
      openCLManager.enqueueWriteBuffer(bufferObject, sizeInBytes, hostMemoryPointer);
      openCLManager.setKernelArgument(kernel, 0, bufferObject);
      openCLManager.execute2D(kernel, imageWidth, imageHeight);
      openCLManager.enqueueReadBuffer(bufferObject, sizeInBytes, hostMemoryPointer);
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
}
