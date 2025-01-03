package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.jcodec.common.Assert;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencl.OpenCLManager;

public class HeightMapUtilsTest
{
   private final int floatBytes = Float.BYTES;
   private final int intBytes = Integer.BYTES;

   @Test
   public void testIndexToCoordinateOpenCL()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      _cl_program openCLProgram = openCLManager.loadProgram("HeightMapUtilsTest", "HeightMapUtils.cl");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "test_indices_to_coordinate");

      // This creates the (global int* index) parameter used in the kernel.
      // Then this adds the data inside indexPointer to be the first argument for the kernel
      int indexIntSize = 1;
      int[] indexInt = {5};
      IntPointer indexPointer = new IntPointer(indexIntSize);
      indexPointer.put(0, indexInt[0]);
      _cl_mem indexPointerObject = openCLManager.createBufferObject(intBytes, indexPointer);
      openCLManager.enqueueWriteBuffer(indexPointerObject, intBytes , indexPointer);
      openCLManager.setKernelArgument(openCLKernel, 0, indexPointerObject);

      // This creates the (global float* center) parameter used in the kernel.
      // Then this adds the data inside centerPointer to be the first argument for the kernel
      int centerFloatSize = 1;
      float[] centerFloat = {0.0f};
      FloatPointer centerPointer = new FloatPointer(centerFloatSize);
      centerPointer.put(0, centerFloat[0]);
      _cl_mem centerPointerObject = openCLManager.createBufferObject(floatBytes, centerPointer);
      openCLManager.enqueueWriteBuffer(centerPointerObject, floatBytes, centerPointer);
      openCLManager.setKernelArgument(openCLKernel, 1, centerPointerObject);

      FloatPointer resolutionPointer = new FloatPointer(1);
      resolutionPointer.put(0, 1.0f);
      _cl_mem resolutionPointerObject = openCLManager.createBufferObject(floatBytes, resolutionPointer);
      openCLManager.enqueueWriteBuffer(resolutionPointerObject, floatBytes, resolutionPointer);
      openCLManager.setKernelArgument(openCLKernel, 2, resolutionPointerObject);

      IntPointer centerIndexPointer = new IntPointer(1);
      centerIndexPointer.put(0, 5);
      _cl_mem centerIndexPointerObject = openCLManager.createBufferObject(intBytes, centerIndexPointer);
      openCLManager.enqueueWriteBuffer(centerIndexPointerObject, intBytes, centerIndexPointer);
      openCLManager.setKernelArgument(openCLKernel, 3, centerIndexPointerObject);

      FloatPointer xHostPointer = new FloatPointer(1);
      _cl_mem xHostPointerObject = openCLManager.createBufferObject(floatBytes, xHostPointer);
      openCLManager.enqueueWriteBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 4, xHostPointerObject);

      FloatPointer yHostPointer = new FloatPointer(1);
      _cl_mem yHostPointerObject = openCLManager.createBufferObject(floatBytes, yHostPointer);
      openCLManager.enqueueWriteBuffer(yHostPointerObject, floatBytes, yHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 5, yHostPointerObject);

      openCLManager.execute1D(openCLKernel, 1);

      openCLManager.enqueueReadBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.enqueueReadBuffer(yHostPointerObject, floatBytes, yHostPointer);


      float expectedXValueFromKernel = (indexPointer.get(0) - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);
      int yIndexValue = 1;// Represents the y value in indexForConversion
      float expectedYValueFromKernel = (yIndexValue - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);

      Assert.assertEquals((int) expectedXValueFromKernel, (int) xHostPointer.get(0));
      Assert.assertEquals((int) expectedYValueFromKernel, (int) yHostPointer.get(0));

      LogTools.info(xHostPointer.get(0) + " is the X result!");
      LogTools.info(expectedXValueFromKernel + " is the X expected value!");
      LogTools.info(yHostPointer.get(0) + " is the Y result!");
      LogTools.info(expectedYValueFromKernel + " is the Y expected value!");

      openCLProgram.close();
      openCLManager.destroy();
   }

   @Test
   public void testIndexToCoordinateCUDA()
   {


      OpenCLManager openCLManager = new OpenCLManager();
      _cl_program openCLProgram = openCLManager.loadProgram("HeightMapUtilsTest", "HeightMapUtils.cl");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "test_indices_to_coordinate");

      // This creates the (global int* index) parameter used in the kernel.
      // Then this adds the data inside indexPointer to be the first argument for the kernel
      int indexIntSize = 1;
      int[] indexInt = {5};
      IntPointer indexPointer = new IntPointer(indexIntSize);
      indexPointer.put(0, indexInt[0]);
      _cl_mem indexPointerObject = openCLManager.createBufferObject(intBytes, indexPointer);
      openCLManager.enqueueWriteBuffer(indexPointerObject, intBytes , indexPointer);
      openCLManager.setKernelArgument(openCLKernel, 0, indexPointerObject);

      // This creates the (global float* center) parameter used in the kernel.
      // Then this adds the data inside centerPointer to be the first argument for the kernel
      int centerFloatSize = 1;
      float[] centerFloat = {0.0f};
      FloatPointer centerPointer = new FloatPointer(centerFloatSize);
      centerPointer.put(0, centerFloat[0]);
      _cl_mem centerPointerObject = openCLManager.createBufferObject(floatBytes, centerPointer);
      openCLManager.enqueueWriteBuffer(centerPointerObject, floatBytes, centerPointer);
      openCLManager.setKernelArgument(openCLKernel, 1, centerPointerObject);

      FloatPointer resolutionPointer = new FloatPointer(1);
      resolutionPointer.put(0, 1.0f);
      _cl_mem resolutionPointerObject = openCLManager.createBufferObject(floatBytes, resolutionPointer);
      openCLManager.enqueueWriteBuffer(resolutionPointerObject, floatBytes, resolutionPointer);
      openCLManager.setKernelArgument(openCLKernel, 2, resolutionPointerObject);

      IntPointer centerIndexPointer = new IntPointer(1);
      centerIndexPointer.put(0, 5);
      _cl_mem centerIndexPointerObject = openCLManager.createBufferObject(intBytes, centerIndexPointer);
      openCLManager.enqueueWriteBuffer(centerIndexPointerObject, intBytes, centerIndexPointer);
      openCLManager.setKernelArgument(openCLKernel, 3, centerIndexPointerObject);

      FloatPointer xHostPointer = new FloatPointer(1);
      _cl_mem xHostPointerObject = openCLManager.createBufferObject(floatBytes, xHostPointer);
      openCLManager.enqueueWriteBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 4, xHostPointerObject);

      FloatPointer yHostPointer = new FloatPointer(1);
      _cl_mem yHostPointerObject = openCLManager.createBufferObject(floatBytes, yHostPointer);
      openCLManager.enqueueWriteBuffer(yHostPointerObject, floatBytes, yHostPointer);
      openCLManager.setKernelArgument(openCLKernel, 5, yHostPointerObject);

      openCLManager.execute1D(openCLKernel, 1);

      openCLManager.enqueueReadBuffer(xHostPointerObject, floatBytes, xHostPointer);
      openCLManager.enqueueReadBuffer(yHostPointerObject, floatBytes, yHostPointer);


      float expectedXValueFromKernel = (indexPointer.get(0) - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);
      int yIndexValue = 1;// Represents the y value in indexForConversion
      float expectedYValueFromKernel = (yIndexValue - centerIndexPointer.get()) * resolutionPointer.get(0) + centerPointer.get(0);

      Assert.assertEquals((int) expectedXValueFromKernel, (int) xHostPointer.get(0));
      Assert.assertEquals((int) expectedYValueFromKernel, (int) yHostPointer.get(0));

      LogTools.info(xHostPointer.get(0) + " is the X result!");
      LogTools.info(expectedXValueFromKernel + " is the X expected value!");
      LogTools.info(yHostPointer.get(0) + " is the Y result!");
      LogTools.info(expectedYValueFromKernel + " is the Y expected value!");

      openCLProgram.close();
      openCLManager.destroy();
   }
}
