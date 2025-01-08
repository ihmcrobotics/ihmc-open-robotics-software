package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.junit.jupiter.api.Test;

public class OpenCLKernelsSimpleTest
{
   /**
    * This test makes sure that we can pass a value to the GPU from the OpenCL kernel code.
    * The goal here is to create a couple values, and pass those values to the GPU.
    * Then have the GPU print those values, return successfully, and shutdown!
    */

   // Don't need to run this on CI, its good sanity check when running locally that things are working
   @Test
   public void passIntToGPUKernelExample()
   {
      // Overhead to getting a OpenCL kernel up and running
      OpenCLManager openCLManager = new OpenCLManager();
      _cl_program openCLProgram = openCLManager.loadProgram("OpenCLKernelsSimpleTest");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "text_parameter_gets_passed_in");

      // Create Java variables to be passed to the GPU
      int size = 2;
      int[] testInt = {4, 8};
      int intBytes = testInt.length * Integer.BYTES;

      IntPointer indexBuffer = new IntPointer(size);
      indexBuffer.put(0, testInt[0]);
      indexBuffer.put(1, testInt[1]);

      // Set kernel arguments
      _cl_mem bufferObject = openCLManager.createBufferObject(intBytes, indexBuffer);
      openCLManager.enqueueWriteBuffer(bufferObject, intBytes, indexBuffer);
      openCLManager.setKernelArgument(openCLKernel, 0, bufferObject);

      // Execute kernel with the number of threads that is needed to do the kernel all at once
      openCLManager.execute1D(openCLKernel, size);

      openCLProgram.close();
      openCLManager.destroy();
   }
}