package us.ihmc.perception;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

public class OpenCLManagerTest
{
   @Tag("gui")
   @Test
   public void testOpenCLContext()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      openCLManager.create();
      _cl_kernel kernel = openCLManager.loadProgramAndCreateKernel("vectorAddition");
      long numberOfFloats = 128;
      long sizeInBytes = numberOfFloats * Loader.sizeof(FloatPointer.class);
      FloatPointer hostMemoryPointer = new FloatPointer(numberOfFloats);
      for (int i = 0; i < numberOfFloats; i++)
      {
         hostMemoryPointer.put(i, i);
      }
      _cl_mem bufferObject = openCLManager.createBufferObject(sizeInBytes);
      openCLManager.enqueueWriteBuffer(bufferObject, sizeInBytes, hostMemoryPointer);
      openCLManager.setKernelArgument(kernel, 0, bufferObject);
      openCLManager.execute(kernel, numberOfFloats);
      openCLManager.enqueueReadBuffer(bufferObject, sizeInBytes, hostMemoryPointer);

      /* Display result */
      for (int i = 0; i < numberOfFloats; i++)
      {
         System.out.println("hostMemoryPointer[" + i + "] : " + hostMemoryPointer.get(i));
      }
   }
}
