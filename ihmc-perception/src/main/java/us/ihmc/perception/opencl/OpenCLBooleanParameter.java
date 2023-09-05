package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.IntPointer;

/**
 * You can't pass booleans to OpenCL kernels, so we use an int.
 * @deprecated This isn't working!
 */
public class OpenCLBooleanParameter
{
   private final IntPointer intPointer = new IntPointer(1);

   public void setParameter(boolean value)
   {
      intPointer.put(value ? 1 : 0);
   }

   public IntPointer getIntPointer()
   {
      return intPointer;
   }
}
