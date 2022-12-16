package us.ihmc.perception.opencl;

import gnu.trove.list.array.TFloatArrayList;
import org.bytedeco.opencl._cl_mem;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

/**
 * Allocated the OpenCL buffer on the first time before it's written to the GPU.
 * This is done so we can automatically allocate the correct size. Before, the size had
 * to be adjusted whenever a parameter was added or removed, which results in really
 * hard to find bugs if messed up.
 */
public class OpenCLFloatParameters
{
   private boolean bufferCreated = false;
   private int parameterIndex = 0;
   private final TFloatArrayList initialParameterStorage = new TFloatArrayList();
   private OpenCLFloatBuffer openCLFloatBuffer;

   public void setParameter(float parameter)
   {
      if (!bufferCreated)
      {
         initialParameterStorage.add(parameter);
      }
      else
      {
         openCLFloatBuffer.getBytedecoFloatBufferPointer().put(parameterIndex, parameter);
      }

      ++parameterIndex;
   }

   public void writeOpenCLBufferObject(OpenCLManager openCLManager)
   {
      if (!bufferCreated)
      {
         bufferCreated = true;

         openCLFloatBuffer = new OpenCLFloatBuffer(parameterIndex);
         openCLFloatBuffer.createOpenCLBufferObject(openCLManager);

         // Copy initially passed parameters into the newly allocated buffer
         for (int i = 0; i < initialParameterStorage.size(); i++)
         {
            openCLFloatBuffer.getBytedecoFloatBufferPointer().put(i, initialParameterStorage.get(i));
         }
      }

      openCLFloatBuffer.writeOpenCLBufferObject(openCLManager);

      parameterIndex = 0;
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLFloatBuffer.getOpenCLBufferObject();
   }
}
