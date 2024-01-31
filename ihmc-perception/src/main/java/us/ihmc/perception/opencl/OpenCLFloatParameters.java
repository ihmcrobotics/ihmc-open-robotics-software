package us.ihmc.perception.opencl;

import gnu.trove.list.array.TFloatArrayList;
import org.bytedeco.opencl._cl_mem;
import us.ihmc.euclid.transform.RigidBodyTransform;

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

   /**
    * Sets 12 float parameters for a RigidBodyTransform.
    */
   public void setParameter(RigidBodyTransform rigidBodyTransform)
   {
      setParameter(rigidBodyTransform.getTranslation().getX32());
      setParameter(rigidBodyTransform.getTranslation().getY32());
      setParameter(rigidBodyTransform.getTranslation().getZ32());
      setParameter((float) rigidBodyTransform.getRotation().getM00());
      setParameter((float) rigidBodyTransform.getRotation().getM01());
      setParameter((float) rigidBodyTransform.getRotation().getM02());
      setParameter((float) rigidBodyTransform.getRotation().getM10());
      setParameter((float) rigidBodyTransform.getRotation().getM11());
      setParameter((float) rigidBodyTransform.getRotation().getM12());
      setParameter((float) rigidBodyTransform.getRotation().getM20());
      setParameter((float) rigidBodyTransform.getRotation().getM21());
      setParameter((float) rigidBodyTransform.getRotation().getM22());
   }

   public void setParameter(boolean booleanAsFloat)
   {
      setParameter(booleanAsFloat ? 1.0f : 0.0f);
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
