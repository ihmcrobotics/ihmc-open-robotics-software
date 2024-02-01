package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_mem;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.tools.NativeMemoryTools;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

public class OpenCLRigidBodyTransformParameter
{
   public static final int NUMBER_OF_ELEMENTS = 12;
   private final ByteBuffer backingDirectByteBuffer;
   private final FloatBuffer backingDirectFloatBuffer;
   private final FloatPointer bytedecoFloatBufferPointer;
   private _cl_mem openCLBufferObject;

   public OpenCLRigidBodyTransformParameter()
   {
      backingDirectByteBuffer = NativeMemoryTools.allocate(NUMBER_OF_ELEMENTS * Float.BYTES);
      backingDirectFloatBuffer = backingDirectByteBuffer.asFloatBuffer();
      bytedecoFloatBufferPointer = new FloatPointer(backingDirectFloatBuffer);
   }

   public void setParameter(RigidBodyTransform rigidBodyTransform)
   {
      setParameter(rigidBodyTransform.getTranslation(), rigidBodyTransform.getRotation());
   }

   public void setParameter(Tuple3DReadOnly translation, RotationMatrixReadOnly rotationMatrix)
   {
      backingDirectFloatBuffer.rewind();
      backingDirectFloatBuffer.put(translation.getX32());
      backingDirectFloatBuffer.put(translation.getY32());
      backingDirectFloatBuffer.put(translation.getZ32());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM00());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM01());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM02());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM10());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM11());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM12());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM20());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM21());
      backingDirectFloatBuffer.put((float) rotationMatrix.getM22());
   }

   public void getResult(RigidBodyTransform rigidBodyTransformToPack)
   {
      backingDirectFloatBuffer.rewind();
      rigidBodyTransformToPack.getTranslation().set(backingDirectFloatBuffer.get(),
                                                    backingDirectFloatBuffer.get(),
                                                    backingDirectFloatBuffer.get());
      rigidBodyTransformToPack.getRotation().set(backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get(),
                                                 backingDirectFloatBuffer.get());
   }

   public void writeOpenCLBufferObject(OpenCLManager openCLManager)
   {
      ensureOpenCLObjectCreated(openCLManager);
      openCLManager.enqueueWriteBuffer(openCLBufferObject, NUMBER_OF_ELEMENTS * Float.BYTES, bytedecoFloatBufferPointer);
   }

   public void readOpenCLBufferObject(OpenCLManager openCLManager)
   {
      ensureOpenCLObjectCreated(openCLManager);
      openCLManager.enqueueReadBuffer(openCLBufferObject, NUMBER_OF_ELEMENTS * Float.BYTES, bytedecoFloatBufferPointer);
   }

   private void ensureOpenCLObjectCreated(OpenCLManager openCLManager)
   {
      if (openCLBufferObject == null)
         openCLBufferObject = openCLManager.createBufferObject(NUMBER_OF_ELEMENTS * Float.BYTES, bytedecoFloatBufferPointer);
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLBufferObject;
   }
}
