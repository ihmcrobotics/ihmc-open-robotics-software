package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_mem;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.OpenCLManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class OpenCLRigidBodyTransformParameter
{
   public static final int NUMBER_OF_ELEMENTS = 12;
   private final ByteBuffer backingDirectByteBuffer;
   private final FloatBuffer backingDirectFloatBuffer;
   private final FloatPointer bytedecoFloatBufferPointer;
   private final float[] heapArray = new float[NUMBER_OF_ELEMENTS];
   private _cl_mem openCLBufferObject;

   public OpenCLRigidBodyTransformParameter()
   {
      backingDirectByteBuffer = ByteBuffer.allocateDirect(NUMBER_OF_ELEMENTS * Float.BYTES);
      backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      backingDirectFloatBuffer = backingDirectByteBuffer.asFloatBuffer();
      bytedecoFloatBufferPointer = new FloatPointer(this.backingDirectFloatBuffer);
   }

   public void set(RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.get(heapArray);
      backingDirectFloatBuffer.put(heapArray);
   }

   public void get(RigidBodyTransform rigidBodyTransformToPack)
   {
      backingDirectFloatBuffer.get(heapArray);
      rigidBodyTransformToPack.set(heapArray);
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
