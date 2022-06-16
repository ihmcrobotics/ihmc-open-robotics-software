package us.ihmc.perception;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_mem;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class OpenCLFloatBuffer
{
   private long numberOfFloats;
   private ByteBuffer backingDirectByteBuffer;
   private FloatBuffer backingDirectFloatBuffer;
   private FloatPointer bytedecoFloatBufferPointer;
   private _cl_mem openCLBufferObject;

   public OpenCLFloatBuffer(int numberOfFloats)
   {
      this(numberOfFloats, null);
   }

   public OpenCLFloatBuffer(int numberOfFloats, FloatBuffer backingDirectFloatBuffer)
   {
      resize(numberOfFloats, null, backingDirectFloatBuffer);
   }

   public void resize(int numberOfFloats, OpenCLManager openCLManager)
   {
      resize(numberOfFloats, openCLManager, null);
   }

   public void destroy(OpenCLManager openCLManager)
   {
      if (openCLBufferObject != null)
      {
         openCLManager.releaseBufferObject(openCLBufferObject);
         openCLBufferObject.releaseReference();
      }
   }

   public void resize(int numberOfFloats,  OpenCLManager openCLManager, FloatBuffer backingDirectFloatBuffer)
   {
      this.numberOfFloats = numberOfFloats;

      boolean openCLObjectCreated = openCLBufferObject != null;
      if (openCLObjectCreated)
      {
         openCLManager.releaseBufferObject(openCLBufferObject);
      }

      if (backingDirectFloatBuffer == null)
      {
         backingDirectByteBuffer = ByteBuffer.allocateDirect(numberOfFloats * Float.BYTES);
         backingDirectByteBuffer.order(ByteOrder.nativeOrder());
         this.backingDirectFloatBuffer = backingDirectByteBuffer.asFloatBuffer();
      }
      else
      {
         this.backingDirectFloatBuffer = backingDirectFloatBuffer;
      }

      bytedecoFloatBufferPointer = new FloatPointer(this.backingDirectFloatBuffer);

      if (openCLObjectCreated)
      {
         createOpenCLBufferObject(openCLManager);
      }
   }

   public void createOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLBufferObject = openCLManager.createBufferObject(numberOfFloats * Float.BYTES, bytedecoFloatBufferPointer);
   }

   public void writeOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLManager.enqueueWriteBuffer(openCLBufferObject, numberOfFloats * Float.BYTES, bytedecoFloatBufferPointer);
   }

   public void readOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLManager.enqueueReadBuffer(openCLBufferObject, numberOfFloats * Float.BYTES, bytedecoFloatBufferPointer);
   }

   public FloatBuffer getBackingDirectFloatBuffer()
   {
      return backingDirectFloatBuffer;
   }

   public FloatPointer getBytedecoFloatBufferPointer()
   {
      return bytedecoFloatBufferPointer;
   }

   public long getNumberOfFloats()
   {
      return numberOfFloats;
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLBufferObject;
   }
}
