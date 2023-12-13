package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_mem;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;

/**
 * To use this, you need to make sure to use native byte order on the ByteBuffers.
 */
public class OpenCLIntBuffer
{
   private long numberOfIntegers;
   private ByteBuffer backingDirectByteBuffer;
   private IntBuffer backingDirectIntBuffer;
   private IntPointer bytedecoIntBufferPointer;
   private _cl_mem openCLBufferObject;

   public OpenCLIntBuffer(int numberOfIntegers)
   {
      this(numberOfIntegers, null);
   }

   public OpenCLIntBuffer(ByteBuffer backingDirectByteBuffer)
   {
      this.backingDirectByteBuffer = backingDirectByteBuffer;
      IntBuffer backingDirectIntBuffer = backingDirectByteBuffer.asIntBuffer();
      resize(backingDirectIntBuffer.capacity(), null, backingDirectIntBuffer);
   }

   public OpenCLIntBuffer(int numberOfIntegers, IntBuffer backingDirectIntBuffer)
   {
      resize(numberOfIntegers, null, backingDirectIntBuffer);
   }

   public void resize(int numberOfIntegers, OpenCLManager openCLManager)
   {
      resize(numberOfIntegers, openCLManager, null);
   }

   public void destroy(OpenCLManager openCLManager)
   {
      if (openCLBufferObject != null)
      {
         openCLManager.releaseBufferObject(openCLBufferObject);
         openCLBufferObject.releaseReference();
         openCLBufferObject = null;
      }
   }

   public void resize(int numberOfIntegers, OpenCLManager openCLManager, IntBuffer backingDirectIntBuffer)
   {
      this.numberOfIntegers = numberOfIntegers;

      boolean openCLObjectCreated = openCLBufferObject != null;
      destroy(openCLManager); // TODO: Is this breaking things? Possible bug.

      if (backingDirectIntBuffer == null)
      {
         backingDirectByteBuffer = ByteBuffer.allocateDirect(numberOfIntegers * Integer.BYTES);
         backingDirectByteBuffer.order(ByteOrder.nativeOrder());
         this.backingDirectIntBuffer = backingDirectByteBuffer.asIntBuffer();
      }
      else
      {
         this.backingDirectIntBuffer = backingDirectIntBuffer;
      }

      bytedecoIntBufferPointer = new IntPointer(this.backingDirectIntBuffer);

      if (openCLObjectCreated)
      {
         createOpenCLBufferObject(openCLManager);
      }
   }

   public void createOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLBufferObject = openCLManager.createBufferObject(numberOfIntegers * Integer.BYTES, bytedecoIntBufferPointer);
   }

   public void writeOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLManager.enqueueWriteBuffer(openCLBufferObject, numberOfIntegers * Integer.BYTES, bytedecoIntBufferPointer);
   }

   public void readOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLManager.enqueueReadBuffer(openCLBufferObject, numberOfIntegers * Integer.BYTES, bytedecoIntBufferPointer);
   }

   public ByteBuffer getBackingDirectByteBuffer()
   {
      return backingDirectByteBuffer;
   }

   public IntBuffer getBackingDirectIntBuffer()
   {
      return backingDirectIntBuffer;
   }

   public IntPointer getBytedecoIntBufferPointer()
   {
      return bytedecoIntBufferPointer;
   }

   public long getNumberOfIntegers()
   {
      return numberOfIntegers;
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLBufferObject;
   }
}
