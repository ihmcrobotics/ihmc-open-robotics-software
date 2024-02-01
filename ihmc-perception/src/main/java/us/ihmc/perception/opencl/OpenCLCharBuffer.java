package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_mem;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * To use this, you need to make sure to use native byte order on the ByteBuffers.
 * @deprecated TODO: I made this but I'm not sure if it works.
 */
public class OpenCLCharBuffer
{
   private long numberOfChars;
   private ByteBuffer backingDirectByteBuffer;
   private BytePointer bytedecoByteBufferPointer;
   private _cl_mem openCLBufferObject;

   public OpenCLCharBuffer(int numberOfChars)
   {
      this(numberOfChars, null);
   }

   public OpenCLCharBuffer(ByteBuffer backingDirectByteBuffer)
   {
      this.backingDirectByteBuffer = backingDirectByteBuffer;
      resize(backingDirectByteBuffer.capacity(), null, backingDirectByteBuffer);
   }

   public OpenCLCharBuffer(int numberOfChars, ByteBuffer backingDirectByteBuffer)
   {
      resize(numberOfChars, null, backingDirectByteBuffer);
   }

   public void resize(int numberOfChars, OpenCLManager openCLManager)
   {
      resize(numberOfChars, openCLManager, null);
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

   public void resize(int numberOfChars, OpenCLManager openCLManager, ByteBuffer backingDirectByteBuffer)
   {
      this.numberOfChars = numberOfChars;

      boolean openCLObjectCreated = openCLBufferObject != null;
      destroy(openCLManager); // TODO: Is this breaking things? Possible bug.

      if (backingDirectByteBuffer == null)
      {
         this.backingDirectByteBuffer = ByteBuffer.allocateDirect(numberOfChars);
         this.backingDirectByteBuffer.order(ByteOrder.nativeOrder());
      }
      else
      {
         this.backingDirectByteBuffer = backingDirectByteBuffer;
      }

      bytedecoByteBufferPointer = new BytePointer(this.backingDirectByteBuffer);

      if (openCLObjectCreated)
      {
         createOpenCLBufferObject(openCLManager);
      }
   }

   public void createOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLBufferObject = openCLManager.createBufferObject(numberOfChars, bytedecoByteBufferPointer);
   }

   public void writeOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLManager.enqueueWriteBuffer(openCLBufferObject, numberOfChars, bytedecoByteBufferPointer);
   }

   public void readOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLManager.enqueueReadBuffer(openCLBufferObject, numberOfChars, bytedecoByteBufferPointer);
   }

   public ByteBuffer getBackingDirectByteBuffer()
   {
      return backingDirectByteBuffer;
   }

   public BytePointer getBytedecoByteBufferPointer()
   {
      return bytedecoByteBufferPointer;
   }

   public long getNumberOfChars()
   {
      return numberOfChars;
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLBufferObject;
   }
}
