package us.ihmc.perception;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.ShortPointer;

import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

/**
 * A pointer that can have its address modified after instantiation.
 */
public class MutableShortPointer extends ShortPointer
{
   public MutableShortPointer()
   {
      super();
   }

   public MutableShortPointer(Pointer p)
   {
      super(p);
   }

   public MutableShortPointer(ShortBuffer buffer)
   {
      super(buffer);
   }

   /**
    * Wrapper method to set a MutableBytePointer
    * to a given address, limit, and capacity
    * @param byteBuffer the ByteBuffer where the MutableBytePointer
    *               will set its address, limit capacity and position
    */
   public void wrapByteBuffer(ByteBuffer byteBuffer)
   {
      this.address = getDirectBufferAddress(byteBuffer);
      this.limit = byteBuffer.limit();
      this.capacity = byteBuffer.capacity();
      this.position = byteBuffer.position();
   }

   /**
    * Change the address this pointer is pointing to
    * Doing this wrong will crash the application
    * @param address the new memory address
    */
   public void setAddress(long address)
   {
      this.address = address;
   }

   public void setLimit(long limit)
   {
      this.limit = limit;
   }

   public void setCapacity(long capacity)
   {
      this.capacity = capacity;
   }

   public void setPosition(long position)
   {
      this.position = position;
   }
}