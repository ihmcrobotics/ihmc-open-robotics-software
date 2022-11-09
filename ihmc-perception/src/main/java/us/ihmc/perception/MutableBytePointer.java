package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;

import java.nio.ByteBuffer;

/**
 * A pointer that can have its address modified after instantiation.
 */
public class MutableBytePointer extends BytePointer
{
   public MutableBytePointer()
   {
      super();
   }

   public MutableBytePointer(Pointer p)
   {
      super(p);
   }

   public MutableBytePointer(ByteBuffer buffer)
   {
      super(buffer);
   }

   /**
    * wrapper method to set a MutableBytePointer
    * to a given address, limit, and capacity
    * @param source the ByteBuffer where the MutableBytePointer
    *               will set its address, limit and capacity
    */
   public void wrapMutableBytePointer(ByteBuffer source)
   {
      this.address = getByteBufferAddress(source);
      this.limit = source.limit();
      this.capacity = source.capacity();
   }

   public long getByteBufferAddress(ByteBuffer source)
   {
      return getDirectBufferAddress(source);
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