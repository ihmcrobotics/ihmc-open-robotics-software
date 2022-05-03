package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;

/**
 * A pointer that can have its address modified after instantiation.
 */
public class MutableBytePointer extends BytePointer
{
   public MutableBytePointer(Pointer p)
   {
      super(p);
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
}