package org.bytedeco.javacpp;
// This is a hack to allow setting the address of a Pointer after it's been created.
// Addesses are package private, so this package needs to be defined to match

/**
 * A pointer that can have it's address modified after instantiation.
 */
public class MutableBytePointer extends BytePointer 
{
   public MutableBytePointer(final Pointer p) 
   {
      address = p.address;
      position = p.position;
      limit = p.limit;
      capacity = p.capacity;
      
      if(p.deallocator() != null)
      {
         deallocator(new ProxyDeallocator(this, p));
      }
   }
   
   /**
    * change the address this pointer is pointing to
    * Doing this wrong will crash the application
    * @param address the new memory address
    */
   public void setAddress(long address)
   {
      this.address = address;
   }
}