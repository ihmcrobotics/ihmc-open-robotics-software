package us.ihmc.robotDataLogger.websocket.server;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.UnpooledByteBufAllocator;
import io.netty.buffer.UnpooledUnsafeDirectByteBuf;


/**
 * 
 * Netty ByteBuf that allows changing the internal capacity without re-allocating the bytebuffer.
 * 
 * Used by the {@link RecyclingByteBufAllocator}
 * 
 * @author Jesper Smith
 *
 */
class ResizeableUnpooledUnsafeDirectByteBuf extends UnpooledUnsafeDirectByteBuf
{

   private int internalCapacity;

   public ResizeableUnpooledUnsafeDirectByteBuf(int initialCapacity, int maxCapacity)
   {
      super(UnpooledByteBufAllocator.DEFAULT, initialCapacity, maxCapacity);
      this.internalCapacity = initialCapacity;
   }

   @Override
   public int capacity()
   {
      return this.internalCapacity;
   }

   @Override
   public ByteBuf capacity(int newCapacity)
   {
      if (newCapacity < maxCapacity())
      {
         this.internalCapacity = newCapacity;
      }
      else
      {
         throw new IllegalArgumentException("newCapacity is larger than maxCapacity");
      }
      
      return this;
   }
}
