package us.ihmc.robotDataLogger.websocket.server;

import java.nio.ByteBuffer;
import java.util.ArrayList;

import io.netty.buffer.AbstractByteBufAllocator;
import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;

public class CustomGCAvoidingByteBufAllocator extends AbstractByteBufAllocator
{
   private final static int INITIAL_MAX_CAPACITY = 128;
   private final static int INITIAL_POOL_SIZE = 16;

   private final ArrayList<ByteBuf> pool = new ArrayList<ByteBuf>(INITIAL_POOL_SIZE);

   private int capacity = INITIAL_MAX_CAPACITY;

   private int index = 0;

   public CustomGCAvoidingByteBufAllocator()
   {
      super(true);
      refill(INITIAL_MAX_CAPACITY);
   }

   private ByteBuf add(int capacity)
   {
      ByteBuf wrappedBuffer = Unpooled.wrappedBuffer(ByteBuffer.allocateDirect(capacity));
      wrappedBuffer.internalNioBuffer(0, 0); // Force allocation of temporary object
      pool.add(wrappedBuffer);
      return wrappedBuffer;
   }

   private void refill(int capacity)
   {
      pool.clear();
      for (int i = 0; i < INITIAL_POOL_SIZE; i++)
      {
         add(capacity);
      }
      this.capacity = capacity;
   }

   @Override
   public boolean isDirectBufferPooled()
   {
      return false;
   }

   @Override
   protected ByteBuf newHeapBuffer(int initialCapacity, int maxCapacity)
   {
      throw new RuntimeException("Heap buffers unsupported by this allocator");
   }

   @Override
   protected ByteBuf newDirectBuffer(int initialCapacity, int maxCapacity)
   {
      try
      {
         if (initialCapacity > capacity)
         {
            System.err.println("Requested capacity " + initialCapacity + " exceeds " + capacity);
            Thread.dumpStack();
            refill(initialCapacity);
         }

         for (int i = index; i < index + pool.size(); i++)
         {
            int item = i < pool.size() ? i : i - pool.size();

            ByteBuf byteBuf = pool.get(item);
            if (byteBuf.refCnt() == 1)
            {
               index = item + 1;
               System.out.println(initialCapacity);
               byteBuf.clear();
               byteBuf.internalNioBuffer(0, 0); // Force allocation of temporary object
               return byteBuf.retain();
            }
         }

         System.err.println("Pool overflowed, adding item");
         return add(capacity);
      }
      catch (Exception e)
      {
         e.printStackTrace();
         throw e;
      }
   }

}
