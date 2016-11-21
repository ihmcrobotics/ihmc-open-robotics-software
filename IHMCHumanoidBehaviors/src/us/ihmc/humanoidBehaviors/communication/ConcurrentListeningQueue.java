package us.ihmc.humanoidBehaviors.communication;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.tools.io.printing.PrintTools;

public class ConcurrentListeningQueue<T>
{
   private final boolean DEBUG = false;

   private final ConcurrentLinkedQueue<T> packetQueue = new ConcurrentLinkedQueue<T>();
   private T lastPacket = null;

   private AtomicInteger size = new AtomicInteger(0);

   private int maximumSizeToBuffer;
   private final StackTraceElement[] elementsOnCreation;

   private boolean printedOverflowWarning = false;

   public ConcurrentListeningQueue(int maximumSizeToBuffer)
   {
      elementsOnCreation = Thread.currentThread().getStackTrace();
      this.maximumSizeToBuffer = maximumSizeToBuffer;
   }

   /**
    * Sets the maximum size to buffer. If items are added to the buffer after it reaches this size, it throws away old ones and keeps the new ones.
    * @param maximumSizeToBuffer
    */
   public void setMaximumSizeToBuffer(int maximumSizeToBuffer)
   {
      this.maximumSizeToBuffer = maximumSizeToBuffer;
   }

   public int getMaximumSizeToBuffer()
   {
      return maximumSizeToBuffer;
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }

   public T getLatestPacket()
   {
      while (isNewPacketAvailable())
      {
         poll();
      }

      return lastPacket;
   }

   public T poll()
   {
      T polledPacket = packetQueue.poll();

      if (polledPacket != null)
      {
         size.decrementAndGet();
      }

      lastPacket = polledPacket;
      return polledPacket;
   }

   public void put(T object)
   {
      if (size.get() > maximumSizeToBuffer)
      {
         poll();
         if (!printedOverflowWarning)
         {
            printedOverflowWarning = true;
            if (DEBUG)
               printOverflowWarning();
         }
      }

      packetQueue.add(object);
      size.incrementAndGet();
   }

   private void printOverflowWarning()
   {
      PrintTools.warn("Filling " + getClass().getSimpleName() + " without emptying it. Stack trace at creation:");

      for (int i = 1; i < elementsOnCreation.length; i++)
      {
         StackTraceElement s = elementsOnCreation[i];
         PrintTools.warn("\tat " + s.getClassName() + "." + s.getMethodName() + "(" + s.getFileName() + ":" + s.getLineNumber() + ")");
      }
   }

   public void clear()
   {
      packetQueue.clear();
      size.set(0);
   }
}
