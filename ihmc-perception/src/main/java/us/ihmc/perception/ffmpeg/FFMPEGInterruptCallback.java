package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVIOInterruptCB;
import org.bytedeco.javacpp.Pointer;

import java.util.concurrent.atomic.AtomicInteger;

public class FFMPEGInterruptCallback extends AVIOInterruptCB
{
   private final Callback_Pointer callbackPointer;
   private final AtomicInteger interruptFlag = new AtomicInteger(0);

   public FFMPEGInterruptCallback()
   {
      // Set up callback pointer
      callbackPointer = new Callback_Pointer()
      {
         @Override
         public int call(Pointer pointer)
         {
            return interruptFlag.getAndSet(0);
         }
      };
      callback(callbackPointer);
   }

   public void interrupt()
   {
      interruptFlag.set(1);
   }

   public boolean isInterrupted()
   {
      return interruptFlag.get() == 1;
   }

   @Override
   public boolean releaseReference()
   {
      return super.releaseReference() || callbackPointer.releaseReference();
   }
}
