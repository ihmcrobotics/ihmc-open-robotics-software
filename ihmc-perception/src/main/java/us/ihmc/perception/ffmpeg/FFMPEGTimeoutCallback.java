package us.ihmc.perception.ffmpeg;

import org.apache.logging.log4j.core.util.ExecutorServices;
import org.bytedeco.ffmpeg.avformat.AVIOInterruptCB;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.commons.Conversions;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

public class FFMPEGTimeoutCallback extends AVIOInterruptCB
{
   private final Callback_Pointer callbackPointer;
   private final AtomicInteger interruptFlag = new AtomicInteger(0);

   private long timeout;
   private TimeUnit timeUnit;
   private final ExecutorService timeoutExecutor;
   private Future<?> futureTimeout;

   public FFMPEGTimeoutCallback()
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

      timeoutExecutor = Executors.newSingleThreadExecutor();
   }

   private void runTimeout()
   {
      try
      {
         timeUnit.sleep(timeout);
         interruptFlag.set(1);
      }
      catch (InterruptedException ignored) {}
   }

   /**
    * Starts counting down the timeout. Once the timeout passes, sets the interrupt flag.
    * @param timeoutSeconds Timeout duration in seconds
    */
   public void start(double timeoutSeconds)
   {
      start(Conversions.secondsToNanoseconds(timeoutSeconds), TimeUnit.NANOSECONDS); // Convert to nanoseconds to preserve some precision
   }

   /**
    * Starts counting down the timeout. Once the timeout passes, sets the interrupt flag.
    * @param timeout Timeout duration
    * @param timeUnit Time unit of timeout duration
    */
   public void start(long timeout, TimeUnit timeUnit)
   {
      stop();

      if (timeout < 0)
         return;

      this.timeout = timeout;
      this.timeUnit = timeUnit;

      futureTimeout = timeoutExecutor.submit(this::runTimeout);
   }

   /**
    * Stops the timeout without setting the interrupt flag
    */
   public void stop()
   {
      stop(false);
   }

   /**
    * Stops the timeout
    * @param interrupt Whether to set the interrupt flag.
    */
   public void stop(boolean interrupt)
   {
      interruptFlag.set(interrupt ? 1 : 0);
      if (futureTimeout != null && !futureTimeout.isDone())
         futureTimeout.cancel(true);
   }

   @Override
   public void close()
   {
      stop(true);
      ExecutorServices.shutdown(timeoutExecutor, 2, TimeUnit.SECONDS, getClass().getSimpleName());

      super.close();
      callbackPointer.close();
   }
}
