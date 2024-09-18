package us.ihmc.perception.ffmpeg;

import org.apache.logging.log4j.core.util.ExecutorServices;
import us.ihmc.commons.Conversions;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class FFMPEGTimeoutCallback extends FFMPEGInterruptCallback
{
   private long timeout;
   private TimeUnit timeUnit;
   private final ExecutorService timeoutExecutor;
   private Future<?> futureTimeout;

   public FFMPEGTimeoutCallback()
   {
      timeoutExecutor = Executors.newSingleThreadExecutor();
   }

   private void runTimeout()
   {
      try
      {
         timeUnit.sleep(timeout);
         interrupt();
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
      if (interrupt)
         interrupt();
      if (futureTimeout != null)
         futureTimeout.cancel(true);
   }

   @Override
   public boolean releaseReference()
   {
      stop(true);
      ExecutorServices.shutdown(timeoutExecutor, 1, TimeUnit.SECONDS, getClass().getSimpleName());

      return super.releaseReference();
   }
}
