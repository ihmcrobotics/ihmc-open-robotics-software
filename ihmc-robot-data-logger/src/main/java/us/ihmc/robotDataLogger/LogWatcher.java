package us.ihmc.robotDataLogger;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.logger.LogAliveListener;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;

public class LogWatcher implements LogAliveListener
{
   private static final long TIMEOUT = Conversions.secondsToNanoseconds(2.5) + YoVariableLoggerListener.STATUS_PACKET_RATE;

   private AtomicLong lastAliveTime = new AtomicLong(-TIMEOUT);
   private AtomicLong lastTimestamp = new AtomicLong();

   private AtomicBoolean isLogging = new AtomicBoolean();
   private AtomicBoolean camerasLogging = new AtomicBoolean();

   @Override
   public void receivedLogAliveCommand(boolean camerasLogging)
   {
      lastAliveTime.set(lastTimestamp.get());
      this.camerasLogging.set(camerasLogging);
   }

   public void update(long timestamp)
   {
      lastTimestamp.set(timestamp);
      isLogging.set(lastTimestamp.get() - lastAliveTime.get() < TIMEOUT);
   }

   public boolean isLogging()
   {
      return isLogging.get();
   }

   public boolean isLoggingWithCameras()
   {
      return isLogging.get() && camerasLogging.get();
   }
}
