package us.ihmc.robotDataLogger;

import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.logger.LogAliveListener;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;

public class LogWatcher implements LogAliveListener
{
   private static final long TIMEOUT = Conversions.secondsToNanoseconds(2.5) + YoVariableLoggerListener.STATUS_PACKET_RATE;

   private long lastAliveTime = -TIMEOUT;
   private long lastTimestamp = 0;

   private boolean isLogging;

   @Override
   public void recievedLogAliveCommand()
   {
      lastAliveTime = lastTimestamp;
   }

   public void update(long timestamp)
   {
      lastTimestamp = timestamp;
      isLogging = lastTimestamp - lastAliveTime < TIMEOUT;
   }

   public boolean isLogging()
   {
      return isLogging;
   }
}
