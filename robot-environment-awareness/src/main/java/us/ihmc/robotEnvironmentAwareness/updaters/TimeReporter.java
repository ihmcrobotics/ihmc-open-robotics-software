package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import org.apache.commons.lang3.time.StopWatch;

import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;

public class TimeReporter
{
   private final AtomicLong minimumNanoTimeToReport = new AtomicLong(Conversions.millisecondsToNanoseconds(100L));
   private final AtomicBoolean reportTimeEnabled = new AtomicBoolean(false);
   private final ThreadLocal<StopWatch> stopWatchLocal = ThreadLocal.withInitial(() -> new StopWatch());
   private String stringToReport = "";

   public TimeReporter()
   {
   }

   public void enableTimeReport(boolean value)
   {
      reportTimeEnabled.set(value);
   }

   public void mininmumTimeToReport(long minTimeInMilliseconds)
   {
      minimumNanoTimeToReport.set(Conversions.millisecondsToNanoseconds(minTimeInMilliseconds));
   }

   public String getStringToReport()
   {
      return stringToReport;
   }

   public void run(Runnable command, String timeReportPrefix)
   {
      if (reportTimeEnabled.get())
      {
         StopWatch stopWatch = stopWatchLocal.get();
         stopWatch.reset();
         stopWatch.start();
         command.run();
         long nanoTime = stopWatch.getNanoTime();
         stringToReport = timeReportPrefix + Conversions.nanosecondsToSeconds(nanoTime);
         if (nanoTime > minimumNanoTimeToReport.get())
            LogTools.info(stringToReport);
      }
      else
         command.run();
   }
}
