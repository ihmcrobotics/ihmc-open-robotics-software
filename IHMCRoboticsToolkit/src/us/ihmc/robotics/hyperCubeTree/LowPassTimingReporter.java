package us.ihmc.robotics.hyperCubeTree;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class LowPassTimingReporter
{
   private static final int HEURISTIC_OWN_COMPUTATION_TIME_APPROXIMATION = 22;// in nanoseconds
   private long startTime;
   private long averageTime;
   private double averageSeconds;
   private final int alpha;
   private final double divisor;
   private long averagePeriod;
   private double periodSeconds;
   private double timeConstantSeconds;
   boolean firstTime = true;

   public LowPassTimingReporter(int alpha)
   {
      this.alpha = alpha;
      this.divisor = 1 / ((double) (1 << alpha));
   }

   public void startTime()
   {
      long nanoTime = System.nanoTime();
      if (!firstTime)
      {
      averagePeriod = averagePeriod + ((((nanoTime-startTime) << alpha) - averagePeriod) >> alpha);
      periodSeconds = (double) averagePeriod * divisor * 1e-9;
      timeConstantSeconds = periodSeconds*(1<<alpha);
      } else
      {
         firstTime=false;
      }
      this.startTime = nanoTime;
   }

   public void endTime()
   {
      averageTime = averageTime + ((((System.nanoTime() - startTime-HEURISTIC_OWN_COMPUTATION_TIME_APPROXIMATION) << alpha) - averageTime) >> alpha);
      averageSeconds = (double) averageTime * divisor * 1e-9;
   }

   public void setupRecording(String name, String action)
   {
      this.setupRecording(name, action, 1000L, 1000L);
   }

   public void setupRecording(final String name, final String action, long loopPeriodMilliseconds, long delayMilliseconds)
   {
      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(new Runnable()
      {
         public void run()
         {
            System.out.println(generateMessage(name, action));
         }

         
      }, delayMilliseconds, loopPeriodMilliseconds, TimeUnit.MILLISECONDS);
   }
   public String generateMessage(String name, String action)
   {
      return name + " takes on average " + averageSeconds + " seconds to " + action
            +"\n\tThe time spent on this method is approximately "+ (averageSeconds/periodSeconds)
                         + "\n\tThe low pass filter time constant for this timer is approximately " + timeConstantSeconds + " seconds.";
   }
   public double getAverageSeconds()
   {
      return averageSeconds;
   }
   public double getApproximateTimeConstant()
   {
      return timeConstantSeconds;
   }
}
