package us.ihmc.realtime.barrierScheduler.benchmarks;

import org.apache.commons.math3.util.Pair;
import us.ihmc.affinity.CPUTopology;
import us.ihmc.affinity.Package;
import us.ihmc.commons.Conversions;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.BindingContext;
import us.ihmc.realtime.*;
import us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses.*;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BarrierSchedulerMatrixMultiplyBenchmark
{
   private static final long SCHEDULER_PERIOD_NANOSECONDS = 1000000; // 1KHz
   private static final int NUM_ITERATIONS_OF_SCHEDULER = 60000; // 60 seconds @ 1KHz
   private static final double ESTIMATED_DURATION = (double) SCHEDULER_PERIOD_NANOSECONDS * (double) NUM_ITERATIONS_OF_SCHEDULER / 1e9;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble actualSchedulerDTMillis = new YoDouble("actualSchedulerDTMillis", registry);
   private final YoDouble actualFastTaskDTMillis = new YoDouble("actualFastTaskDTMillis", registry);
   private final YoDouble actualSlowTaskDTMillis = new YoDouble("actualSlowTaskDTMillis", registry);
   private PeriodicRealtimeThread schedulerThread;

   public void testBarrierSchedulerThreeThreadTwoTaskMatrixMultiply(boolean useNativeCommonOps)
   {
      System.out.println("Performing Benchmark; Using Native Commons Ops: " + useNativeCommonOps);

      CPUDMALatency.setLatency(0);

      PriorityParameters schedulerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
      PriorityParameters fastTaskPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
      PriorityParameters slowTaskPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);

      PeriodicParameters schedulerPeriodicParameters = new PeriodicParameters(SCHEDULER_PERIOD_NANOSECONDS);

      CPUTopology cpuTopology = new CPUTopology();
      Package cpuPackage = cpuTopology.getPackage(0);

      if (cpuTopology.isHyperThreadingEnabled())
      {
         System.err.println("WARNING: Hyper-Threading is enabled. Expect higher amounts of jitter");
      }

      final YoVariableServer yoVariableServer = new YoVariableServer(getClass(), null, new DataServerSettings(false),
                                                                     Conversions.nanosecondsToSeconds(SCHEDULER_PERIOD_NANOSECONDS));

      yoVariableServer.setMainRegistry(registry, null, null);

      MultiplySmallMatricesALotTask fastTask = new MultiplySmallMatricesALotTask(actualFastTaskDTMillis, registry, SCHEDULER_PERIOD_NANOSECONDS, 1,
                                                                                 useNativeCommonOps); // 1KHz
      MultiplyBigMatricesALotTask slowTask = new MultiplyBigMatricesALotTask(actualSlowTaskDTMillis, registry, SCHEDULER_PERIOD_NANOSECONDS, 4,
                                                                             useNativeCommonOps); // 250Hz

      List<BarrierSchedulerLoadTestTask> tasks = Arrays.asList(fastTask, slowTask);
      BarrierSchedulerLoadTestContext context = new BarrierSchedulerLoadTestContext(new Pair<>(fastTask, slowTask));

      BarrierScheduler<BindingContext> barrierScheduler = new BarrierScheduler<>(tasks, context, BarrierScheduler.TaskOverrunBehavior.SKIP_TICK);

      final TimingInformation schedulerTimingInformation = new TimingInformation("Scheduler", SCHEDULER_PERIOD_NANOSECONDS);

      RealtimeThread fastTaskThread = new RealtimeThread(fastTaskPriority, fastTask, "fastTask");
      RealtimeThread slowTaskThread = new RealtimeThread(slowTaskPriority, slowTask, "slowTask");

      final Runnable schedulerRunnable = new Runnable()
      {
         boolean isFinished = false;
         boolean firstTick = true;
         int iterations = -1;

         @Override
         public void run()
         {
            long nanoTime = System.nanoTime();
            if (firstTick)
            {
               schedulerTimingInformation.initialize(nanoTime, actualSchedulerDTMillis);
               firstTick = false;
            }
            else
            {
               schedulerTimingInformation.updateTimingInformation(nanoTime);
            }

            barrierScheduler.run();
            iterations++;

            yoVariableServer.update(nanoTime);

            if (iterations > NUM_ITERATIONS_OF_SCHEDULER && !isFinished)
            {
               barrierScheduler.shutdown();

               fastTask.doTimingReporting();
               System.out.println();
               slowTask.doTimingReporting();
               System.out.println();

               BarrierSchedulerLoadTestHelper.printTimingStatisticsCSV("Scheduler", schedulerTimingInformation);

               isFinished = true;
               finished();
            }

         }
      };

      schedulerThread = new PeriodicRealtimeThread(schedulerPriority, schedulerPeriodicParameters, schedulerRunnable, "barrierSchedulerThread");

      System.out.println("Pinning scheduler thread to core 1");
      schedulerThread.setAffinity(cpuPackage.getCore(1).getDefaultProcessor());

      System.out.println("Pinning fast task to core 2 and slow task to core 3.");
      fastTaskThread.setAffinity(cpuPackage.getCore(2).getDefaultProcessor());
      slowTaskThread.setAffinity(cpuPackage.getCore(3).getDefaultProcessor());

      yoVariableServer.start();

      fastTaskThread.start();
      slowTaskThread.start();

      schedulerThread.start();

      System.out.println("Starting cyclic test [Iterations: " + NUM_ITERATIONS_OF_SCHEDULER + "; Estimated Duration: " + ESTIMATED_DURATION + "s]");

      fastTaskThread.join();
      slowTaskThread.join();
      schedulerThread.join();

      yoVariableServer.close();
   }

   private void finished()
   {
      schedulerThread.shutdown();
   }

   public static void main(String[] args)
   {
      int numberOfIterations = Integer.parseInt(args[0]);
      boolean useNativeCommonOps = Boolean.parseBoolean(args[1]);

      BarrierSchedulerLoadTestHelper.setNumberOfOperations(numberOfIterations);
      BarrierSchedulerMatrixMultiplyBenchmark barrierSchedulerMatrixMultiplyBenchmark = new BarrierSchedulerMatrixMultiplyBenchmark();

      //      barrierSchedulerMatrixMultiplyBenchmark.testBarrierSchedulerThreeThreadTwoTaskMatrixMultiplyNativeCommonOps();
      barrierSchedulerMatrixMultiplyBenchmark.testBarrierSchedulerThreeThreadTwoTaskMatrixMultiply(useNativeCommonOps);
   }
}
