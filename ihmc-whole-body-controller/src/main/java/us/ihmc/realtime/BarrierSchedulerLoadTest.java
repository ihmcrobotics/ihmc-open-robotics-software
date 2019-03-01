package us.ihmc.realtime;

import org.apache.commons.math3.util.Pair;
import us.ihmc.affinity.CPUTopology;
import us.ihmc.affinity.Package;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.BindingContext;
import us.ihmc.realtime.helperClasses.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Arrays;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BarrierSchedulerLoadTest
{
   private static final long SCHEDULER_PERIOD_NANOSECONDS = 1000000; // 1KHz
   private static final int NUM_ITERATIONS_OF_SCHEDULER = 60000; // 60 seconds @ 1KHz
   private static final double ESTIMATED_DURATION = (double) SCHEDULER_PERIOD_NANOSECONDS * (double) NUM_ITERATIONS_OF_SCHEDULER / 1e9;

   public void testBarrierSchedulerThreeThreadTwoTaskMatrixMultiply(boolean useNativeCommonOps)
   {
      System.out.println("Performing Benchmark; Using Native Commons Ops: " + useNativeCommonOps);

      final YoVariableRegistry registry = new YoVariableRegistry("BarrierSchedulerLoadTestRegistry");
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

      MultiplySmallMatricesALotTask fastTask = new MultiplySmallMatricesALotTask(registry, SCHEDULER_PERIOD_NANOSECONDS, 1, useNativeCommonOps); // 1KHz
      MultiplyBigMatricesALotTask slowTask = new MultiplyBigMatricesALotTask(registry, SCHEDULER_PERIOD_NANOSECONDS, 4, useNativeCommonOps); // 250Hz

      List<BarrierSchedulerLoadTestTask> tasks = Arrays.asList(fastTask, slowTask);
      BarrierSchedulerLoadTestContext context = new BarrierSchedulerLoadTestContext(new Pair<>(fastTask, slowTask));

      BarrierScheduler<BindingContext> barrierScheduler = new BarrierScheduler<>(tasks, context, BarrierScheduler.TaskOverrunBehavior.SKIP_TICK);

      final TimingInformation schedulerTimingInformation = new TimingInformation("Scheduler", SCHEDULER_PERIOD_NANOSECONDS);

      RealtimeThread fastTaskThread = new RealtimeThread(fastTaskPriority, fastTask, "fastTask");
      RealtimeThread slowTaskThread = new RealtimeThread(slowTaskPriority, slowTask, "slowTask");

      RealtimeThread schedulerThread = new RealtimeThread(schedulerPriority, schedulerPeriodicParameters, "barrierSchedulerThread")
      {
         boolean firstTick = true;
         int iterations = -1;

         @Override
         public void run()
         {
            while (iterations < NUM_ITERATIONS_OF_SCHEDULER)
            {
               super.waitForNextPeriod();
               if (firstTick)
               {
                  schedulerTimingInformation.initialize(System.nanoTime());
                  firstTick = false;
               }
               else
               {
                  schedulerTimingInformation.updateTimingInformation(System.nanoTime());
               }

               barrierScheduler.run();
               iterations++;
            }

            fastTask.doTimingReporting();
            System.out.println();
            slowTask.doTimingReporting();
            System.out.println();

            BarrierSchedulerLoadTestHelper.printTimingStatisticsCSV("Scheduler", schedulerTimingInformation);
         }
      };

      System.out.println("Pinning scheduler thread to core 1");
      schedulerThread.setAffinity(cpuPackage.getCore(1).getDefaultProcessor());

      System.out.println("Pinning fast task to core 2 and slow task to core 3.");
      fastTaskThread.setAffinity(cpuPackage.getCore(2).getDefaultProcessor());
      slowTaskThread.setAffinity(cpuPackage.getCore(3).getDefaultProcessor());

      fastTaskThread.start();
      slowTaskThread.start();

      schedulerThread.start();

      System.out.println("Starting cyclic test [Iterations: " + NUM_ITERATIONS_OF_SCHEDULER + "; Estimated Duration: " + ESTIMATED_DURATION + "s]");

      schedulerThread.join();
   }

   public static void main(String[] args)
   {
      int numberOfIterations = Integer.parseInt(args[0]);
      boolean useNativeCommonOps = Boolean.parseBoolean(args[1]);

      BarrierSchedulerLoadTestHelper.setNumberOfOperations(numberOfIterations);
      BarrierSchedulerLoadTest barrierSchedulerLoadTest = new BarrierSchedulerLoadTest();

      //      barrierSchedulerLoadTest.testBarrierSchedulerThreeThreadTwoTaskMatrixMultiplyNativeCommonOps();
      barrierSchedulerLoadTest.testBarrierSchedulerThreeThreadTwoTaskMatrixMultiply(useNativeCommonOps);
   }
}
