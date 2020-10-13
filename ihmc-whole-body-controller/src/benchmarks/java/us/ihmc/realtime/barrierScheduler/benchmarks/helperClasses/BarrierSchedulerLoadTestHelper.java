package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.time.ExecutionTimer;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BarrierSchedulerLoadTestHelper
{
   private static int numberOfOperations = 1;

   private BarrierSchedulerLoadTestHelper(){}

   public static void setNumberOfOperations(int numberOfOperations)
   {
      BarrierSchedulerLoadTestHelper.numberOfOperations = numberOfOperations;
   }

   public static int generateFastTaskNumberOfOperations(Random random)
   {
      //      return random.nextInt(4) + 3;
      return numberOfOperations;
   }

   public static int generateSlowTaskNumberOfOperations(Random random)
   {
      //      return random.nextInt(2) + 2;
      return numberOfOperations;
   }

   public static void doMatrixMultiplyOperationsNativeCommonOps(DMatrixRMaj matrixA, DMatrixRMaj matrixB, DMatrixRMaj resultMatrix,
                                                                int numberOfOperations)
   {
      for (int i = 0; i < numberOfOperations; i++)
      {
         if (i % 2 == 0)
         {
            NativeCommonOps.mult(matrixA, matrixB, resultMatrix);
         }
         else
         {
            NativeCommonOps.mult(matrixB, matrixA, resultMatrix);
         }
      }
   }

   public static void doMatrixMultiplyOperationsEJMLCommonOps(DMatrixRMaj matrixA, DMatrixRMaj matrixB, DMatrixRMaj resultMatrix,
                                                              int numberOfOperations)
   {
      for (int i = 0; i < numberOfOperations; i++)
      {
         if (i % 2 == 0)
         {
            CommonOps_DDRM.mult(matrixA, matrixB, resultMatrix);
         }
         else
         {
            CommonOps_DDRM.mult(matrixB, matrixA, resultMatrix);
         }
      }
   }

   public static void printTimingStatisticsCSV(String name, TimingInformation timingInformation)
   {
      System.out
            .format(name + " Jitter(avg, max)%n    %.8f,%.8f%n", timingInformation.getFinalAvgJitterSeconds(), timingInformation.getFinalMaxJitterSeconds());
   }

   public static void printTimingStatisticsCSV(String name, ExecutionTimer executionTimer)
   {
      System.out.format(name + " Execution Time(avg, max, std.dev.)%n    %.8f,%.8f,%.8f%n", executionTimer.getAverageTime().getDoubleValue(),
                        executionTimer.getMaxTime().getDoubleValue(), executionTimer.getStandardDeviation().getDoubleValue());
   }

   public static void doTimingStatistics(TimingInformation timingInformation)
   {
         timingInformation.updateTimingInformation(System.nanoTime());
      //      if (!timingInformation.isInitialized())
      //      {
      //
      //      }
      //      else
      //      {
      //      }
   }
}
