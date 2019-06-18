package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;
import us.ihmc.robotics.time.ExecutionTimer;

import java.util.Random;

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

   public static void doMatrixMultiplyOperationsNativeCommonOps(DenseMatrix64F matrixA, DenseMatrix64F matrixB, DenseMatrix64F resultMatrix,
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

   public static void doMatrixMultiplyOperationsEJMLCommonOps(DenseMatrix64F matrixA, DenseMatrix64F matrixB, DenseMatrix64F resultMatrix,
                                                              int numberOfOperations)
   {
      for (int i = 0; i < numberOfOperations; i++)
      {
         if (i % 2 == 0)
         {
            CommonOps.mult(matrixA, matrixB, resultMatrix);
         }
         else
         {
            CommonOps.mult(matrixB, matrixA, resultMatrix);
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
