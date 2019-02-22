package us.ihmc.realtime.helperClasses;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;

import java.util.Random;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BarrierSchedulerLoadTestHelper
{
   private BarrierSchedulerLoadTestHelper(){}

   public static int generateFastTaskNumberOfOperations(Random random)
   {
      return random.nextInt(4) + 12;
   }

   public static int generateSlowTaskNumberOfOperations(Random random)
   {
      return random.nextInt(2) + 4;
   }

   public static void doMatrixMultiplyOperations(DenseMatrix64F matrixA, DenseMatrix64F matrixB, DenseMatrix64F resultMatrix, int numberOfOperations)
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

   public static void doTimingStatistics(TimingInformation timingInformation)
   {
      if(!timingInformation.isInitialized())
      {
         timingInformation.initialize(System.nanoTime());
      }
      else
      {
         timingInformation.updateTimingInformation(System.nanoTime());
      }
   }
}
