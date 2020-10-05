package us.ihmc.robotics.linearAlgebra;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;

public class NullspaceCalculatorTimingTest
{
   @Test
   public void testRemoveNullspaceComponent()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      ExecutionTimer svdTimer = new ExecutionTimer("svdTimer", registry);
      ExecutionTimer leastSquaresTimer = new ExecutionTimer("leastSquarestTimer", registry);
      ExecutionTimer qrTimer = new ExecutionTimer("qrTimer", registry);

      NullspaceCalculator svdCalculator = new SVDNullspaceCalculator(100, true);
      NullspaceCalculator qrCalculator = new QRNullspaceCalculator(100);
      NullspaceCalculator leastSquaresCalculator = new DampedLeastSquaresNullspaceCalculator(100, 0.0);

      Random random = new Random(10L);
      double[] Jvalues = RandomNumbers.nextDoubleArray(random, 200, 10.0);
      double[] Avalues = RandomNumbers.nextDoubleArray(random, 100, 10.0);

      for (int i = 0; i < 10000; i++)
      {
         DMatrixRMaj J = new DMatrixRMaj(10, 20, false, Jvalues);
         DMatrixRMaj A = new DMatrixRMaj(5, 20, false, Avalues);

         DMatrixRMaj A_projected1 = new DMatrixRMaj(5, 20, false, Avalues);
         DMatrixRMaj A_projected2 = new DMatrixRMaj(5, 20, false, Avalues);
         DMatrixRMaj A_projected3 = new DMatrixRMaj(5, 20, false, Avalues);

         svdTimer.startMeasurement();
         svdCalculator.projectOntoNullspace(A, J, A_projected1);
         svdTimer.stopMeasurement();

         leastSquaresTimer.startMeasurement();
         leastSquaresCalculator.projectOntoNullspace(A, J, A_projected2);
         leastSquaresTimer.stopMeasurement();

         qrTimer.startMeasurement();
         qrCalculator.projectOntoNullspace(A, J, A_projected3);
         qrTimer.stopMeasurement();


         // check the solutions are equal
         for (int j = 0; j < A_projected1.getNumElements(); j++)
         {
            assertEquals(A_projected1.get(j), A_projected2.get(j), 1e-5);
            assertEquals(A_projected1.get(j), A_projected3.get(j), 1e-5);
         }
      }

      PrintTools.info("SVD average time : " + svdTimer.getAverageTime());
      PrintTools.info("Least Squares average time : " + leastSquaresTimer.getAverageTime());
      PrintTools.info("QR average time : " + qrTimer.getAverageTime());
   }
}
