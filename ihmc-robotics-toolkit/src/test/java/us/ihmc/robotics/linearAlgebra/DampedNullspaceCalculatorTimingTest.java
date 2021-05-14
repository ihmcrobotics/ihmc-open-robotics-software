package us.ihmc.robotics.linearAlgebra;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DampedNullspaceCalculatorTimingTest
{
   @Test
   public void testTimings()
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      ExecutionTimer svdTimer = new ExecutionTimer("svdTimer", registry);
      ExecutionTimer leastSquaresTimer = new ExecutionTimer("leastSquaresTimer", registry);
      ExecutionTimer qrTimer = new ExecutionTimer("qrTimer", registry);

      DampedNullspaceCalculator svdCalculator = new DampedSVDNullspaceCalculator(100, 0.1);
      DampedNullspaceCalculator leastSquaresCalculator = new DampedLeastSquaresNullspaceCalculator(100, 0.1);
      DampedNullspaceCalculator qrCalculator = new DampedQRNullspaceCalculator(100, 0.1);

      svdCalculator.setPseudoInverseAlpha(0.1);
      leastSquaresCalculator.setPseudoInverseAlpha(0.1);

      Random random = new Random(10L);
      // test a bunch of sizes
      for (int j = 0; j < 100; j++)
      {

         int Jcolumns = RandomNumbers.nextInt(random, 5, 100);
         int Jrows = RandomNumbers.nextInt(random, 1, Jcolumns);

         int ARows = RandomNumbers.nextInt(random, 1, Jcolumns);

         double[] Jvalues = RandomNumbers.nextDoubleArray(random, Jrows * Jcolumns, 10.0);
         double[] Avalues = RandomNumbers.nextDoubleArray(random, ARows * Jcolumns, 10.0);

         for (int i = 0; i < 100; i++)
         {
            DMatrixRMaj J = new DMatrixRMaj(Jrows, Jcolumns, false, Jvalues);
            DMatrixRMaj A = new DMatrixRMaj(ARows, Jcolumns, false, Avalues);

            DMatrixRMaj A_projected1 = new DMatrixRMaj(ARows, Jcolumns, false, Avalues);
            DMatrixRMaj A_projected2 = new DMatrixRMaj(ARows, Jcolumns, false, Avalues);
            DMatrixRMaj A_projected3 = new DMatrixRMaj(ARows, Jcolumns, false, Avalues);

            svdTimer.startMeasurement();
            svdCalculator.projectOntoNullspace(A, J, A_projected1);
            svdTimer.stopMeasurement();

            leastSquaresTimer.startMeasurement();
            leastSquaresCalculator.projectOntoNullspace(A, J, A_projected2);
            leastSquaresTimer.stopMeasurement();

            qrTimer.startMeasurement();
            qrCalculator.projectOntoNullspace(A, J, A_projected3);
            qrTimer.stopMeasurement();
         }
      }

      PrintTools.info("SVD average time : " + svdTimer.getAverageTime());
      PrintTools.info("Least Squares average time : " + leastSquaresTimer.getAverageTime());
      PrintTools.info("QR average time : " + qrTimer.getAverageTime());
   }
}
