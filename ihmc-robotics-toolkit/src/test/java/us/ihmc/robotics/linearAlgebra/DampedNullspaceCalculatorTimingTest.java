package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.QRDecomposition;
import org.ejml.interfaces.decomposition.QRPDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class DampedNullspaceCalculatorTimingTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 100000000)
   public void testRemoveNullspaceComponent()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      ExecutionTimer svdTimer = new ExecutionTimer("svdTimer", registry);
      ExecutionTimer leastSquaresTimer = new ExecutionTimer("leastSquaresTimer", registry);
      ExecutionTimer qrTimer = new ExecutionTimer("qrTimer", registry);
      ExecutionTimer svdDecomposerTimer = new ExecutionTimer("svdDecomposerTimer", registry);
      ExecutionTimer qrDecomposerTimer = new ExecutionTimer("qrDecomposerTimer", registry);
      ExecutionTimer qrpDecomposerTimer = new ExecutionTimer("qrpDecomposerTimer", registry);


      DampedNullspaceCalculator svdCalculator = new DampedSVDNullspaceCalculator(100, 0.1);
      DampedNullspaceCalculator leastSquaresCalculator = new DampedLeastSquaresNullspaceCalculator(100, 0.1, registry);
      DampedNullspaceCalculator qrCalculator = new DampedQRNullspaceCalculator(100, 0.1);

      SingularValueDecomposition<DenseMatrix64F> svdDecomposer = DecompositionFactory.svd(10, 10, false, true, false);
      QRDecomposition<DenseMatrix64F> qrDecomposer = DecompositionFactory.qr(10, 10);
      QRPDecomposition<DenseMatrix64F> qrpDecomposer = DecompositionFactory.qrp(10, 10);

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
            DenseMatrix64F J = new DenseMatrix64F(Jrows, Jcolumns, false, Jvalues);
            DenseMatrix64F A = new DenseMatrix64F(ARows, Jcolumns, false, Avalues);

            DenseMatrix64F A_projected1 = new DenseMatrix64F(ARows, Jcolumns, false, Avalues);
            DenseMatrix64F A_projected2 = new DenseMatrix64F(ARows, Jcolumns, false, Avalues);
            DenseMatrix64F A_projected3 = new DenseMatrix64F(ARows, Jcolumns, false, Avalues);

            /*
            svdDecomposerTimer.startMeasurement();
            svdDecomposer.decompose(J);
            svdDecomposerTimer.stopMeasurement();

            qrDecomposerTimer.startMeasurement();
            qrDecomposer.decompose(J);
            qrDecomposerTimer.stopMeasurement();

            qrpDecomposerTimer.startMeasurement();
            qrpDecomposer.decompose(J);
            qrpDecomposerTimer.stopMeasurement();
            */

            svdTimer.startMeasurement();
            svdCalculator.projectOntoNullspace(A, J, A_projected1);
            svdTimer.stopMeasurement();

            leastSquaresTimer.startMeasurement();
            leastSquaresCalculator.projectOntoNullspace(A, J, A_projected2);
            leastSquaresTimer.stopMeasurement();

            qrTimer.startMeasurement();
            qrCalculator.projectOntoNullspace(A, J, A_projected3);
            qrTimer.stopMeasurement();

            /*
            // check the solutions are equal
            for (int k = 0; k < A_projected1.getNumElements(); k++)
            {
               assertEquals(A_projected1.get(k), A_projected2.get(k), 1e-5);
               assertEquals(A_projected1.get(k), A_projected3.get(k), 1e-5);
            }
            */
         }
      }

      PrintTools.info("SVD average time : " + svdTimer.getAverageTime());
      PrintTools.info("Least Squares average time : " + leastSquaresTimer.getAverageTime());
      PrintTools.info("QR average time : " + qrTimer.getAverageTime());
      /*
      PrintTools.info("SVD decompose average time : " + svdDecomposerTimer.getAverageTime());
      PrintTools.info("QR decompose average time : " + qrDecomposerTimer.getAverageTime());
      PrintTools.info("QRP decompose average time : " + qrpDecomposerTimer.getAverageTime());
      */
   }
}
