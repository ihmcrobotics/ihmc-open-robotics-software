package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class NullspaceCalculatorTimingTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRemoveNullspaceComponent()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      ExecutionTimer svdTimer = new ExecutionTimer("svdTimer", registry);
      ExecutionTimer leastSquaresTimer = new ExecutionTimer("leastSquarestTimer", registry);


      DampedNullspaceCalculator svdCalculator = new DampedSVDNullspaceCalculator(100, 0.1);
      DampedNullspaceCalculator leastSquaresCalculator = new DampedLeastSquaresNullspaceCalculator(100, 0.1, registry);

      svdCalculator.setPseudoInverseAlpha(0.1);
      leastSquaresCalculator.setPseudoInverseAlpha(0.1);

      for (int i = 0; i < 10000; i++)
      {
         Random random = new Random(10L);
         double[] Jvalues = RandomNumbers.nextDoubleArray(random, 200, 10.0);
         double[] Avalues = RandomNumbers.nextDoubleArray(random, 100, 10.0);
         DenseMatrix64F J = new DenseMatrix64F(10, 20, false, Jvalues);
         DenseMatrix64F A = new DenseMatrix64F(5, 20, false, Avalues);

         DenseMatrix64F A_projected1 = new DenseMatrix64F(5, 20, false, Avalues);
         DenseMatrix64F A_projected2 = new DenseMatrix64F(5, 20, false, Avalues);

         svdTimer.startMeasurement();
         svdCalculator.projectOntoNullspace(A, J, A_projected1);
         svdTimer.stopMeasurement();

         leastSquaresTimer.startMeasurement();
         leastSquaresCalculator.projectOntoNullspace(A, J, A_projected2);
         leastSquaresTimer.stopMeasurement();

         // check the solutions are equal
         for (int j = 0; j < A_projected1.getNumElements(); j++)
            assertEquals(A_projected1.get(j), A_projected2.get(j), 1e-5);
      }

      PrintTools.info("SVD average time : " + svdTimer.getAverageTime());
      PrintTools.info("Least Squares average time : " + leastSquaresTimer.getAverageTime());
   }
}
