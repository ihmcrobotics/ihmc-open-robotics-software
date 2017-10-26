package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;

import java.util.Random;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPQPInputTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSize()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         ICPQPInput icpqpInput = new ICPQPInput(size);

         Assert.assertEquals(size, icpqpInput.quadraticTerm.numRows);
         Assert.assertEquals(size, icpqpInput.quadraticTerm.numCols);
         Assert.assertEquals(size, icpqpInput.linearTerm.numRows);
         Assert.assertEquals(1, icpqpInput.linearTerm.numCols);
         Assert.assertEquals(1, icpqpInput.residualCost.numRows);
         Assert.assertEquals(1, icpqpInput.residualCost.numCols);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testReshape()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         ICPQPInput icpqpInput = new ICPQPInput(size);

         Assert.assertEquals(size, icpqpInput.quadraticTerm.numRows);
         Assert.assertEquals(size, icpqpInput.quadraticTerm.numCols);
         Assert.assertEquals(size, icpqpInput.linearTerm.numRows);
         Assert.assertEquals(1, icpqpInput.linearTerm.numCols);
         Assert.assertEquals(1, icpqpInput.residualCost.numRows);
         Assert.assertEquals(1, icpqpInput.residualCost.numCols);

         int newSize = RandomNumbers.nextInt(random, 1, 200);
         icpqpInput.reshape(newSize);

         Assert.assertEquals(newSize, icpqpInput.quadraticTerm.numRows);
         Assert.assertEquals(newSize, icpqpInput.quadraticTerm.numCols);
         Assert.assertEquals(newSize, icpqpInput.linearTerm.numRows);
         Assert.assertEquals(1, icpqpInput.linearTerm.numCols);
         Assert.assertEquals(1, icpqpInput.residualCost.numRows);
         Assert.assertEquals(1, icpqpInput.residualCost.numCols);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testReset()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         ICPQPInput icpqpInput = new ICPQPInput(size);

         CommonOps.fill(icpqpInput.quadraticTerm, RandomNumbers.nextDouble(random, -1000.0, 1000.0));
         CommonOps.fill(icpqpInput.linearTerm, RandomNumbers.nextDouble(random, -1000.0, 1000.0));
         CommonOps.fill(icpqpInput.residualCost, RandomNumbers.nextDouble(random, -1000.0, 1000.0));

         Assert.assertNotEquals(CommonOps.elementSum(icpqpInput.quadraticTerm), 0.0, 1e-7);
         Assert.assertNotEquals(CommonOps.elementSum(icpqpInput.linearTerm), 0.0, 1e-7);
         Assert.assertNotEquals(CommonOps.elementSum(icpqpInput.residualCost), 0.0, 1e-7);

         icpqpInput.reset();

         Assert.assertEquals(CommonOps.elementSum(icpqpInput.quadraticTerm), 0.0, 1e-7);
         Assert.assertEquals(CommonOps.elementSum(icpqpInput.linearTerm), 0.0, 1e-7);
         Assert.assertEquals(CommonOps.elementSum(icpqpInput.residualCost), 0.0, 1e-7);
      }
   }
}
