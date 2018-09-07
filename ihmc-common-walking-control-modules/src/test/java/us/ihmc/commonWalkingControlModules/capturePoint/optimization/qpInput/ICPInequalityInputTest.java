package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

import java.util.Random;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPInequalityInputTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSize()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         int numberOfConstraints = RandomNumbers.nextInt(random, 1, 200);
         ICPInequalityInput input = new ICPInequalityInput(numberOfConstraints, size);

         Assert.assertEquals(numberOfConstraints, input.linearOperator.numRows);
         Assert.assertEquals(size, input.linearOperator.numCols);
         Assert.assertEquals(numberOfConstraints, input.objective.numRows);
         Assert.assertEquals(1, input.objective.numCols);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testReshape()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         int numberOfConstraints = RandomNumbers.nextInt(random, 1, 200);

         ICPInequalityInput input = new ICPInequalityInput(numberOfConstraints, size);

         Assert.assertEquals(numberOfConstraints, input.linearOperator.numRows);
         Assert.assertEquals(size, input.linearOperator.numCols);
         Assert.assertEquals(numberOfConstraints, input.objective.numRows);
         Assert.assertEquals(1, input.objective.numCols);

         int newSize = RandomNumbers.nextInt(random, 1, 200);
         int newNumberOfConstraints = RandomNumbers.nextInt(random, 1, 200);

         input.reshape(newNumberOfConstraints, newSize);

         Assert.assertEquals(newNumberOfConstraints, input.linearOperator.numRows);
         Assert.assertEquals(newSize, input.linearOperator.numCols);
         Assert.assertEquals(newNumberOfConstraints, input.objective.numRows);
         Assert.assertEquals(1, input.objective.numCols);
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testReset()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         int numberOfConstraints = RandomNumbers.nextInt(random, 1, 200);

         ICPInequalityInput input = new ICPInequalityInput(numberOfConstraints, size);

         CommonOps.fill(input.linearOperator, RandomNumbers.nextDouble(random, -1000.0, 1000.0));
         CommonOps.fill(input.objective, RandomNumbers.nextDouble(random, -1000.0, 1000.0));

         Assert.assertNotEquals(CommonOps.elementSum(input.linearOperator), 0.0, 1e-7);
         Assert.assertNotEquals(CommonOps.elementSum(input.objective), 0.0, 1e-7);

         input.reset();

         Assert.assertEquals(CommonOps.elementSum(input.linearOperator), 0.0, 1e-7);
         Assert.assertEquals(CommonOps.elementSum(input.objective), 0.0, 1e-7);
      }
   }




}
