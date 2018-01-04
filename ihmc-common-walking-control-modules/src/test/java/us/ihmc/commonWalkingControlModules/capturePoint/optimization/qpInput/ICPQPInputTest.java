package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEquals()
   {
      Random random = new Random(10L);
      int size = RandomNumbers.nextInt(random, 1, 100);
      DenseMatrix64F quadratic = new DenseMatrix64F(size, size);
      DenseMatrix64F linear = new DenseMatrix64F(size, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      ICPQPInput icpqpInput = new ICPQPInput(size);
      ICPQPInput other = new ICPQPInput(size);

      for (int i = 0; i < quadratic.getNumElements(); i++)
         quadratic.set(i, RandomNumbers.nextDouble(random, 1000.0));
      for (int i = 0; i < linear.getNumElements(); i++)
         linear.set(i, RandomNumbers.nextDouble(random, 1000.0));
      scalar.set(0, RandomNumbers.nextDouble(random, 1000.0));

      icpqpInput.quadraticTerm.set(quadratic);
      icpqpInput.linearTerm.set(linear);
      icpqpInput.residualCost.set(scalar);

      other.quadraticTerm.set(quadratic);
      other.linearTerm.set(linear);
      other.residualCost.set(scalar);

      Assert.assertTrue(icpqpInput.equals(other));

      other.residualCost.set(0, 0, other.residualCost.get(0, 0) + 1.0);
      Assert.assertFalse(icpqpInput.equals(other));

      other.residualCost.set(icpqpInput.residualCost);
      Assert.assertTrue(icpqpInput.equals(other));

      other.quadraticTerm.set(0, 0, icpqpInput.quadraticTerm.get(0, 0) + 1.0);
      Assert.assertFalse(icpqpInput.equals(other));

      other.quadraticTerm.set(icpqpInput.quadraticTerm);
      Assert.assertTrue(icpqpInput.equals(other));

      other.linearTerm.set(0, 0, icpqpInput.linearTerm.get(0, 0) + 1.0);
      Assert.assertFalse(icpqpInput.equals(other));

      other.linearTerm.set(icpqpInput.linearTerm);
      Assert.assertTrue(icpqpInput.equals(other));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeCost()
   {
      DenseMatrix64F quadratic = new DenseMatrix64F(2, 2);
      DenseMatrix64F linear = new DenseMatrix64F(2, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F solution = new DenseMatrix64F(2, 1);

      ICPQPInput icpqpInput = new ICPQPInput(2);

      solution.set(0, 2.0);
      solution.set(1, 3.0);

      quadratic.set(0, 0, 2.0);
      quadratic.set(1, 1, 4.0);

      linear.set(0, 0, 3.0);
      linear.set(1, 0, 5.0);

      scalar.set(0, 0, 6.0);

      icpqpInput.quadraticTerm.set(quadratic);

      double shouldBe = 22.0;
      double cost = icpqpInput.computeCost(solution);

      Assert.assertEquals(shouldBe, cost, 1e-7);

      icpqpInput.linearTerm.set(linear);
      shouldBe += 21.0;
      cost = icpqpInput.computeCost(solution);
      Assert.assertEquals(shouldBe, cost, 1e-7);

      icpqpInput.residualCost.set(scalar);
      shouldBe += 6.0;
      cost = icpqpInput.computeCost(solution);
      Assert.assertEquals(shouldBe, cost, 1e-7);
   }
}
