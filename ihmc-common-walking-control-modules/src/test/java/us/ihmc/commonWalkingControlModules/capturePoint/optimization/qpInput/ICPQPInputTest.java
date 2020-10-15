package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.Assert;
public class ICPQPInputTest
{
   @Test
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

   @Test
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

   @Test
   public void testReset()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         ICPQPInput icpqpInput = new ICPQPInput(size);

         CommonOps_DDRM.fill(icpqpInput.quadraticTerm, RandomNumbers.nextDouble(random, -1000.0, 1000.0));
         CommonOps_DDRM.fill(icpqpInput.linearTerm, RandomNumbers.nextDouble(random, -1000.0, 1000.0));
         CommonOps_DDRM.fill(icpqpInput.residualCost, RandomNumbers.nextDouble(random, -1000.0, 1000.0));

         Assert.assertNotEquals(CommonOps_DDRM.elementSum(icpqpInput.quadraticTerm), 0.0, 1e-7);
         Assert.assertNotEquals(CommonOps_DDRM.elementSum(icpqpInput.linearTerm), 0.0, 1e-7);
         Assert.assertNotEquals(CommonOps_DDRM.elementSum(icpqpInput.residualCost), 0.0, 1e-7);

         icpqpInput.reset();

         Assert.assertEquals(CommonOps_DDRM.elementSum(icpqpInput.quadraticTerm), 0.0, 1e-7);
         Assert.assertEquals(CommonOps_DDRM.elementSum(icpqpInput.linearTerm), 0.0, 1e-7);
         Assert.assertEquals(CommonOps_DDRM.elementSum(icpqpInput.residualCost), 0.0, 1e-7);
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(10L);
      int size = RandomNumbers.nextInt(random, 1, 100);
      DMatrixRMaj quadratic = new DMatrixRMaj(size, size);
      DMatrixRMaj linear = new DMatrixRMaj(size, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

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

   @Test
   public void testComputeCost()
   {
      DMatrixRMaj quadratic = new DMatrixRMaj(2, 2);
      DMatrixRMaj linear = new DMatrixRMaj(2, 1);
      DMatrixRMaj scalar = new DMatrixRMaj(1, 1);

      DMatrixRMaj solution = new DMatrixRMaj(2, 1);

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
      shouldBe -= 21.0;
      cost = icpqpInput.computeCost(solution);
      Assert.assertEquals(shouldBe, cost, 1e-7);

      icpqpInput.residualCost.set(scalar);
      shouldBe += 6.0;
      cost = icpqpInput.computeCost(solution);
      Assert.assertEquals(shouldBe, cost, 1e-7);
   }
}
