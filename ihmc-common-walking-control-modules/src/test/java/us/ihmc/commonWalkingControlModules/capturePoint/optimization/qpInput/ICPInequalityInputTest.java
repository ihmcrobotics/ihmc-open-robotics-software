package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import java.util.Random;

import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.Assert;

public class ICPInequalityInputTest
{
   @Test
   public void testSize()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         int numberOfConstraints = RandomNumbers.nextInt(random, 1, 200);
         ICPInequalityInput input = new ICPInequalityInput(numberOfConstraints, size);

         Assert.assertEquals(numberOfConstraints, input.Aineq.numRows);
         Assert.assertEquals(size, input.Aineq.numCols);
         Assert.assertEquals(numberOfConstraints, input.bineq.numRows);
         Assert.assertEquals(1, input.bineq.numCols);
      }
   }

   @Test
   public void testReshape()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         int numberOfConstraints = RandomNumbers.nextInt(random, 1, 200);

         ICPInequalityInput input = new ICPInequalityInput(numberOfConstraints, size);

         Assert.assertEquals(numberOfConstraints, input.Aineq.numRows);
         Assert.assertEquals(size, input.Aineq.numCols);
         Assert.assertEquals(numberOfConstraints, input.bineq.numRows);
         Assert.assertEquals(1, input.bineq.numCols);

         int newSize = RandomNumbers.nextInt(random, 1, 200);
         int newNumberOfConstraints = RandomNumbers.nextInt(random, 1, 200);

         input.reshape(newNumberOfConstraints, newSize);

         Assert.assertEquals(newNumberOfConstraints, input.Aineq.numRows);
         Assert.assertEquals(newSize, input.Aineq.numCols);
         Assert.assertEquals(newNumberOfConstraints, input.bineq.numRows);
         Assert.assertEquals(1, input.bineq.numCols);

         Assert.assertEquals(newNumberOfConstraints, input.getNumberOfConstraints());
         Assert.assertEquals(newSize, input.getNumberOfVariables());
      }
   }


   @Test
   public void testReset()
   {
      Random random = new Random(10L);

      for (int i = 0; i < 1000; i++)
      {
         int size = RandomNumbers.nextInt(random, 1, 200);
         int numberOfConstraints = RandomNumbers.nextInt(random, 1, 200);

         ICPInequalityInput input = new ICPInequalityInput(numberOfConstraints, size);

         CommonOps_DDRM.fill(input.Aineq, RandomNumbers.nextDouble(random, -1000.0, 1000.0));
         CommonOps_DDRM.fill(input.bineq, RandomNumbers.nextDouble(random, -1000.0, 1000.0));

         Assert.assertNotEquals(CommonOps_DDRM.elementSum(input.Aineq), 0.0, 1e-7);
         Assert.assertNotEquals(CommonOps_DDRM.elementSum(input.bineq), 0.0, 1e-7);

         input.reset();

         Assert.assertEquals(CommonOps_DDRM.elementSum(input.Aineq), 0.0, 1e-7);
         Assert.assertEquals(CommonOps_DDRM.elementSum(input.bineq), 0.0, 1e-7);
      }
   }




}
