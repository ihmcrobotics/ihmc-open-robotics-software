package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

public class CoefficientJacobianMatrixHelperTest
{
   @Test
   public void testCalculationWithOneContact()
   {
      double omega = 3.0;
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(1);
         helper.computeMatrices(time, omega);

         DMatrixRMaj positionExpected = new DMatrixRMaj(4, 16);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(4, 16);
         DMatrixRMaj accelerationExpected = new DMatrixRMaj(4, 16);

         double c0 = Math.exp(omega * time);
         double c1 = Math.exp(-omega * time);
         double c2 = time * time * time;
         double c3 = time * time;

         double c0Dot = omega * Math.exp(omega * time);
         double c1Dot = -omega * Math.exp(-omega * time);
         double c2Dot = 3 * time * time;
         double c3Dot = 2 * time;

         double c0Ddot = omega * omega * Math.exp(omega * time);
         double c1Ddot = omega * omega * Math.exp(-omega * time);
         double c2Ddot = 6 * time;
         double c3Ddot = 2;

         for (int i = 0; i < 4; i++)
         {
            positionExpected.set(i, 4 * i, c0);
            positionExpected.set(i, 4 * i + 1, c1);
            positionExpected.set(i, 4 * i + 2, c2);
            positionExpected.set(i, 4 * i + 3, c3);

            velocityExpected.set(i, 4 * i, c0Dot);
            velocityExpected.set(i, 4 * i + 1, c1Dot);
            velocityExpected.set(i, 4 * i + 2, c2Dot);
            velocityExpected.set(i, 4 * i + 3, c3Dot);

            accelerationExpected.set(i, 4 * i, c0Ddot);
            accelerationExpected.set(i, 4 * i + 1, c1Ddot);
            accelerationExpected.set(i, 4 * i + 2, c2Ddot);
            accelerationExpected.set(i, 4 * i + 3, c3Ddot);
         }

         EjmlUnitTests.assertEquals(positionExpected, helper.getPositionJacobianMatrix(), 1e-4);
         EjmlUnitTests.assertEquals(velocityExpected, helper.getVelocityJacobianMatrix(), 1e-4);
         EjmlUnitTests.assertEquals(accelerationExpected, helper.getAccelerationJacobianMatrix(), 1e-4);
      }
   }

   @Test
   public void testCalculationWithFourContacts()
   {
      double omega = 3.0;
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(4);
         helper.computeMatrices(time, omega);

         DMatrixRMaj positionExpected = new DMatrixRMaj(4 * 4, 4 * 16);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(4 * 4, 4 * 16);
         DMatrixRMaj accelerationExpected = new DMatrixRMaj(4 * 4, 4 * 16);

         double c0 = Math.exp(omega * time);
         double c1 = Math.exp(-omega * time);
         double c2 = time * time * time;
         double c3 = time * time;

         double c0Dot = omega * Math.exp(omega * time);
         double c1Dot = -omega * Math.exp(-omega * time);
         double c2Dot = 3 * time * time;
         double c3Dot = 2 * time;

         double c0Ddot = omega * omega * Math.exp(omega * time);
         double c1Ddot = omega * omega * Math.exp(-omega * time);
         double c2Ddot = 6 * time;
         double c3Ddot = 2;

         for (int i = 0; i < 16; i++)
         {
            positionExpected.set(i, 4 * i, c0);
            positionExpected.set(i, 4 * i + 1, c1);
            positionExpected.set(i, 4 * i + 2, c2);
            positionExpected.set(i, 4 * i + 3, c3);

            velocityExpected.set(i, 4 * i, c0Dot);
            velocityExpected.set(i, 4 * i + 1, c1Dot);
            velocityExpected.set(i, 4 * i + 2, c2Dot);
            velocityExpected.set(i, 4 * i + 3, c3Dot);

            accelerationExpected.set(i, 4 * i, c0Ddot);
            accelerationExpected.set(i, 4 * i + 1, c1Ddot);
            accelerationExpected.set(i, 4 * i + 2, c2Ddot);
            accelerationExpected.set(i, 4 * i + 3, c3Ddot);
         }

         EjmlUnitTests.assertEquals(positionExpected, helper.getPositionJacobianMatrix(), 1e-4);
         EjmlUnitTests.assertEquals(velocityExpected, helper.getVelocityJacobianMatrix(), 1e-4);
         EjmlUnitTests.assertEquals(accelerationExpected, helper.getAccelerationJacobianMatrix(), 1e-4);
      }
   }
}
