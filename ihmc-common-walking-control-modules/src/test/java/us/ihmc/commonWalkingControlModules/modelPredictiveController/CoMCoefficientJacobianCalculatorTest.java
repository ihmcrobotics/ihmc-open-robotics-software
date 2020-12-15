package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

public class CoMCoefficientJacobianCalculatorTest
{
   @Test
   public void testCoMOneSegment()
   {
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         DMatrixRMaj positionExpected = new DMatrixRMaj(3, 6);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(3, 6);
         DMatrixRMaj accelerationExpected = new DMatrixRMaj(3, 6);

         DMatrixRMaj position = new DMatrixRMaj(3, 6);
         DMatrixRMaj velocity = new DMatrixRMaj(3, 6);
         DMatrixRMaj acceleration = new DMatrixRMaj(3, 6);

         CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, time, position, 0, 1.0);
         CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, time, velocity, 1, 1.0);
         CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, time, acceleration, 2, 1.0);

         double c0 = time;
         double c1 = 1.0;

         double c0Dot = 1.0;
         double c1Dot = 0.0;

         for (int i = 0; i < 3; i++)
         {
            positionExpected.set(i, 2 * i, c0);
            positionExpected.set(i, 2 * i + 1, c1);

            velocityExpected.set(i, 2 * i, c0Dot);
            velocityExpected.set(i, 2 * i + 1, c1Dot);
         }

         EjmlUnitTests.assertEquals(positionExpected, position, 1e-6);
         EjmlUnitTests.assertEquals(velocityExpected, velocity, 1e-6);
         EjmlUnitTests.assertEquals(accelerationExpected, acceleration, 1e-6);
      }
   }

   @Test
   public void testCoMFourSegments()
   {
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         DMatrixRMaj positionExpected = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj accelerationExpected = new DMatrixRMaj(3, 4 * 6);

         DMatrixRMaj position = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj velocity = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj acceleration = new DMatrixRMaj(3, 4 * 6);

         for (int segment = 0; segment < 4; segment++)
         {
            int startIndex = 6 * segment;
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(startIndex, time, position, 0, 1.0);
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(startIndex, time, velocity, 1, 1.0);
            CoMCoefficientJacobianCalculator.calculateCoMJacobian(startIndex, time, acceleration, 2, 1.0);

            double c0 = time;
            double c1 = 1.0;

            double c0Dot = 1.0;
            double c1Dot = 0.0;

            for (int i = 0; i < 3; i++)
            {
               positionExpected.set(i, segment * 6 + 2 * i, c0);
               positionExpected.set(i, segment * 6 + 2 * i + 1, c1);

               velocityExpected.set(i, segment * 6 + 2 * i, c0Dot);
               velocityExpected.set(i, segment * 6 + 2 * i + 1, c1Dot);
            }
         }

         EjmlUnitTests.assertEquals(positionExpected, position, 1e-6);
         EjmlUnitTests.assertEquals(velocityExpected, velocity, 1e-6);
         EjmlUnitTests.assertEquals(accelerationExpected, acceleration, 1e-6);
      }
   }

   @Test
   public void testDCMOneSegment()
   {
      double omega = 3.0;
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         DMatrixRMaj positionExpected = new DMatrixRMaj(3, 6);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(3, 6);
         DMatrixRMaj accelerationExpected = new DMatrixRMaj(3, 6);

         DMatrixRMaj position = new DMatrixRMaj(3, 6);
         DMatrixRMaj velocity = new DMatrixRMaj(3, 6);
         DMatrixRMaj acceleration = new DMatrixRMaj(3, 6);

         CoMCoefficientJacobianCalculator.calculateDCMJacobian(0, omega, time, position, 0, 1.0);
         CoMCoefficientJacobianCalculator.calculateDCMJacobian(0, omega, time, velocity, 1, 1.0);
         CoMCoefficientJacobianCalculator.calculateDCMJacobian(0, omega, time, acceleration, 2, 1.0);

         double c0 = time + 1.0 / omega;
         double c1 = 1.0;

         double c0Dot = 1.0;
         double c1Dot = 0.0;

         for (int i = 0; i < 3; i++)
         {
            positionExpected.set(i, 2 * i, c0);
            positionExpected.set(i, 2 * i + 1, c1);

            velocityExpected.set(i, 2 * i, c0Dot);
            velocityExpected.set(i, 2 * i + 1, c1Dot);
         }

         EjmlUnitTests.assertEquals(positionExpected, position, 1e-6);
         EjmlUnitTests.assertEquals(velocityExpected, velocity, 1e-6);
         EjmlUnitTests.assertEquals(accelerationExpected, acceleration, 1e-6);
      }
   }

   @Test
   public void testDCMFourSegments()
   {
      double omega = 3.0;
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         DMatrixRMaj positionExpected = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj accelerationExpected = new DMatrixRMaj(3, 4 * 6);

         DMatrixRMaj position = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj velocity = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj acceleration = new DMatrixRMaj(3, 4 * 6);

         for (int segment = 0; segment < 4; segment++)
         {
            int comStartCol = segment * 6;
            CoMCoefficientJacobianCalculator.calculateDCMJacobian(comStartCol, omega, time, position, 0, 1.0);
            CoMCoefficientJacobianCalculator.calculateDCMJacobian(comStartCol, omega, time, velocity, 1, 1.0);
            CoMCoefficientJacobianCalculator.calculateDCMJacobian(comStartCol, omega, time, acceleration, 2, 1.0);

            double c0 = time + 1.0 / omega;
            double c1 = 1.0;

            double c0Dot = 1.0;
            double c1Dot = 0.0;

            for (int i = 0; i < 3; i++)
            {
               positionExpected.set(i, segment * 6 + 2 * i, c0);
               positionExpected.set(i, segment * 6 + 2 * i + 1, c1);

               velocityExpected.set(i, segment * 6 + 2 * i, c0Dot);
               velocityExpected.set(i, segment * 6 + 2 * i + 1, c1Dot);
            }
         }

         EjmlUnitTests.assertEquals(positionExpected, position, 1e-6);
         EjmlUnitTests.assertEquals(velocityExpected, velocity, 1e-6);
         EjmlUnitTests.assertEquals(accelerationExpected, acceleration, 1e-6);
      }
   }

   @Test
   public void testVRPOneSegment()
   {
      double omega = 3.0;
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         DMatrixRMaj positionExpected = new DMatrixRMaj(3, 6);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(3, 6);

         DMatrixRMaj position = new DMatrixRMaj(3, 6);
         DMatrixRMaj velocity = new DMatrixRMaj(3, 6);

         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, position, 0, 1.0);
         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, velocity, 1, 1.0);

         double c0 = time;
         double c1 = 1.0;

         double c0Dot = 1.0;
         double c1Dot = 0.0;

         for (int i = 0; i < 3; i++)
         {
            positionExpected.set(i, 2 * i, c0);
            positionExpected.set(i, 2 * i + 1, c1);

            velocityExpected.set(i, 2 * i, c0Dot);
            velocityExpected.set(i, 2 * i + 1, c1Dot);
         }

         EjmlUnitTests.assertEquals(positionExpected, position, 1e-6);
         EjmlUnitTests.assertEquals(velocityExpected, velocity, 1e-6);
      }
   }

   @Test
   public void testVRPFourSegments()
   {
      double omega = 3.0;
      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         DMatrixRMaj positionExpected = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj velocityExpected = new DMatrixRMaj(3, 4 * 6);

         DMatrixRMaj position = new DMatrixRMaj(3, 4 * 6);
         DMatrixRMaj velocity = new DMatrixRMaj(3, 4 * 6);

         for (int segment = 0; segment < 4; segment++)
         {
            int comStartIndex = 6 * segment;
            CoMCoefficientJacobianCalculator.calculateVRPJacobian(comStartIndex, omega, time, position, 0, 1.0);
            CoMCoefficientJacobianCalculator.calculateVRPJacobian(comStartIndex, omega, time, velocity, 1, 1.0);

            double c0 = time;
            double c1 = 1.0;

            double c0Dot = 1.0;
            double c1Dot = 0.0;

            for (int i = 0; i < 3; i++)
            {
               positionExpected.set(i, segment * 6 + 2 * i, c0);
               positionExpected.set(i, segment * 6 + 2 * i + 1, c1);

               velocityExpected.set(i, segment * 6 + 2 * i, c0Dot);
               velocityExpected.set(i, segment * 6 + 2 * i + 1, c1Dot);
            }
         }

         EjmlUnitTests.assertEquals(positionExpected, position, 1e-6);
         EjmlUnitTests.assertEquals(velocityExpected, velocity, 1e-6);
      }
   }
}
