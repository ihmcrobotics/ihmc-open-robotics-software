package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class TriangleToolsTest
{
   private static final double epsilon = 1e-4;

   @Test(timeout = 30000)
   public void testReturnsTheSame()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-2, Math.PI - 1e-2);

         double farSideLength = TriangleTools.computeSideLength(sideALength, sideBLength, interiorAngle);
         assertEquals("Iteration " + iter + " length failed.", interiorAngle, TriangleTools.computeInteriorAngle(sideALength, sideBLength, farSideLength), epsilon);

         double interiorAngleVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double farSideVelocity = TriangleTools.computeSideLengthVelocity(sideALength, sideBLength, interiorAngle, interiorAngleVelocity);
         assertEquals("Iteration " + iter + " velocity failed.", interiorAngleVelocity, TriangleTools.computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength, farSideVelocity), epsilon);

         double interiorAngleAcceleration = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double farSideAcceleration = TriangleTools
               .computeSideLengthAcceleration(sideALength, sideBLength, interiorAngle, interiorAngleVelocity, interiorAngleAcceleration);
         assertEquals("Iteration " + iter + " acceleration failed.", interiorAngleAcceleration,
                      TriangleTools.computeInteriorAngleAcceleration(sideALength, sideBLength, farSideLength, farSideVelocity, farSideAcceleration), epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindInteriorAngle()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double farSideLength = RandomNumbers.nextDouble(random, 0.1, 0.95 * (sideALength + sideBLength));

         double lengthDifference = Math.pow(farSideLength, 2.0) - Math.pow(sideALength, 2.0) - Math.pow(sideBLength, 2.0);
         double interiorAngle = Math.acos(-lengthDifference / 2.0 / sideALength / sideBLength);

         assertEquals(interiorAngle, TriangleTools.computeInteriorAngle(sideALength, sideBLength, farSideLength), epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindInteriorAngleVelocity()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double farSideLength = RandomNumbers.nextDouble(random, 0.1, 0.95 * (sideALength + sideBLength));
         double farSideLengthVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);

         double lengthDifference = Math.pow(farSideLength, 2.0) - Math.pow(sideALength, 2.0) - Math.pow(sideBLength, 2.0);
         double interiorAngle = Math.acos(-lengthDifference / 2.0 / sideALength / sideBLength);

         double interiorAngleVelocity = 2.0 * farSideLength * farSideLengthVelocity / 2.0 / sideALength / sideBLength / Math.sin(interiorAngle);

         assertEquals(interiorAngleVelocity, TriangleTools.computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength, farSideLengthVelocity),
                      epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindInteriorVelocityNumerically()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double farSideLength = RandomNumbers.nextDouble(random, 0.1, 0.95 * (sideALength + sideBLength));
         double farSideLengthVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);

         double dt = 0.0001;
         double angle = TriangleTools.computeInteriorAngle(sideALength, sideBLength, farSideLength);
         double anglePlus = TriangleTools.computeInteriorAngle(sideALength, sideBLength, farSideLength + dt * farSideLengthVelocity);
         double angleMinus = TriangleTools.computeInteriorAngle(sideALength, sideBLength, farSideLength - dt * farSideLengthVelocity);

         double velocityPlus = (anglePlus - angle) / dt;
         double velocityMinus = (angle - angleMinus) / dt;
         double velocityPlusMinus = (anglePlus - angleMinus) / (2 * dt);

         double numericalVelocity = (velocityPlus + velocityMinus + velocityPlusMinus) / 3.0;

         assertEquals(numericalVelocity, TriangleTools.computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength, farSideLengthVelocity), 1e-2);
      }
   }

   @Test(timeout = 30000)
   public void testFindInteriorAngleAccelerationA()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double farSideLength = RandomNumbers.nextDouble(random, 0.1, 0.95 * (sideALength + sideBLength));
         double farSideLengthVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double farSideLengthAcceleration = RandomNumbers.nextDouble(random, -100.0, 100.0);

         double lengthDifference = Math.pow(farSideLength, 2.0) - Math.pow(sideALength, 2.0) - Math.pow(sideBLength, 2.0);
         double interiorAngle = Math.acos(-lengthDifference / 2.0 / sideALength / sideBLength);
         double interiorAngleVelocity = 2.0 * farSideLength * farSideLengthVelocity / 2.0 / sideALength / sideBLength / Math.sin(interiorAngle);

         double interiorAngleAcceleration =
               1.0 / (sideALength * sideBLength * Math.sin(interiorAngle)) * (Math.pow(farSideLengthVelocity, 2.0) + farSideLength * farSideLengthAcceleration)
                     - Math.pow(interiorAngleVelocity, 2.0) * Math.cos(interiorAngle) / Math.sin(interiorAngle);

         assertEquals(interiorAngleAcceleration,
                      TriangleTools.computeInteriorAngleAcceleration(sideALength, sideBLength, farSideLength, farSideLengthVelocity, farSideLengthAcceleration),
                      epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindInteriorAngleAccelerationB()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double farSideLength = RandomNumbers.nextDouble(random, 0.1, 0.95 * (sideALength + sideBLength));
         double farSideLengthVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double farSideLengthAcceleration = RandomNumbers.nextDouble(random, -100.0, 100.0);

         double lengthDifference = Math.pow(farSideLength, 2.0) - Math.pow(sideALength, 2.0) - Math.pow(sideBLength, 2.0);
         double interiorAngle = Math.acos(-lengthDifference / 2.0 / sideALength / sideBLength);
         double interiorAngleVelocity = 2.0 * farSideLength * farSideLengthVelocity / 2.0 / sideALength / sideBLength / Math.sin(interiorAngle);

         double interiorAngleAcceleration = farSideLengthAcceleration * Math.pow(farSideLength, 2.0) / (sideALength * sideBLength) - Math.cos(interiorAngle) * Math.pow(interiorAngleVelocity, 2.0) * farSideLength + farSideLengthVelocity * interiorAngleVelocity * Math.sin(interiorAngle);
         interiorAngleAcceleration /= Math.sin(interiorAngle) * farSideLength;

         assertEquals(interiorAngleAcceleration,
                      TriangleTools.computeInteriorAngleAcceleration(sideALength, sideBLength, farSideLength, farSideLengthVelocity, farSideLengthAcceleration),
                      epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindInteriorAccelerationNumerically()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double farSideLength = RandomNumbers.nextDouble(random, 0.1, 0.95 * (sideALength + sideBLength));
         double farSideLengthVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double farSideLengthAcceleration = RandomNumbers.nextDouble(random, -100.0, 100.0);

         double dt = 0.00001;
         double angleVelocity = TriangleTools.computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength, farSideLengthVelocity);
         double angleVelocityPlus = TriangleTools
               .computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength + farSideLengthVelocity * dt  + 0.5 * farSideLengthAcceleration * dt * dt, farSideLengthVelocity + dt * farSideLengthAcceleration);
         double angleVelocityMinus = TriangleTools
               .computeInteriorAngleVelocity(sideALength, sideBLength, farSideLength - farSideLengthVelocity * dt  + 0.5 * farSideLengthAcceleration * dt * dt, farSideLengthVelocity - dt * farSideLengthAcceleration);

         double accelerationPlus = (angleVelocityPlus - angleVelocity) / dt;
         double accelerationMinus = (angleVelocity - angleVelocityMinus) / dt;
         double accelerationPlusMinus = (angleVelocityPlus - angleVelocityMinus) / (2 * dt);

         double numericalAcceleration = (accelerationPlus + accelerationMinus + accelerationPlusMinus) / 3.0;

         assertEquals("Iteration " + iter + " failed.", numericalAcceleration,
                      TriangleTools.computeInteriorAngleAcceleration(sideALength, sideBLength, farSideLength, farSideLengthVelocity, farSideLengthAcceleration),
                      1e-1);
      }
   }

   @Test(timeout = 30000)
   public void testFindFarSideLength()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-3, Math.PI - 1e-3);

         double farSideLengthSquared = Math.pow(sideALength, 2.0) + Math.pow(sideBLength, 2.0) - 2.0 * sideALength * sideBLength * Math.cos(interiorAngle);
         double farSideLength = Math.sqrt(farSideLengthSquared);

         assertEquals(farSideLength, TriangleTools.computeSideLength(sideALength, sideBLength, interiorAngle), epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindFarSideVelocity()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-3, Math.PI - 1e-3);
         double interiorAngleVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);

         double farSideLengthSquared = Math.pow(sideALength, 2.0) + Math.pow(sideBLength, 2.0) - 2.0 * sideALength * sideBLength * Math.cos(interiorAngle);
         double farSideLength = Math.sqrt(farSideLengthSquared);
         double velocityExpected = 2.0 * sideALength * sideBLength * interiorAngleVelocity * Math.sin(interiorAngle) / 2.0 / farSideLength;

         assertEquals(velocityExpected, TriangleTools.computeSideLengthVelocity(sideALength, sideBLength, interiorAngle, interiorAngleVelocity), epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindFarSideVelocityNumerically()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-3, Math.PI - 1e-3);
         double interiorAngleVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);

         double dt = 0.0001;
         double length = TriangleTools.computeSideLength(sideALength, sideBLength, interiorAngle);
         double lengthPlus = TriangleTools.computeSideLength(sideALength, sideBLength, interiorAngle + dt * interiorAngleVelocity);
         double lengthMinus = TriangleTools.computeSideLength(sideALength, sideBLength, interiorAngle - dt * interiorAngleVelocity);

         double velocityPlus = (lengthPlus - length) / dt;
         double velocityMinus = (length - lengthMinus) / dt;
         double velocityPlusMinus = (lengthPlus - lengthMinus) / (2 * dt);

         double numericalVelocity = (velocityPlus + velocityMinus + velocityPlusMinus) / 3.0;

         assertEquals(numericalVelocity, TriangleTools.computeSideLengthVelocity(sideALength, sideBLength, interiorAngle, interiorAngleVelocity), 1e-2);
      }
   }

   @Test(timeout = 30000)
   public void testFindFarSideAccelerationA()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-3, Math.PI - 1e-3);
         double interiorAngleVelocity = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double interiorAngleAcceleration = RandomNumbers.nextDouble(random, -1000.0, 1000.0);

         double farSideLengthSquared = Math.pow(sideALength, 2.0) + Math.pow(sideBLength, 2.0) - 2.0 * sideALength * sideBLength * Math.cos(interiorAngle);
         double farSideLength = Math.sqrt(farSideLengthSquared);

         double farSideLengthVelocity = 2.0 * sideALength * sideBLength * interiorAngleVelocity * Math.sin(interiorAngle) / 2.0 / farSideLength;

         double farSideAcceleration =
               sideALength * sideBLength / farSideLength * (interiorAngleAcceleration * Math.sin(interiorAngle) + Math.pow(interiorAngleVelocity, 2.0) * Math
                     .cos(interiorAngle));
         farSideAcceleration -= Math.pow(farSideLengthVelocity, 2.0) / farSideLength;

         assertEquals(farSideAcceleration,
                      TriangleTools.computeSideLengthAcceleration(sideALength, sideBLength, interiorAngle, interiorAngleVelocity, interiorAngleAcceleration),
                      epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindFarSideAccelerationB()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-3, Math.PI - 1e-3);
         double interiorAngleVelocity = RandomNumbers.nextDouble(random, -100.0, 100.0);
         double interiorAngleAcceleration = RandomNumbers.nextDouble(random, -1000.0, 1000.0);

         double farSideLengthSquared = Math.pow(sideALength, 2.0) + Math.pow(sideBLength, 2.0) - 2.0 * sideALength * sideBLength * Math.cos(interiorAngle);
         double farSideLength = Math.sqrt(farSideLengthSquared);

         double farSideLengthVelocity = 2.0 * sideALength * sideBLength * interiorAngleVelocity * Math.sin(interiorAngle) / 2.0 / farSideLength;

         double farSideAcceleration =
               Math.pow(interiorAngleVelocity, 2.0) * Math.cos(interiorAngle) * farSideLength + interiorAngleAcceleration * Math.sin(interiorAngle)
                     * farSideLength - farSideLengthVelocity * interiorAngleVelocity * Math.sin(interiorAngle);
         farSideAcceleration *= sideALength * sideBLength / Math.pow(farSideLength, 2.0);

         assertEquals(farSideAcceleration,
                      TriangleTools.computeSideLengthAcceleration(sideALength, sideBLength, interiorAngle, interiorAngleVelocity, interiorAngleAcceleration),
                      epsilon);
      }
   }

   @Test(timeout = 30000)
   public void testFindFarSideAccelerationNumerically()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double sideALength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double sideBLength = RandomNumbers.nextDouble(random, 0.1, 10.0);
         double interiorAngle = RandomNumbers.nextDouble(random, 0.0 + 1e-3, Math.PI - 1e-3);
         double interiorAngleVelocity = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double interiorAngleAcceleration = RandomNumbers.nextDouble(random, -100.0, 100.0);

         double dt = 0.00001;
         double velocity = TriangleTools.computeSideLengthVelocity(sideALength, sideBLength, interiorAngle, interiorAngleVelocity);
         double velocityPlus = TriangleTools
               .computeSideLengthVelocity(sideALength, sideBLength, interiorAngle + dt * interiorAngleVelocity + 0.5 * interiorAngleAcceleration * dt * dt,
                                          interiorAngleVelocity + dt * interiorAngleAcceleration);
         double velocityMinus = TriangleTools
               .computeSideLengthVelocity(sideALength, sideBLength, interiorAngle - dt * interiorAngleVelocity + 0.5 * interiorAngleAcceleration * dt * dt,
                                          interiorAngleVelocity - dt * interiorAngleAcceleration);

         double accelerationPlus = (velocityPlus - velocity) / dt;
         double accelerationMinus = (velocity - velocityMinus) / dt;
         double accelerationPlusMinus = (velocityPlus - velocityMinus) / (2 * dt);

         double numericalAcceleration = (accelerationPlus + accelerationMinus + accelerationPlusMinus) / 3.0;

         assertEquals("iteration " + iter + " failed.", numericalAcceleration,
                      TriangleTools.computeSideLengthAcceleration(sideALength, sideBLength, interiorAngle, interiorAngleVelocity, interiorAngleAcceleration),
                      1e-2);
      }
   }

}
