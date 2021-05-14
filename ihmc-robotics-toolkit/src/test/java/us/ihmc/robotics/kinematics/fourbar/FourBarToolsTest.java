package us.ihmc.robotics.kinematics.fourbar;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.log.LogTools;

public class FourBarToolsTest
{
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-11;
   private static final double FD_DOT_EPSILON = 1.0e-6;
   private static final double FD_DDOT_EPSILON = 5.0e-4;

   @Test
   public void testFastAcos()
   {
      for (double x = -1.0; x < 1.0; x += 1.0e-6)
      {
         double expected = Math.acos(x);
         double actual = FourBarTools.fastAcos(x);
         assertEquals(expected, actual, 1.0e-14, "x=" + x);
      }

      assertEquals(Math.acos(-1.0), FourBarTools.fastAcos(-1.0), 1.0e-14, "x=" + (-1.0));
      assertEquals(Math.acos(1.0), FourBarTools.fastAcos(1.0), 1.0e-14, "x=" + (1.0));
   }

   @Test
   public void testAngleWithCosineLaw()
   {
      Random random = new Random(4643675);

      for (int i = 0; i < ITERATIONS; i++)
      { // Build right triangle from expected angle and one random side length
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, 1.0e-3, 0.5 * Math.PI - 1.0e-3);
         double AB = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         // Triangle's right angle is at B => AC is the hypotenuse.
         double BC = AB * Math.tan(expectedAngle);
         double AC = Math.hypot(AB, BC);

         double actualAngle = FourBarTools.angleWithCosineLaw(AB, AC, BC);

         assertEquals(expectedAngle, actualAngle, EPSILON, "error=" + Math.abs(expectedAngle - actualAngle));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Build triangle using points and vectors, compare the result with the Vector2DReadOnly.angle(...).
         Point2D A = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D B = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D C = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Vector2D AB = new Vector2D();
         Vector2D AC = new Vector2D();
         Vector2D BC = new Vector2D();

         AB.sub(B, A);
         AC.sub(C, A);
         BC.sub(C, B);

         double expectedAngle = Math.abs(AB.angle(AC));
         double actualAngle = FourBarTools.angleWithCosineLaw(AB.length(), AC.length(), BC.length());

         assertEquals(expectedAngle, actualAngle, EPSILON, "error=" + Math.abs(expectedAngle - actualAngle));
      }
   }

   @Test
   public void testAngleDotWithCosineLaw(TestInfo testInfo)
   {
      Random random = new Random(4643675);

      double dt = 3.0e-6;
      {
         double averageError = 0.0;

         for (int i = 0; i < ITERATIONS; i++)
         { // Build random triangle and compare result against finite difference.
            Point2D A = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            Point2D B = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            Point2D C = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

            Vector2D CDot = EuclidCoreRandomTools.nextVector2D(random, -10.0, 10.0);
            Point2D CFuture = new Point2D();
            CFuture.scaleAdd(0.5 * dt, CDot, C);
            Point2D CPast = new Point2D();
            CPast.scaleAdd(-0.5 * dt, CDot, C);

            double ABLength = A.distance(B);
            double ACLength = A.distance(C);
            double BCLength = B.distance(C);

            double ACLengthFuture = A.distance(CFuture);
            double BCLengthFuture = B.distance(CFuture);
            double ACLengthPast = A.distance(CPast);
            double BCLengthPast = B.distance(CPast);

            double ACLengthDot = (ACLengthFuture - ACLengthPast) / dt;
            double BCLengthDot = (BCLengthFuture - BCLengthPast) / dt;

            double anglePast = FourBarTools.angleWithCosineLaw(ABLength, ACLengthPast, BCLengthPast);
            double angleFuture = FourBarTools.angleWithCosineLaw(ABLength, ACLengthFuture, BCLengthFuture);
            double expectedAngleDot = (angleFuture - anglePast) / dt;

            double actualAngleDot = FourBarTools.angleDotWithCosineLaw(ABLength, ACLength, ACLengthDot, BCLength, BCLengthDot);

            double error = Math.abs(expectedAngleDot - actualAngleDot);
            averageError += error;

            assertEquals(expectedAngleDot, actualAngleDot, FD_DOT_EPSILON * Math.max(1.0, Math.abs(expectedAngleDot)), "error=" + error);
         }

         averageError /= ITERATIONS;
         LogTools.info(testInfo.getTestMethod().get().getName() + ": Average error = " + averageError);
      }
   }

   @Test
   public void testAngleDDotWithCosineLaw(TestInfo testInfo)
   {
      Random random = new Random(4643675);

      double dt = 1.0e-6;
      {
         double averageError = 0.0;

         for (int i = 0; i < ITERATIONS; i++)
         { // Build random triangle and compare result against finite difference.
            Point2D A = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            Point2D B = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            Point2D C = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

            Vector2D CDot = EuclidCoreRandomTools.nextVector2D(random, -10.0, 10.0);
            Vector2D CDDot = EuclidCoreRandomTools.nextVector2D(random, -10.0, 10.0);

            Vector2D CDotPast = new Vector2D();
            CDotPast.scaleAdd(-0.5 * dt, CDDot, CDot);
            Vector2D CDotFuture = new Vector2D();
            CDotFuture.scaleAdd(0.5 * dt, CDDot, CDot);

            Point2D CPast = new Point2D();
            CPast.scaleAdd(-0.5 * dt, CDot, C);
            CPast.scaleAdd(-0.25 * dt * dt, CDDot, CPast);
            Point2D CFuture = new Point2D();
            CFuture.scaleAdd(0.5 * dt, CDot, C);
            CFuture.scaleAdd(0.25 * dt * dt, CDDot, CFuture);

            double AB = A.distance(B);
            double AC = A.distance(C);
            double BC = B.distance(C);
            double ACDot = projectOntoSide(CDot, A, C);
            double BCDot = projectOntoSide(CDot, B, C);
            double ACDDot = projectOntoSide(CDDot, A, C);
            double BCDDot = projectOntoSide(CDDot, B, C);

            double ACPast = A.distance(CPast);
            double BCPast = B.distance(CPast);
            double ACFuture = A.distance(CFuture);
            double BCFuture = B.distance(CFuture);

            double ACDotPast = projectOntoSide(CDotPast, A, C);
            double BCDotPast = projectOntoSide(CDotPast, B, C);
            double ACDotFuture = projectOntoSide(CDotFuture, A, C);
            double BCDotFuture = projectOntoSide(CDotFuture, B, C);

            double angleDotPast = FourBarTools.angleDotWithCosineLaw(AB, ACPast, ACDotPast, BCPast, BCDotPast);
            double angleDotFuture = FourBarTools.angleDotWithCosineLaw(AB, ACFuture, ACDotFuture, BCFuture, BCDotFuture);
            double expectedAngleDDot = (angleDotFuture - angleDotPast) / dt;

            double actualAngleDDot = FourBarTools.angleDDotWithCosineLaw(AB, AC, ACDot, ACDDot, BC, BCDot, BCDDot);

            double error = Math.abs(expectedAngleDDot - actualAngleDDot);
            averageError += error;

            assertEquals(expectedAngleDDot, actualAngleDDot, FD_DDOT_EPSILON * Math.max(1.0, Math.abs(expectedAngleDDot)), "error=" + error);
         }

         averageError /= ITERATIONS;
         LogTools.info(testInfo.getTestMethod().get().getName() + ": Average error = " + averageError);
      }
   }

   private static double projectOntoSide(Vector2DReadOnly vectorToProject, Point2DReadOnly sideStart, Point2DReadOnly sideEnd)
   {
      Vector2D side = new Vector2D();
      side.sub(sideEnd, sideStart);
      side.normalize();
      return vectorToProject.dot(side);
   }
}
