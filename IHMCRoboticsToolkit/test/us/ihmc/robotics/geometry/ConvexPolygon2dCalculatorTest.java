package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = {TestPlanTarget.Fast})
public class ConvexPolygon2dCalculatorTest
{
   private static final double epsilon = 1.0e-10;
   private final ConvexPolygon2dCalculator calculator = new ConvexPolygon2dCalculator();

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSignedDistance1()
   {
      // single point polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.update();

      Point2d point = new Point2d(2.5, 1.0);
      double distance = calculator.getSignedDistance(point, polygon);
      assertDistanceCorrect(-Math.sqrt(2.5*2.5 + 1.0*1.0), distance);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSignedDistance2()
   {
      // line polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(1.0, 0.0));
      polygon.update();

      Point2d point1 = new Point2d(2.5, 1.0);
      double distance1 = calculator.getSignedDistance(point1, polygon);
      assertDistanceCorrect(-Math.sqrt(1.5*1.5 + 1.0*1.0), distance1);

      Point2d point2 = new Point2d(0.5, 1.0);
      double distance2 = calculator.getSignedDistance(point2, polygon);
      assertDistanceCorrect(-1.0, distance2);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSignedDistance3()
   {
      // triangle polygon
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      polygon.addVertex(new Point2d(0.0, 0.0));
      polygon.addVertex(new Point2d(10.0, 0.0));
      polygon.addVertex(new Point2d(0.0, 10.0));
      polygon.update();

      Point2d point1 = new Point2d(10.0, 10.0);
      double distance1 = calculator.getSignedDistance(point1, polygon);
      assertDistanceCorrect(-5.0 * Math.sqrt(2.0), distance1);

      Point2d point2 = new Point2d(1.2, 1.1);
      double distance2 = calculator.getSignedDistance(point2, polygon);
      assertDistanceCorrect(1.1, distance2);

      Point2d point3 = new Point2d(0.05, 9.8);
      double distance3 = calculator.getSignedDistance(point3, polygon);
      assertDistanceCorrect(0.05, distance3);

      Point2d point4 = new Point2d(9.8, 0.15);
      double distance4 = calculator.getSignedDistance(point4, polygon);
      assertDistanceCorrect(0.5 * Math.sqrt(0.05 * 0.05 * 2.0), distance4);

      Point2d point5 = new Point2d(5.0, -0.15);
      double distance5 = calculator.getSignedDistance(point5, polygon);
      assertDistanceCorrect(-0.15, distance5);

      Point2d point6 = new Point2d(15.0, -0.15);
      double distance6 = calculator.getSignedDistance(point6, polygon);
      assertDistanceCorrect(-Math.sqrt(5.0 * 5.0 + 0.15 * 0.15), distance6);
   }

   private static void assertDistanceCorrect(double expected, double actual)
   {
      assertEquals("Distance to point did not equal expected", expected, actual, epsilon);
   }

   public static void main(String[] args)
   {
      String targetTests = "us.ihmc.robotics.geometry.ConvexPolygon2dCalculatorTest";
      String targetClasses = "us.ihmc.robotics.geometry.ConvexPolygon2dCalculator";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
