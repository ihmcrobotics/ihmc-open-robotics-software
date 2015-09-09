package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.Fast, TestPlanTarget.UI})
public class HumanEvaluationLine2dTest
{
   private final boolean WAIT_FOR_BUTTON_PUSH = false;
   private final double EPSILON_FOR_EQUALS = 1e-8;

   private Random random = new Random(100L);
   private Point2d point1 = new Point2d(random.nextDouble(), random.nextDouble());
   private Point2d point2 = new Point2d(random.nextDouble(), random.nextDouble());
   private Vector2d vector = new Vector2d(random.nextDouble() + 1.0, random.nextDouble());    // to ensure that the vector is not vertical (for the slope test)
   private Line2d line2dPointVector = new Line2d(point1, vector);
   private Line2d line2dPointPoint = new Line2d(point1, point2);
   private Line2d line2dLine2d = new Line2d(line2dPointPoint);

   @Before
   public void setUp() throws Exception
   {
      double epsilon = 1E-6;
      assertTrue("Normalized vector not normalized", Math.abs(line2dPointVector.normalizedVector.length() - 1.0) < epsilon);
      assertTrue("Normalized vector not normalized", Math.abs(line2dPointPoint.normalizedVector.length() - 1.0) < epsilon);
      assertTrue("Normalized vector not normalized", Math.abs(line2dLine2d.normalizedVector.length() - 1.0) < epsilon);

      assertEquals("Creating line from other line failed", line2dPointPoint.point.x, line2dLine2d.point.x, EPSILON_FOR_EQUALS);
      assertEquals("Creating line from other line failed", line2dPointPoint.point.y, line2dLine2d.point.y, EPSILON_FOR_EQUALS);
      assertEquals("Creating line from other line failed", line2dPointPoint.normalizedVector.x, line2dLine2d.normalizedVector.x, EPSILON_FOR_EQUALS);
      assertEquals("Creating line from other line failed", line2dPointPoint.normalizedVector.y, line2dLine2d.normalizedVector.y, EPSILON_FOR_EQUALS);

      assertNotSame("Line fields not copied", point1, line2dPointVector.point);
      assertNotSame("Line fields not copied", vector, line2dPointVector.normalizedVector);    // this would be really weird, but test anyway
      assertNotSame("Line fields not copied", point1, line2dPointPoint.point);
      assertNotSame("Line fields not copied", line2dPointPoint.normalizedVector, line2dLine2d.normalizedVector);
      assertNotSame("Line fields not copied", line2dPointPoint.point, line2dLine2d.point);

      boolean lineCreatedFromCoincidalPoints = true;
      try
      {
         // Try creating a line from two coincidal points.
         new Line2d(point1, new Point2d(point1));
      }
      catch (RuntimeException runtimeException)
      {
         lineCreatedFromCoincidalPoints = false;
      }

      assertFalse(lineCreatedFromCoincidalPoints);
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testIsOnLeftSideOfLine()
   {
	   TestPlanTarget.assumeRunningLocally();
	   
      double xMin = -1.0;
      double xMax = 1.0;
      double yMin = -1.0;
      double yMax = 1.0;

      ReferenceFrame someFrame = ReferenceFrame.constructARootFrame("someFrame", false, false, true);
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
      FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();

      Point2d randomPoint = new Point2d(xMin + (xMax - xMin) * random.nextDouble(), yMin + (yMax - yMin) * random.nextDouble());
      Vector2d randomVector = new Vector2d(random.nextDouble(), random.nextDouble());    // both are positive, so vector is always in upper right hand quadrant
      Line2d randomLine = new Line2d(randomPoint, randomVector);
      plotter.addFrameLine2d(new FrameLine2d(someFrame, randomLine));
      plotter.addFrameLine2d(new FrameLine2d(someFrame, randomLine.negateDirectionCopy()));

      Point2d testPoint = new Point2d();
      for (int i = 0; i < 1000; i++)
      {
         testPoint.set(xMin + (xMax - xMin) * random.nextDouble(), yMin + (yMax - yMin) * random.nextDouble());

         if (randomLine.isPointOnLeftSideOfLine(testPoint))
         {
            plotter.addFramePoint2d(new FramePoint2d(someFrame, testPoint), Color.blue);
         }

         if (randomLine.isPointOnRightSideOfLine(testPoint))
         {
            plotter.addFramePoint2d(new FramePoint2d(someFrame, testPoint), Color.orange);
         }

         assertFalse("Point neither on the right nor on the left side of the line",
                     !randomLine.isPointOnLeftSideOfLine(testPoint) &&!randomLine.isPointOnRightSideOfLine(testPoint));
      }

      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();

      testFrame.dispose();
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testIsInFrontOfLine()
   {
      TestPlanTarget.assumeRunningLocally();
      
      // illegal method call:
      Line2d line = new Line2d(new Point2d(0.0, 0.0), new Vector2d(1.0, 0.0));

      boolean exceptionThrown = false;
      try
      {
         line.isPointInFrontOfLine(new Point2d(random.nextDouble(), random.nextDouble()));
      }
      catch (RuntimeException e)
      {
         exceptionThrown = true;
      }

      assertTrue("Line pointing in front direction didn't throw an exception", exceptionThrown);

      // visual verification:
      double xMin = -1.0;
      double xMax = 1.0;
      double yMin = -1.0;
      double yMax = 1.0;

      ReferenceFrame someFrame = ReferenceFrame.constructARootFrame("someFrame", false, false, true);
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
      FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();

      Point2d pointOnLine = new Point2d(0.0, 0.0);
      Vector2d directionVector1 = new Vector2d(1.0, 1.0);
      Vector2d directionVector2 = new Vector2d(1.0, -1.0);
      Line2d line1 = new Line2d(pointOnLine, directionVector1);
      Line2d line2 = new Line2d(pointOnLine, directionVector2);
      plotter.addFrameLine2d(new FrameLine2d(someFrame, line1));
      plotter.addFrameLine2d(new FrameLine2d(someFrame, line2));

      Point2d testPoint = new Point2d();
      for (int i = 0; i < 1000; i++)
      {
         testPoint.set(xMin + (xMax - xMin) * random.nextDouble(), yMin + (yMax - yMin) * random.nextDouble());

         if (line1.isPointInFrontOfLine(testPoint) && line2.isPointInFrontOfLine(testPoint))
         {
            plotter.addFramePoint2d(new FramePoint2d(someFrame, testPoint), Color.green);
         }
         else
         {
            plotter.addFramePoint2d(new FramePoint2d(someFrame, testPoint), Color.red);
         }
      }

      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      
      testFrame.dispose();
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransform()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      RigidBodyTransform transform = new RigidBodyTransform();

      // pure translation:
      Vector3d translation = new Vector3d(random.nextDouble(), random.nextDouble(), 0.0);
      Vector3d eulerAngles = new Vector3d(0.0, 0.0, 0.0);

      transform.setEuler(eulerAngles);
      transform.setTranslation(translation);

      Line2d line = new Line2d(line2dPointPoint);
      Point2d point = new Point2d();
      line.getPoint(point);
      Vector2d vector = new Vector2d();
      line.getNormalizedVector(vector);

      line.applyTransform(transform);
      assertEquals("pure translation failed", point.x + translation.x, line.point.x, EPSILON_FOR_EQUALS);
      assertEquals("pure translation failed", point.y + translation.y, line.point.y, EPSILON_FOR_EQUALS);
      assertEquals("pure translation failed", vector.x, line.normalizedVector.x, EPSILON_FOR_EQUALS);
      assertEquals("pure translation failed", vector.y, line.normalizedVector.y, EPSILON_FOR_EQUALS);

      // TODO: test rotation
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testContainsEpsilon()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
	   
      // Test will fail if you change epsilon too much.

      double epsilon = 1E-12;
      assertTrue("Line created from two points does not contain one of those points", line2dPointPoint.containsEpsilon(point1, epsilon));
      assertTrue("Line created from two points does not contain one of those points", line2dPointPoint.containsEpsilon(point2, epsilon));

      Point2d randomPointOnLine = new Point2d();
      Point2d randomPointNotOnLine = new Point2d();
      Vector2d perp;
      for (int i = 0; i < 1000; i++)
      {
         double scaleFactor = 10.0 * random.nextDouble() - 5.0;
         randomPointOnLine.set(point1.x + scaleFactor * (point2.x - point1.x), point1.y + scaleFactor * (point2.y - point1.y));
         assertTrue("Random point on line not handled correctly", line2dPointPoint.containsEpsilon(randomPointOnLine, epsilon));

         double deviation = random.nextDouble() - 0.5;
         perp = line2dPointPoint.perpendicularVector();
         randomPointNotOnLine.set(randomPointOnLine.x + deviation * perp.x, randomPointOnLine.y + deviation * perp.y);

         if (deviation != 0.0)
         {
            assertFalse("Random point not on line not handled correctly", line2dPointPoint.containsEpsilon(randomPointNotOnLine, epsilon));
         }
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetNormalizedVectorCopy()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      Vector2d vector = new Vector2d();
      line2dPointPoint.getNormalizedVector(vector);
      assertNotSame("Normalized vector copy is not a copy of the normalized vector", line2dPointPoint.normalizedVector,
            vector);
      assertEquals("Normalized vector copy doesn't have the same elements as the original", line2dPointPoint.normalizedVector.x,
            vector.x, EPSILON_FOR_EQUALS);
      assertEquals("Normalized vector copy doesn't have the same elements as the original", line2dPointPoint.normalizedVector.y,
            vector.y, EPSILON_FOR_EQUALS);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetSlope()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      double slope = line2dPointPoint.getSlope();
      double scaleFactor = 10.0 * random.nextDouble() - 5.0;
      Point2d shouldBeOnLine = new Point2d(point1.x + scaleFactor * (point2.x - point1.x), point1.y + scaleFactor * (point2.y - point1.y));
      Point2d shouldBeOnLineToo = new Point2d(point1.x + scaleFactor, point1.y + scaleFactor * slope);
      double epsilon = 1E-12;
      assertTrue("You didn't even get this right", line2dPointPoint.containsEpsilon(shouldBeOnLine, epsilon));
      assertTrue("Point should have been on the line", line2dPointPoint.containsEpsilon(shouldBeOnLineToo, epsilon));
   }

	@DeployableTestMethod(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testInteriorBisector()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      Point2d somePoint = new Point2d(random.nextDouble(), random.nextDouble());
      Vector2d vector1 = new Vector2d(1.0, 0.0);
      Vector2d vector2 = new Vector2d(0.0, 1.0);

      double epsilon = 1.0E-8;

      Line2d firstLine = new Line2d(somePoint, vector1);
      Line2d secondLine = new Line2d(somePoint, vector2);
      Line2d bisector = firstLine.interiorBisector(secondLine);

      assertEquals("Bisector point on line not correct", bisector.point.x, somePoint.x, EPSILON_FOR_EQUALS);
      assertEquals("Bisector point on line not correct", bisector.point.y, somePoint.y, EPSILON_FOR_EQUALS);


      assertTrue("Bisector direction not correct", Math.abs(bisector.normalizedVector.x - Math.sqrt(2.0) / 2.0) < epsilon);
      assertTrue("Bisector direction not correct", Math.abs(bisector.normalizedVector.x - Math.sqrt(2.0) / 2.0) < epsilon);

      Line2d parallelLine = new Line2d(firstLine);
      parallelLine.point.y += 5.0;
      bisector = firstLine.interiorBisector(parallelLine);
      assertNull(bisector);

      for (int i = 0; i < 100000; i++)
      {
         Line2d randomLine1 = new Line2d(new Point2d(50.0 * random.nextDouble(), 50.0 * random.nextDouble()),
                                         new Vector2d(random.nextDouble(), random.nextDouble()));
         Line2d randomLine2 = new Line2d(new Point2d(50.0 * random.nextDouble(), 50.0 * random.nextDouble()),
                                         new Vector2d(random.nextDouble(), random.nextDouble()));
         bisector = randomLine1.interiorBisector(randomLine2);
         Point2d intersection = randomLine1.intersectionWith(randomLine2);

         if (intersection == null)
         {
            assertNull(bisector);
         }
         else
         {
            assertEquals("Bisector point on line not correct", intersection.x, bisector.point.x, EPSILON_FOR_EQUALS);
            assertEquals("Bisector point on line not correct", intersection.y, bisector.point.y, EPSILON_FOR_EQUALS);
            double angleBetweenRandomLines = randomLine1.normalizedVector.angle(randomLine2.normalizedVector);
            double angleBetweenRandomLineAndBisector = randomLine1.normalizedVector.angle(bisector.normalizedVector);
            assertTrue("Angle not correct", Math.abs(angleBetweenRandomLines - 2.0 * angleBetweenRandomLineAndBisector) < epsilon);
         }
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNegateDirection()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      Line2d someLine = new Line2d(line2dPointPoint);

      Vector2d directionVectorBefore = someLine.normalizedVector;
      double directionBeforeX = someLine.normalizedVector.x;
      double directionBeforeY = someLine.normalizedVector.y;

      someLine.negateDirection();

      assertEquals(-directionBeforeX, someLine.normalizedVector.x, EPSILON_FOR_EQUALS);
      assertEquals(-directionBeforeY, someLine.normalizedVector.y, EPSILON_FOR_EQUALS);
      assertEquals(directionVectorBefore, someLine.normalizedVector);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNegateDirectionCopy()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      Line2d someLine = new Line2d(line2dPointPoint);

      Vector2d directionVectorBefore = someLine.normalizedVector;
      Point2d pointBefore = someLine.point;
      double directionBeforeX = someLine.normalizedVector.x;
      double directionBeforeY = someLine.normalizedVector.y;

      Line2d copy = someLine.negateDirectionCopy();

      assertEquals(-directionBeforeX, copy.normalizedVector.x, EPSILON_FOR_EQUALS);
      assertEquals(-directionBeforeY, copy.normalizedVector.y, EPSILON_FOR_EQUALS);
      assertNotSame(directionVectorBefore, copy.normalizedVector);
      assertNotSame(pointBefore, copy.point);
      assertNotSame(someLine, copy);
   }

	@DeployableTestMethod(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testPerpendicularVector()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      Line2d line;
      Vector2d vector;
      Vector2d vectorPerp;
      for (int i = 0; i < 1000000; i++)
      {
         line = new Line2d(new Point2d(random.nextDouble(), random.nextDouble()), new Vector2d(random.nextDouble(), random.nextDouble()));
         vector = new Vector2d();
         line.getNormalizedVector(vector);
         vectorPerp = line.perpendicularVector();
         assertEquals("Dot product of perpendicular vectors not equal to zero", 0.0, vector.dot(vectorPerp), EPSILON_FOR_EQUALS);
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testZeroLength()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      
      Vector2d directionAndLength = new Vector2d(0.0, 0.0);
      Point2d start = new Point2d(1.0, 1.0);
      new Line2d(start, directionAndLength);
   }
}
