package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class HumanEvaluationLine2dTest
{
   private final boolean WAIT_FOR_BUTTON_PUSH = false;
   private final double EPSILON_FOR_EQUALS = 1e-8;

   private Random random = new Random(100L);
   private Point2D point1 = new Point2D(random.nextDouble(), random.nextDouble());
   private Point2D point2 = new Point2D(random.nextDouble(), random.nextDouble());
   private Vector2D vector = new Vector2D(random.nextDouble() + 1.0, random.nextDouble());    // to ensure that the vector is not vertical (for the slope test)
   private Line2D line2dPointVector = new Line2D(point1, vector);
   private Line2D line2dPointPoint = new Line2D(point1, point2);
   private Line2D line2dLine2d = new Line2D(line2dPointPoint);

   @Before
   public void setUp() throws Exception
   {
      double epsilon = 1E-6;
      assertTrue("Normalized vector not normalized", Math.abs(line2dPointVector.getDirection().length() - 1.0) < epsilon);
      assertTrue("Normalized vector not normalized", Math.abs(line2dPointPoint.getDirection().length() - 1.0) < epsilon);
      assertTrue("Normalized vector not normalized", Math.abs(line2dLine2d.getDirection().length() - 1.0) < epsilon);

      assertEquals("Creating line from other line failed", line2dPointPoint.getPointX(), line2dLine2d.getPointX(), EPSILON_FOR_EQUALS);
      assertEquals("Creating line from other line failed", line2dPointPoint.getPointY(), line2dLine2d.getPointY(), EPSILON_FOR_EQUALS);
      assertEquals("Creating line from other line failed", line2dPointPoint.getDirectionX(), line2dLine2d.getDirectionX(), EPSILON_FOR_EQUALS);
      assertEquals("Creating line from other line failed", line2dPointPoint.getDirectionY(), line2dLine2d.getDirectionY(), EPSILON_FOR_EQUALS);

      assertNotSame("Line fields not copied", point1, line2dPointVector.getPoint());
      assertNotSame("Line fields not copied", vector, line2dPointVector.getDirection());    // this would be really weird, but test anyway
      assertNotSame("Line fields not copied", point1, line2dPointPoint.getPoint());
      assertNotSame("Line fields not copied", line2dPointPoint.getDirection(), line2dLine2d.getDirection());
      assertNotSame("Line fields not copied", line2dPointPoint.getPoint(), line2dLine2d.getPoint());

      boolean lineCreatedFromCoincidalPoints = true;
      try
      {
         // Try creating a line from two coincidal points.
         new Line2D(point1, new Point2D(point1));
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

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout = 30000)
   public void testIsOnLeftSideOfLine()
   {
      double xMin = -1.0;
      double xMax = 1.0;
      double yMin = -1.0;
      double yMax = 1.0;

      ReferenceFrame someFrame = ReferenceFrame.constructARootFrame("someFrame");
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
      FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();

      Point2D randomPoint = new Point2D(xMin + (xMax - xMin) * random.nextDouble(), yMin + (yMax - yMin) * random.nextDouble());
      Vector2D randomVector = new Vector2D(random.nextDouble(), random.nextDouble());    // both are positive, so vector is always in upper right hand quadrant
      Line2D randomLine = new Line2D(randomPoint, randomVector);
      plotter.addFrameLine2d(new FrameLine2d(someFrame, randomLine));
      plotter.addFrameLine2d(new FrameLine2d(someFrame, randomLine.negateDirectionCopy()));

      Point2D testPoint = new Point2D();
      for (int i = 0; i < 1000; i++)
      {
         testPoint.set(xMin + (xMax - xMin) * random.nextDouble(), yMin + (yMax - yMin) * random.nextDouble());

         if (randomLine.isPointOnLeftSideOfLine(testPoint))
         {
            plotter.addFramePoint2d(new FramePoint2D(someFrame, testPoint), Color.blue);
         }

         if (randomLine.isPointOnRightSideOfLine(testPoint))
         {
            plotter.addFramePoint2d(new FramePoint2D(someFrame, testPoint), Color.orange);
         }

         assertFalse("Point neither on the right nor on the left side of the line",
                     !randomLine.isPointOnLeftSideOfLine(testPoint) &&!randomLine.isPointOnRightSideOfLine(testPoint));
      }

      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();

      testFrame.dispose();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout = 30000)
   public void testIsInFrontOfLine()
   {
      // illegal method call:
      Line2D line = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));

      boolean exceptionThrown = false;
      try
      {
         line.isPointInFrontOfLine(new Point2D(random.nextDouble(), random.nextDouble()));
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

      ReferenceFrame someFrame = ReferenceFrame.constructARootFrame("someFrame");
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
      FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();

      Point2D pointOnLine = new Point2D(0.0, 0.0);
      Vector2D directionVector1 = new Vector2D(1.0, 1.0);
      Vector2D directionVector2 = new Vector2D(1.0, -1.0);
      Line2D line1 = new Line2D(pointOnLine, directionVector1);
      Line2D line2 = new Line2D(pointOnLine, directionVector2);
      plotter.addFrameLine2d(new FrameLine2d(someFrame, line1));
      plotter.addFrameLine2d(new FrameLine2d(someFrame, line2));

      Point2D testPoint = new Point2D();
      for (int i = 0; i < 1000; i++)
      {
         testPoint.set(xMin + (xMax - xMin) * random.nextDouble(), yMin + (yMax - yMin) * random.nextDouble());

         if (line1.isPointInFrontOfLine(testPoint) && line2.isPointInFrontOfLine(testPoint))
         {
            plotter.addFramePoint2d(new FramePoint2D(someFrame, testPoint), Color.green);
         }
         else
         {
            plotter.addFramePoint2d(new FramePoint2D(someFrame, testPoint), Color.red);
         }
      }

      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      
      testFrame.dispose();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransform()
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      // pure translation:
      Vector3D translation = new Vector3D(random.nextDouble(), random.nextDouble(), 0.0);
      Vector3D eulerAngles = new Vector3D(0.0, 0.0, 0.0);

      transform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.setTranslation(translation);

      Line2D line = new Line2D(line2dPointPoint);
      Point2D point = new Point2D();
      line.getPoint(point);
      Vector2D vector = new Vector2D();
      line.getDirection(vector);

      line.applyTransform(transform);
      assertEquals("pure translation failed", point.getX() + translation.getX(), line.getPointX(), EPSILON_FOR_EQUALS);
      assertEquals("pure translation failed", point.getY() + translation.getY(), line.getPointY(), EPSILON_FOR_EQUALS);
      assertEquals("pure translation failed", vector.getX(), line.getDirectionX(), EPSILON_FOR_EQUALS);
      assertEquals("pure translation failed", vector.getY(), line.getDirectionY(), EPSILON_FOR_EQUALS);

      // TODO: test rotation
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testContainsEpsilon()
   {
      // Test will fail if you change epsilon too much.

      double epsilon = 1E-12;
      assertTrue("Line created from two points does not contain one of those points", line2dPointPoint.isPointOnLine(point1, epsilon));
      assertTrue("Line created from two points does not contain one of those points", line2dPointPoint.isPointOnLine(point2, epsilon));

      Point2D randomPointOnLine = new Point2D();
      Point2D randomPointNotOnLine = new Point2D();
      Vector2D perp;
      for (int i = 0; i < 1000; i++)
      {
         double scaleFactor = 10.0 * random.nextDouble() - 5.0;
         randomPointOnLine.set(point1.getX() + scaleFactor * (point2.getX() - point1.getX()), point1.getY() + scaleFactor * (point2.getY() - point1.getY()));
         assertTrue("Random point on line not handled correctly", line2dPointPoint.isPointOnLine(randomPointOnLine, epsilon));

         double deviation = random.nextDouble() - 0.5;
         perp = line2dPointPoint.perpendicularVector();
         randomPointNotOnLine.set(randomPointOnLine.getX() + deviation * perp.getX(), randomPointOnLine.getY() + deviation * perp.getY());

         if (deviation != 0.0)
         {
            assertFalse("Random point not on line not handled correctly", line2dPointPoint.isPointOnLine(randomPointNotOnLine, epsilon));
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetNormalizedVectorCopy()
   {
      Vector2D vector = new Vector2D();
      line2dPointPoint.getDirection(vector);
      assertNotSame("Normalized vector copy is not a copy of the normalized vector", line2dPointPoint.getDirection(),
            vector);
      assertEquals("Normalized vector copy doesn't have the same elements as the original", line2dPointPoint.getDirectionX(),
            vector.getX(), EPSILON_FOR_EQUALS);
      assertEquals("Normalized vector copy doesn't have the same elements as the original", line2dPointPoint.getDirectionY(),
            vector.getY(), EPSILON_FOR_EQUALS);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetSlope()
   {
      double slope = line2dPointPoint.slope();
      double scaleFactor = 10.0 * random.nextDouble() - 5.0;
      Point2D shouldBeOnLine = new Point2D(point1.getX() + scaleFactor * (point2.getX() - point1.getX()), point1.getY() + scaleFactor * (point2.getY() - point1.getY()));
      Point2D shouldBeOnLineToo = new Point2D(point1.getX() + scaleFactor, point1.getY() + scaleFactor * slope);
      double epsilon = 1E-12;
      assertTrue("You didn't even get this right", line2dPointPoint.isPointOnLine(shouldBeOnLine, epsilon));
      assertTrue("Point should have been on the line", line2dPointPoint.isPointOnLine(shouldBeOnLineToo, epsilon));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testInteriorBisector()
   {
      Point2D somePoint = new Point2D(random.nextDouble(), random.nextDouble());
      Vector2D vector1 = new Vector2D(1.0, 0.0);
      Vector2D vector2 = new Vector2D(0.0, 1.0);

      double epsilon = 1.0E-8;

      Line2D firstLine = new Line2D(somePoint, vector1);
      Line2D secondLine = new Line2D(somePoint, vector2);
      Line2D bisector = firstLine.interiorBisector(secondLine);

      assertEquals("Bisector point on line not correct", bisector.getPointX(), somePoint.getX(), EPSILON_FOR_EQUALS);
      assertEquals("Bisector point on line not correct", bisector.getPointY(), somePoint.getY(), EPSILON_FOR_EQUALS);


      assertTrue("Bisector direction not correct", Math.abs(bisector.getDirectionX() - Math.sqrt(2.0) / 2.0) < epsilon);
      assertTrue("Bisector direction not correct", Math.abs(bisector.getDirectionX() - Math.sqrt(2.0) / 2.0) < epsilon);

      Line2D parallelLine = new Line2D(firstLine);
      parallelLine.translate(0.0, 5.0);
      bisector = firstLine.interiorBisector(parallelLine);
      assertNull(bisector);

      for (int i = 0; i < 100000; i++)
      {
         Line2D randomLine1 = new Line2D(new Point2D(50.0 * random.nextDouble(), 50.0 * random.nextDouble()),
                                         new Vector2D(random.nextDouble(), random.nextDouble()));
         Line2D randomLine2 = new Line2D(new Point2D(50.0 * random.nextDouble(), 50.0 * random.nextDouble()),
                                         new Vector2D(random.nextDouble(), random.nextDouble()));
         bisector = randomLine1.interiorBisector(randomLine2);
         Point2D intersection = randomLine1.intersectionWith(randomLine2);

         if (intersection == null)
         {
            assertNull(bisector);
         }
         else
         {
            assertEquals("Bisector point on line not correct", intersection.getX(), bisector.getPointX(), EPSILON_FOR_EQUALS);
            assertEquals("Bisector point on line not correct", intersection.getY(), bisector.getPointY(), EPSILON_FOR_EQUALS);
            double angleBetweenRandomLines = randomLine1.getDirection().angle(randomLine2.getDirection());
            double angleBetweenRandomLineAndBisector = randomLine1.getDirection().angle(bisector.getDirection());
            assertTrue("Angle not correct", Math.abs(angleBetweenRandomLines - 2.0 * angleBetweenRandomLineAndBisector) < epsilon);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNegateDirection()
   {
      Line2D someLine = new Line2D(line2dPointPoint);

      Vector2DReadOnly directionVectorBefore = someLine.getDirection();
      double directionBeforeX = someLine.getDirectionX();
      double directionBeforeY = someLine.getDirectionY();

      someLine.negateDirection();

      assertEquals(-directionBeforeX, someLine.getDirectionX(), EPSILON_FOR_EQUALS);
      assertEquals(-directionBeforeY, someLine.getDirectionY(), EPSILON_FOR_EQUALS);
      assertEquals(directionVectorBefore, someLine.getDirection());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNegateDirectionCopy()
   {
      Line2D someLine = new Line2D(line2dPointPoint);

      Vector2DReadOnly directionVectorBefore = someLine.getDirection();
      Point2DReadOnly pointBefore = someLine.getPoint();
      double directionBeforeX = someLine.getDirectionX();
      double directionBeforeY = someLine.getDirectionY();

      Line2D copy = someLine.negateDirectionCopy();

      assertEquals(-directionBeforeX, copy.getDirectionX(), EPSILON_FOR_EQUALS);
      assertEquals(-directionBeforeY, copy.getDirectionY(), EPSILON_FOR_EQUALS);
      assertNotSame(directionVectorBefore, copy.getDirection());
      assertNotSame(pointBefore, copy.getPoint());
      assertNotSame(someLine, copy);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testPerpendicularVector()
   {
      Line2D line;
      Vector2D vector;
      Vector2D vectorPerp;
      for (int i = 0; i < 1000000; i++)
      {
         line = new Line2D(new Point2D(random.nextDouble(), random.nextDouble()), new Vector2D(random.nextDouble(), random.nextDouble()));
         vector = new Vector2D();
         line.getDirection(vector);
         vectorPerp = line.perpendicularVector();
         assertEquals("Dot product of perpendicular vectors not equal to zero", 0.0, vector.dot(vectorPerp), EPSILON_FOR_EQUALS);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testZeroLength()
   {
      Vector2D directionAndLength = new Vector2D(0.0, 0.0);
      Point2D start = new Point2D(1.0, 1.0);
      new Line2D(start, directionAndLength);
   }
}
