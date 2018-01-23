package us.ihmc.robotics.geometry;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class FrameLine2dTest
{
   private static final boolean VERBOSE = false;

   private final boolean DISPLAY_PANEL = false;
   private final boolean WAIT_FOR_BUTTON_PUSH = false;

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionOne()
   {
      double xMin = -1.0, xMax = 2.0, yMin = -1.0, yMax = 2.0;

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("zUpFrame");

      FramePoint2D point1 = new FramePoint2D(zUpFrame, 0.0, 0.0);
      FrameVector2D vector1 = new FrameVector2D(zUpFrame, 1.0, 1.0);

      FrameLine2D line1 = new FrameLine2D(point1, vector1);

      FramePoint2D point2 = new FramePoint2D(zUpFrame, 1.0, 0.0);
      FrameVector2D vector2 = new FrameVector2D(zUpFrame, -1.0, 1.0);

      FrameLine2D line2 = new FrameLine2D(point2, vector2);


      FramePoint2DBasics intersection = line1.intersectionWith(line2);

//    assertEpsilonEquals(intersection.getX(), 0.5);
//    assertEpsilonEquals(intersection.getY(), 0.5);

      if (VERBOSE) System.out.println("intersection = " + intersection.getX() + ", " + intersection.getY());


      if (DISPLAY_PANEL)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);

         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();

         plotter.addFrameLine2d(line1);
         plotter.addFrameLine2d(line2);

         plotter.addFramePoint2d(intersection);

         if (WAIT_FOR_BUTTON_PUSH)
            testFrame.waitForButtonPush();
      }

   }

   @SuppressWarnings("unused")
   private void assertEpsilonEquals(double v1, double v2)
   {
      if (Math.abs(v1 - v2) > 1e-7)
         throw new RuntimeException("v1 = " + v1 + ", v2 = " + v2);
   }

}
