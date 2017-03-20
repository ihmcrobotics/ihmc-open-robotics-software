package us.ihmc.robotics.geometry;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameConvexPolygon2dTest
{

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private ConvexPolygon2d convexPolygon2d;
   private FrameConvexPolygon2d frameConvexPolygon2d;

   @Before
   public void setUp()
   {
      convexPolygon2d = createSomeValidPolygon();
      frameConvexPolygon2d = new FrameConvexPolygon2d(worldFrame, convexPolygon2d);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void thereIsNoTestHere()
   {
      
   }
   
   private ConvexPolygon2d createSomeValidPolygon()
   {
      double[][] polygonPoints = new double[][]
            {
               {-0.05107802536335158, 0.04155594197133163}, {-0.05052044462374434, 0.1431544119584275}, {0.12219695435431863, 0.14220652470109518},
               {0.12219695435431865, -0.041946248489056696}, {0.12163937361471142, -0.1435447184761526}, {-0.05107802536335154, -0.14259683121882027}
            };

      ConvexPolygon2d polygon = new ConvexPolygon2d(polygonPoints);
      return polygon;
   }

}
