package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SmartCMPPlanarProjectorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleProjections()
   {
      YoVariableRegistry registry;

      registry = new YoVariableRegistry("Test");

      SmartCMPPlanarProjector cmpProjector = new SmartCMPPlanarProjector(registry);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Test all in same place
      double[][] pointList = new double[][] { { 0.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 1.0 }, { 0.0, 1.0 } };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.5, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge
      desiredCMP = new FramePoint2d(worldFrame, 1.0, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.97, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2d(worldFrame, 1.02, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge
      desiredCMP = new FramePoint2d(worldFrame, 1.9, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge past y edge
      desiredCMP = new FramePoint2d(worldFrame, 1.0, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.97, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2d(worldFrame, 1.02, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge and y edge
      desiredCMP = new FramePoint2d(worldFrame, 1.9, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge
      desiredCMP = new FramePoint2d(worldFrame, 0.0, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2d(worldFrame, 0.03, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.03, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2d(worldFrame, -0.02, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge
      desiredCMP = new FramePoint2d(worldFrame, -0.9, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge past y edge
      desiredCMP = new FramePoint2d(worldFrame, 0.0, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2d(worldFrame, 0.03, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.03, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2d(worldFrame, -0.02, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge and y edge
      desiredCMP = new FramePoint2d(worldFrame, -0.9, 2.0);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }

   private void checkOne(SmartCMPPlanarProjector cmpProjection, FrameConvexPolygon2d supportPolygon, FramePoint2d capturePoint,
         FramePoint2d desiredCMP, FramePoint2d expectedCMPProjection)
   {
      FramePoint2d desiredCMPProjection = new FramePoint2d(desiredCMP);

      cmpProjection.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, null, desiredCMPProjection);
      assertTrue(expectedCMPProjection.epsilonEquals(desiredCMPProjection, 1e-7));
   }
}
