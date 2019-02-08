package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SmartCMPPlanarProjectorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testSimpleProjections()
   {
      YoVariableRegistry registry;

      registry = new YoVariableRegistry("Test");

      SmartCMPPlanarProjector cmpProjector = new SmartCMPPlanarProjector(registry);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Test all in same place
      double[][] pointList = new double[][] { { 0.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 1.0 }, { 0.0, 1.0 } };
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(pointList));

      FramePoint2D capturePoint = new FramePoint2D(worldFrame, 0.5, 0.5);
      FramePoint2D desiredCMP = new FramePoint2D(worldFrame, 0.5, 0.5);
      FramePoint2D expectedCMPProjection = new FramePoint2D(worldFrame, 0.5, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge
      desiredCMP = new FramePoint2D(worldFrame, 1.0, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 1.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2D(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.97, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2D(worldFrame, 1.02, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 1.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge
      desiredCMP = new FramePoint2D(worldFrame, 1.9, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 1.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge past y edge
      desiredCMP = new FramePoint2D(worldFrame, 1.0, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 1.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2D(worldFrame, 0.97, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.97, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2D(worldFrame, 1.02, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 1.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge and y edge
      desiredCMP = new FramePoint2D(worldFrame, 1.9, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 1.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge
      desiredCMP = new FramePoint2D(worldFrame, 0.0, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2D(worldFrame, 0.03, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.03, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge
      desiredCMP = new FramePoint2D(worldFrame, -0.02, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge
      desiredCMP = new FramePoint2D(worldFrame, -0.9, 0.5);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.0, 0.5);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection at x edge past y edge
      desiredCMP = new FramePoint2D(worldFrame, 0.0, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2D(worldFrame, 0.03, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.03, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection near x edge past y edge
      desiredCMP = new FramePoint2D(worldFrame, -0.02, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection past x edge and y edge
      desiredCMP = new FramePoint2D(worldFrame, -0.9, 2.0);
      expectedCMPProjection = new FramePoint2D(worldFrame, 0.0, 2.0);
      checkOne(cmpProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);
   }

   private void checkOne(SmartCMPPlanarProjector cmpProjection, FrameConvexPolygon2D supportPolygon, FramePoint2D capturePoint,
         FramePoint2D desiredCMP, FramePoint2D expectedCMPProjection)
   {
      FramePoint2D desiredCMPProjection = new FramePoint2D(desiredCMP);

      cmpProjection.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, null, desiredCMPProjection);
      assertTrue(expectedCMPProjection.epsilonEquals(desiredCMPProjection, 1e-7));
   }
}
