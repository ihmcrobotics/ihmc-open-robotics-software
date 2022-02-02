package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.robotics.Assert.assertEquals;

public class DistanceInsideHeuristicsTest
{
   @Test
   public void testSimpleStandingInside()
   {
      FrameConvexPolygon2DReadOnly polygon = generateBasicSupportPolygon(0.2, 0.4);
      FramePoint2D icp = new FramePoint2D();

      double maxDistanceToCMP = 0.05;
      double minDistanceInside = 0.005;

      YoRegistry testRegistry = new YoRegistry("testRegistry");
      DistanceInsideHeuristics heuristics = new DistanceInsideHeuristics(polygon, () -> maxDistanceToCMP, () -> minDistanceInside, testRegistry);

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(testRegistry);

      DistanceInsideHeuristicsVisualizer visualizer = new DistanceInsideHeuristicsVisualizer(testRegistry, null);

      visualizer.updateInputs(polygon, icp);

      heuristics.updateDistanceInside(icp);

      assertEquals(maxDistanceToCMP, heuristics.getCmpDistanceFromSupport(), 1e-5);
      assertEquals(minDistanceInside, heuristics.getCoPDistanceInsideSupport(), 1e-5);

      visualizer.updateOutputs(heuristics);

      ThreadTools.sleepForever();
   }

   @Test
   public void testStandingToTheRight()
   {
      double length = 0.2;
      double width = 0.4;
      FrameConvexPolygon2DReadOnly polygon = generateBasicSupportPolygon(length, width);
      FramePoint2D icp = new FramePoint2D();

      double maxDistanceToCMP = 0.05;
      double minDistanceInside = 0.005;

      YoRegistry testRegistry = new YoRegistry("testRegistry");
      DistanceInsideHeuristics heuristics = new DistanceInsideHeuristics(polygon, () -> maxDistanceToCMP, () -> minDistanceInside, testRegistry);

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(testRegistry);

      DistanceInsideHeuristicsVisualizer visualizer = new DistanceInsideHeuristicsVisualizer(testRegistry, null);

      double distanceOutside = 0.1;
      double distanceToUseless = ((YoDouble) testRegistry.findVariable("distanceWhenCoPControlIsUseless")).getDoubleValue();
      icp.setY(0.5 * width + distanceOutside);

      visualizer.updateInputs(polygon, icp);

      heuristics.updateDistanceInside(icp);

      double fractionToUseless = Math.min(distanceOutside / distanceToUseless, 1.0);
      double maxShrinkDistance = 0.5 * width;

      assertEquals(InterpolationTools.linearInterpolate(maxDistanceToCMP, -maxShrinkDistance, fractionToUseless), heuristics.getCmpDistanceFromSupport(), 1e-5);
      assertEquals(InterpolationTools.linearInterpolate(minDistanceInside, maxShrinkDistance, fractionToUseless), heuristics.getCoPDistanceInsideSupport(), 1e-5);

      visualizer.updateOutputs(heuristics);

      ThreadTools.sleepForever();
   }

   private FrameConvexPolygon2DReadOnly generateBasicSupportPolygon(double length, double width)
   {
      FrameConvexPolygon2D polygon = new FrameConvexPolygon2D();
      polygon.addVertex(0.5 * length, 0.5 * width);
      polygon.addVertex(0.5 * length, -0.5 * width);
      polygon.addVertex(-0.5 * length, -0.5 * width);
      polygon.addVertex(-0.5 * length, 0.5 * width);
      polygon.update();

      return polygon;
   }
}
