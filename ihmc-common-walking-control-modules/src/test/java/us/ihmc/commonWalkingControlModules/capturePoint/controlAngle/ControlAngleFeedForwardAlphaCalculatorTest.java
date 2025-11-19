package us.ihmc.commonWalkingControlModules.capturePoint.controlAngle;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Gray;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Red;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPolygon2D;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerHelper;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ControlAngleFeedForwardAlphaCalculatorTest
{
   private static final double footLength = 0.2;
   private static final double footWidth = 0.1;

   private static final boolean visualize = false;

   @Test
   public void testStandingInPlace()
   {
      YoRegistry registry = new YoRegistry("test");

      ControlAngleFeedForwardAlphaCalculator calculator = new ControlAngleFeedForwardAlphaCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ((YoBoolean) registry.findVariable("useICPFeedForwardScaling")).set(true);

      FrameConvexPolygon2D leftFoot = createFootPolygon(0.3, 0.15, footLength, footWidth);
      FrameConvexPolygon2D rightFoot = createFootPolygon(0.3, -0.225, footLength, footWidth);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertices(leftFoot);
      supportPolygon.addVertices(rightFoot);
      supportPolygon.update();

      FramePoint2D finalICP = new FramePoint2D();
      FramePoint2D referenceCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.306324, -0.037566);
      FramePoint2D referenceICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.306271, -0.037566);
      FramePoint2D currentICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.305207, -0.056521);
      FramePoint2D feedbackCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.302256, -0.112021);
      finalICP.setToNaN();

      FeedForwardVisualizer visualizer = null;
      if (visualize)
         visualizer = new FeedForwardVisualizer(registry, calculator.getSCS2YoGraphics());

      double computedAlpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }
      assertEquals(0.0, computedAlpha, 1e-5);
   }

   @Test
   public void testLeftFootForward()
   {
      YoRegistry registry = new YoRegistry("test");

      ControlAngleFeedForwardAlphaCalculator calculator = new ControlAngleFeedForwardAlphaCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ((YoBoolean) registry.findVariable("useICPFeedForwardScaling")).set(true);

      FrameConvexPolygon2D leftFoot = createFootPolygon(0.4, 0.15, footLength, footWidth);
      FrameConvexPolygon2D rightFoot = createFootPolygon(0.0, -0.15, footLength, footWidth);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertices(leftFoot);
      supportPolygon.addVertices(rightFoot);
      supportPolygon.update();

      FramePoint2D finalICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.4 + 0.25 * footLength, 0.15);
      FramePoint2D referenceCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, -0.15);
      FramePoint2D referenceICP = new FramePoint2D();

      double omega = 3.0;
      double timeRemaining = 0.25;

      CapturePointTools.computeDesiredCapturePointPosition(omega, -timeRemaining, finalICP, referenceCMP, referenceICP);

      FramePoint2D currentICP = new FramePoint2D(referenceICP);
      currentICP.add(-0.05, 0.05);

      FrameVector2D error = new FrameVector2D();
      error.sub(currentICP, referenceICP);
      error.scale(2.0);

      FramePoint2D feedbackCMP = new FramePoint2D();
      feedbackCMP.add(referenceCMP, error);

      error.negate();

      FeedForwardVisualizer visualizer = null;
      if (visualize)
         visualizer = new FeedForwardVisualizer(registry, calculator.getSCS2YoGraphics());

      double computedAlpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }

      FrameLineSegment2D backEdge = new FrameLineSegment2D();
      FrameLine2D errorLine = new FrameLine2D();
      FrameLine2D feedbackLine = new FrameLine2D();
      errorLine.set(referenceICP, currentICP);
      feedbackLine.set(referenceCMP, feedbackCMP);
      backEdge.getFirstEndpoint().set(0.3, 0.2);
      backEdge.getSecondEndpoint().set(-0.1, 0.1);

      FramePoint2DReadOnly[] intersections = supportPolygon.intersectionWithRay(errorLine);

      FramePoint2D pointWithNoFeedForward = new FramePoint2D();
      FramePoint2D pointWithFullFeedForward = new FramePoint2D();

      if (referenceICP.getX() > intersections[0].getX())
         pointWithNoFeedForward.set(intersections[0]);
      else
         pointWithNoFeedForward.set(intersections[1]);

      intersections = supportPolygon.intersectionWith(feedbackLine);
      if (intersections[0].getX() < intersections[1].getX())
         pointWithFullFeedForward.set(intersections[0]);
      else
         pointWithFullFeedForward.set(intersections[1]);

      EuclidFrameTestTools.assertGeometricallyEquals(pointWithNoFeedForward, calculator.getPointOnEdgeWithNoFeedForward(), 1e-5);
      EuclidFrameTestTools.assertGeometricallyEquals(pointWithFullFeedForward, calculator.getPointOnEdgeWithFullFeedForward(), 1e-5);

      assertTrue(computedAlpha > 0.0);

      FrameVector2D vectorToEnd = new FrameVector2D();
      vectorToEnd.sub(finalICP, currentICP);
      FrameVector2D minConvergenceVector = new FrameVector2D();
      FramePoint2D convergedPoint = new FramePoint2D();
      convergedPoint.add(currentICP, minConvergenceVector);

      assertTrue(vectorToEnd.angle(error) < 0.0);
      RotationMatrixTools.applyYawRotation(-Math.abs(ControlAngleFeedForwardAlphaCalculator.nominalAngleToConverge), vectorToEnd, minConvergenceVector);
      assertTrue(vectorToEnd.angle(minConvergenceVector) < 0.0);
      assertTrue(convergedPoint.getY() < finalICP.getY());

      FrameLine2D convergenceLine = new FrameLine2D();
      convergenceLine.set(currentICP, minConvergenceVector);

      FramePoint2D minPointAlongEdge = new FramePoint2D();
      intersections = supportPolygon.intersectionWith(convergenceLine);

      if (intersections[0].getX() < intersections[1].getX())
         minPointAlongEdge.set(intersections[0]);
      else
         minPointAlongEdge.set(intersections[1]);

      FrameVector2D dynamicsDirection = new FrameVector2D();
      dynamicsDirection.sub(referenceCMP, finalICP);
      FramePoint2D reallyFarAwayCMP = new FramePoint2D();
      reallyFarAwayCMP.scaleAdd(10.0, dynamicsDirection, referenceCMP);
      FrameLineSegment2D referenceControlLine = new FrameLineSegment2D();
      referenceControlLine.set(finalICP, reallyFarAwayCMP);

      assertTrue(Double.isFinite(EuclidGeometryTools.percentageOfIntersectionBetweenLineSegment2DAndLine2D(reallyFarAwayCMP,
                                                                                                                  finalICP,
                                                                                                                  currentICP,
                                                                                                                  minConvergenceVector)));
      assertFalse(convergenceLine.intersectionWith(referenceControlLine) == null);

      EuclidFrameTestTools.assertGeometricallyEquals(minPointAlongEdge, calculator.getPointOnEdgeThatConverges(), 5e-4);

      assertEquals(EuclidGeometryTools.percentageAlongLineSegment2D(minPointAlongEdge, pointWithFullFeedForward, pointWithNoFeedForward),
                          computedAlpha,
                          5e-2);
   }

   @Test
   @Disabled
   public void testLeftFootForwardDynamicsInLine()
   {
      YoRegistry registry = new YoRegistry("test");

      ControlAngleFeedForwardAlphaCalculator calculator = new ControlAngleFeedForwardAlphaCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ((YoBoolean) registry.findVariable("useICPFeedForwardScaling")).set(true);

      FrameConvexPolygon2D leftFoot = createFootPolygon(0.4, 0.15, footLength, footWidth);
      FrameConvexPolygon2D rightFoot = createFootPolygon(0.0, -0.15, footLength, footWidth);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertices(leftFoot);
      supportPolygon.addVertices(rightFoot);
      supportPolygon.update();

      FramePoint2D finalICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.4 + 0.25 * footLength, 0.15);
      FramePoint2D referenceCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, -0.15);
      FramePoint2D referenceICP = new FramePoint2D();

      double omega = 3.0;
      double timeRemaining = 0.25;

      CapturePointTools.computeDesiredCapturePointPosition(omega, -timeRemaining, finalICP, referenceCMP, referenceICP);

      FrameVector2D error = new FrameVector2D();
      error.sub(finalICP, referenceICP);
      error.normalize();
      error.scale(0.05);

      FramePoint2D currentICP = new FramePoint2D(referenceICP);

      currentICP.add(referenceICP, error);

      FramePoint2D feedbackCMP = new FramePoint2D();
      feedbackCMP.scaleAdd(2.0, error, referenceCMP);

      FeedForwardVisualizer visualizer = null;
      if (visualize)
         visualizer = new FeedForwardVisualizer(registry, calculator.getSCS2YoGraphics());

      double alpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }

      assertEquals(0.0, alpha, 1e-5);

      error.negate();
      currentICP.add(referenceICP, error);
      feedbackCMP.scaleAdd(2.0, error, referenceCMP);

      assertEquals(0.0, calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon), 1e-5);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }
   }

   @Test
   public void testLeadingDynamicsWithALittleOrthogonal()
   {
      YoRegistry registry = new YoRegistry("test");

      ControlAngleFeedForwardAlphaCalculator calculator = new ControlAngleFeedForwardAlphaCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ((YoBoolean) registry.findVariable("useICPFeedForwardScaling")).set(true);

      FrameConvexPolygon2D leftFoot = createFootPolygon(0.4, 0.15, footLength, footWidth);
      FrameConvexPolygon2D rightFoot = createFootPolygon(0.0, -0.15, footLength, footWidth);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertices(leftFoot);
      supportPolygon.addVertices(rightFoot);
      supportPolygon.update();

      FramePoint2D finalICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.4 + 0.25 * footLength, 0.15);
      FramePoint2D referenceCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, -0.15);
      FramePoint2D referenceICP = new FramePoint2D();

      double omega = 3.0;
      double timeRemaining = 0.25;

      CapturePointTools.computeDesiredCapturePointPosition(omega, -timeRemaining, finalICP, referenceCMP, referenceICP);

      FrameVector2D parallelError = new FrameVector2D();
      FrameVector2D perpendicularError = new FrameVector2D();
      parallelError.sub(finalICP, referenceICP);
      parallelError.normalize();
      parallelError.scale(0.05);
      RotationMatrixTools.applyYawRotation(Math.PI / 2.0, parallelError, perpendicularError);
      perpendicularError.normalize();
      perpendicularError.scale(0.02);

      FramePoint2D currentICP = new FramePoint2D(referenceICP);

      FrameVector2D error = new FrameVector2D();
      error.add(parallelError, perpendicularError);

      currentICP.add(referenceICP, error);

      FramePoint2D feedbackCMP = new FramePoint2D();
      feedbackCMP.scaleAdd(3.0, error, referenceCMP);

      FeedForwardVisualizer visualizer = null;
      if (visualize)
         visualizer = new FeedForwardVisualizer(registry, calculator.getSCS2YoGraphics());

      double alpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }

      assertEquals(0.0, alpha, 1e-5);

      perpendicularError.negate();
      error.add(parallelError, perpendicularError);
      currentICP.add(referenceICP, error);
      feedbackCMP.scaleAdd(2.0, referenceCMP);

      alpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }

      assertEquals(0.0, alpha, 1e-5);

   }

   @Test
   @Disabled
   public void testWithReferenceCMPFarOutside()
   {
      YoRegistry registry = new YoRegistry("test");

      ControlAngleFeedForwardAlphaCalculator calculator = new ControlAngleFeedForwardAlphaCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ((YoBoolean) registry.findVariable("useICPFeedForwardScaling")).set(true);

      FrameConvexPolygon2D leftFoot = createFootPolygon(0.4, 0.15, footLength, footWidth);
      FrameConvexPolygon2D rightFoot = createFootPolygon(0.0, -0.15, footLength, footWidth);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertices(leftFoot);
      supportPolygon.addVertices(rightFoot);
      supportPolygon.update();

      FramePoint2D finalICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.4 + 0.25 * footLength, 0.15);
      FramePoint2D referenceCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, -0.25);
      FramePoint2D referenceICP = new FramePoint2D();

      double omega = 3.0;
      double timeRemaining = 0.15;

      CapturePointTools.computeDesiredCapturePointPosition(omega, -timeRemaining, finalICP, referenceCMP, referenceICP);

      FrameVector2D parallelError = new FrameVector2D();
      FrameVector2D perpendicularError = new FrameVector2D();
      parallelError.sub(finalICP, referenceICP);
      parallelError.normalize();
      parallelError.scale(-0.07);
      RotationMatrixTools.applyYawRotation(Math.PI / 2.0, parallelError, perpendicularError);
      perpendicularError.normalize();
      perpendicularError.scale(0.03);

      FramePoint2D currentICP = new FramePoint2D(referenceICP);

      FrameVector2D error = new FrameVector2D();
      error.add(parallelError, perpendicularError);

      currentICP.add(referenceICP, error);

      FramePoint2D feedbackCMP = new FramePoint2D();
      feedbackCMP.scaleAdd(5.0, error, referenceCMP);

      FeedForwardVisualizer visualizer = null;
      if (visualize)
         visualizer = new FeedForwardVisualizer(registry, calculator.getSCS2YoGraphics());

      double alpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
      double expectedAlpha = EuclidGeometryTools.percentageAlongLineSegment2D(calculator.getPointOnEdgeThatConverges(),
                                                                              calculator.getPointOnEdgeWithFullFeedForward(),
                                                                              calculator.getPointOnEdgeWithNoFeedForward());

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }
      Assert.assertEquals(expectedAlpha, alpha, 1e-2);

      perpendicularError.negate();
      error.add(parallelError, perpendicularError);
      currentICP.add(referenceICP, error);
      feedbackCMP.add(referenceCMP, error);

      Assert.assertEquals(0.0, calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon), 1e-5);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }
   }

   /**
    * Whenever the CMP, ICP, and final ICP are collinear, we don't want the scale to jump. If the CMP passes from being slightly to the left or right of being
    * colinear, we want the scale factor for feedback to be consistent the whole time as it crosses this point. That was a bug noticed on the robot
    *
    */
   @Test
   @Disabled
   public void testTrickyCollinearCase()
   {
      YoRegistry registry = new YoRegistry("test");

      ControlAngleFeedForwardAlphaCalculator calculator = new ControlAngleFeedForwardAlphaCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ((YoBoolean) registry.findVariable("useICPFeedForwardScaling")).set(true);

      FrameConvexPolygon2D leftFoot = new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame());
      leftFoot.addVertex(-3.595471763095993, -7.390665836075279);
      leftFoot.addVertex(-3.505093911908039, -7.198195173564839);
      leftFoot.addVertex(-3.423225825315417, -7.2461599986053935);
      leftFoot.addVertex(-3.543765603142758, -7.420959409785102);
      leftFoot.update();

      FrameConvexPolygon2D rightFoot = new FrameConvexPolygon2D();
      rightFoot.addVertex(-3.5858439860301408, -6.803592147077807);
      rightFoot.addVertex(-3.4854654237375433, -6.612822420209966);
      rightFoot.addVertex(-3.405241406353212, -6.663657655439729);
      rightFoot.addVertex(-3.5351761855768786, -6.835698611433447);
      rightFoot.update();

      createFootPolygon(0.0, -0.15, footLength, footWidth);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertices(leftFoot);
      supportPolygon.addVertices(rightFoot);
      supportPolygon.update();

      FramePoint2D finalICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -3.5337331519404134, -7.297843994153399);
      FramePoint2D referenceCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -3.544940195335232, -6.859968658118232);
      FramePoint2D referenceICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -3.5376237983783314, -7.081717256311211);

      FramePoint2D currentICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -3.5366390956653078, -7.037701051009898);

      FrameVector2D error = new FrameVector2D();
      error.sub(currentICP, referenceICP);

      double kpParallel = 3.0;
      double kpOrthogonal = 3.0;
      double omega = 3.2;

      ICPControllerHelper helper = new ICPControllerHelper();
      DMatrixRMaj transformedGains = new DMatrixRMaj(2, 2);
      FrameVector2D desiredICPVelocity = new FrameVector2D();
      CapturePointTools.computeCapturePointVelocity(currentICP, referenceCMP, omega, desiredICPVelocity);
      helper.transformGainsFromDynamicsFrame(transformedGains,
                                             desiredICPVelocity,
                                             kpParallel,
                                             kpOrthogonal);

      FramePoint2D feedbackCMP = new FramePoint2D();
      feedbackCMP.setX(transformedGains.get(0, 0) * error.getX() + transformedGains.get(0, 1) * error.getY());
      feedbackCMP.setY(transformedGains.get(1, 0) * error.getX() + transformedGains.get(1, 1) * error.getY());
      feedbackCMP.add(referenceCMP);

      FeedForwardVisualizer visualizer = null;
      if (visualize)
         visualizer = new FeedForwardVisualizer(registry, calculator.getSCS2YoGraphics());

      double alpha = calculator.computeAlpha(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);

      if (visualize)
      {
         visualizer.visualize(currentICP, referenceICP, finalICP, referenceCMP, feedbackCMP, supportPolygon);
         visualizer.waitUntilVisualizerDown();
      }

      assertEquals(0.0, alpha, 1e-5);
   }

   private static FrameConvexPolygon2D createFootPolygon(double xPosition, double yPosition, double footLength, double footWidth)
   {
      FrameConvexPolygon2D polygon = new FrameConvexPolygon2D();
      polygon.addVertex(0.5 * footLength, 0.5 * footWidth);
      polygon.addVertex(0.5 * footLength, -0.5 * footWidth);
      polygon.addVertex(-0.5 * footLength, -0.5 * footWidth);
      polygon.addVertex(-0.5 * footLength, 0.5 * footWidth);
      polygon.update();

      polygon.translate(xPosition, yPosition);

      return polygon;
   }

   private static class FeedForwardVisualizer
   {
      private SimulationConstructionSet2 scs;

      private final YoFramePoint2D currentICP;
      private final YoFramePoint2D referenceICP;
      private final YoFramePoint2D referenceCMP;
      private final YoFramePoint2D feedbackCMP;
      private final YoFramePoint2D finalICP;
      private final YoFrameConvexPolygon2D supportPolygon;

      public FeedForwardVisualizer(YoRegistry registry, YoGraphicDefinition graphicDefinition)
      {
         currentICP = new YoFramePoint2D("currentICP", ReferenceFrame.getWorldFrame(), registry);
         referenceICP = new YoFramePoint2D("referenceICP", ReferenceFrame.getWorldFrame(), registry);
         referenceCMP = new YoFramePoint2D("referenceCMP", ReferenceFrame.getWorldFrame(), registry);
         feedbackCMP = new YoFramePoint2D("feedbackCMP", ReferenceFrame.getWorldFrame(), registry);
         finalICP = new YoFramePoint2D("finalICP", ReferenceFrame.getWorldFrame(), registry);
         supportPolygon = new YoFrameConvexPolygon2D("SupportPolygon", ReferenceFrame.getWorldFrame(), 8, registry);

         YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
         group.addChild(newYoGraphicPoint2D("Desired Capture Point", referenceICP, 0.02, ColorDefinitions.Yellow(), DefaultPoint2DGraphic.CIRCLE_CROSS));
         group.addChild(newYoGraphicPoint2D("Current Capture Point", currentICP, 0.02, ColorDefinitions.Blue(), DefaultPoint2DGraphic.CIRCLE_CROSS));
         group.addChild(newYoGraphicPoint2D("Final Desired Capture Point", finalICP, 0.02, ColorDefinitions.Red(), DefaultPoint2DGraphic.CIRCLE_CROSS));
         group.addChild(newYoGraphicPoint2D("Perfect CMP", referenceCMP, 0.02, ColorDefinitions.Gray(), DefaultPoint2DGraphic.CIRCLE_CROSS));
         group.addChild(newYoGraphicPoint2D("Feedback CMP", this.feedbackCMP, 0.02, ColorDefinitions.Purple(), DefaultPoint2DGraphic.CIRCLE_CROSS));
         group.addChild(newYoGraphicPolygon2D("Support Polygon",supportPolygon, ColorDefinitions.Tan()));

         scs = new SimulationConstructionSet2();
         scs.setDT(1.0);
         scs.initializeBufferSize(500);
         scs.setShutdownSessionOnVisualizerClose(false);
         scs.getRootRegistry().addChild(registry);
         scs.addYoGraphic(graphicDefinition);
         scs.start(true, true, true);
      }

      public void visualize(FramePoint2DReadOnly currentICP,
                            FramePoint2DReadOnly referenceICP,
                            FramePoint2DReadOnly finalICP,
                            FramePoint2DReadOnly referenceCMP,
                            FramePoint2DReadOnly feedbackCMP,
                            FrameConvexPolygon2DReadOnly supportPolygon)
      {
         this.currentICP.setMatchingFrame(currentICP);
         this.referenceCMP.setMatchingFrame(referenceCMP);
         this.referenceICP.setMatchingFrame(referenceICP);
         this.finalICP.setMatchingFrame(finalICP);
         this.feedbackCMP.setMatchingFrame(feedbackCMP);
         this.supportPolygon.setMatchingFrame(supportPolygon, false);

         scs.simulateNow(1);
      }

      public void waitUntilVisualizerDown()
      {
         scs.startSimulationThread();
         scs.showOverheadPlotter2D(true);
         scs.waitUntilVisualizerDown();
      }
   }
}
