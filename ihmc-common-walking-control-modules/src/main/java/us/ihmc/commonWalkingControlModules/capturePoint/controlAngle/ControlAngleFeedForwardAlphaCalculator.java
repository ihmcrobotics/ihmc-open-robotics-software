package us.ihmc.commonWalkingControlModules.capturePoint.controlAngle;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControlAngleFeedForwardAlphaCalculator implements ICPControllerParameters.FeedForwardAlphaCalculator
{
   private static final boolean VISUALIZE = true;

   static final double nominalAngleToConverge = Math.toRadians(7.0);
   private static final double defaultErrorToStartScalingFeedForward = 0.01;
   private static final double defaultErrorToCompleteScalingFeedForward = 0.05;

   private final DoubleProvider minimumICPControlAngle;
   private final DoubleProvider errorToStartScalingFeedForward;
   private final DoubleProvider errorToCompleteScalingFeedForward;

   private final BooleanParameter useICPFeedForwardScaling;

   private final YoDouble icpControlAngle;

   private final YoDouble feedForwardScaleBasedOnError;
   private final YoDouble feedForwardScaleBasedOnAngle;

   private final FrameVector2D directionToReferenceICP = new FrameVector2D();
   private final FrameVector2D directionToFinalICP = new FrameVector2D();
   private final FrameVector2D convergenceVector = new FrameVector2D();

   private final FrameVector2D amountOfCMPFeedback = new FrameVector2D();

   private final FrameLine2D cmpFeedbackLine = new FrameLine2D();
   private final FrameLine2D icpErrorLine = new FrameLine2D();
   private final FrameLine2D convergentLine = new FrameLine2D();

   private final YoFramePoint2D pointOnEdgeWithNoFeedForward;
   private final YoFramePoint2D pointOnEdgeWithAllFeedForward;
   private final YoFramePoint2D pointOnEdgeThatConverges;

   private final YoFrameLineSegment2D worstCaseEdge;
   private final YoFrameLineSegment2D errorEdge;
   private final YoFrameLineSegment2D finalLine;
   private final YoFrameLineSegment2D convergentLineSegment;

   private final FramePoint2D cmpWithNoFeedForward = new FramePoint2D();
   private final FramePoint2D cmpWithAllFeedForward = new FramePoint2D();

   private final FrameLineSegment2D lineToShiftAlong = new FrameLineSegment2D();

   private final FramePoint2D finalICP = new FramePoint2D();
   private final FramePoint2D pointToThrowAway = new FramePoint2D();
   private final FrameVector2D tempDirection = new FrameVector2D();

   public ControlAngleFeedForwardAlphaCalculator(YoRegistry registry)
   {
      minimumICPControlAngle = new DoubleParameter("minimumICPControlAngle", registry, nominalAngleToConverge);
      errorToStartScalingFeedForward = new DoubleParameter("errorToStartScalingFeedForward", registry, defaultErrorToStartScalingFeedForward);
      errorToCompleteScalingFeedForward = new DoubleParameter("errorToCompleteScalingFeedForward", registry, defaultErrorToCompleteScalingFeedForward);

      useICPFeedForwardScaling = new BooleanParameter("useICPFeedForwardScaling", registry, true);

      icpControlAngle = new YoDouble("icpControlAngle", registry);
      feedForwardScaleBasedOnError = new YoDouble("feedForwardScaleBasedOnError", registry);
      feedForwardScaleBasedOnAngle = new YoDouble("feedForwardScaleBasedOnAngle", registry);

      pointOnEdgeWithNoFeedForward = new YoFramePoint2D("pointOnEdgeWithNoFeedForward", ReferenceFrame.getWorldFrame(), registry);
      pointOnEdgeWithAllFeedForward = new YoFramePoint2D("pointOnEdgeWithAllFeedForward", ReferenceFrame.getWorldFrame(), registry);
      pointOnEdgeThatConverges = new YoFramePoint2D("pointOnEdgeThatConverges", ReferenceFrame.getWorldFrame(), registry);

      if (VISUALIZE)
      {
         worstCaseEdge = new YoFrameLineSegment2D("worstCaseEdge", ReferenceFrame.getWorldFrame(), registry);
         errorEdge = new YoFrameLineSegment2D("errorEdge", ReferenceFrame.getWorldFrame(), registry);
         finalLine = new YoFrameLineSegment2D("finalLine", ReferenceFrame.getWorldFrame(), registry);
         convergentLineSegment = new YoFrameLineSegment2D("convergentLine", ReferenceFrame.getWorldFrame(), registry);
      }
      else
      {
         worstCaseEdge = null;
         errorEdge = null;
         finalLine = null;
         convergentLineSegment = null;
      }
   }

   @Override
   public double computeAlpha(FramePoint2DReadOnly currentICP,
                              FramePoint2DReadOnly referenceICP,
                              FramePoint2DReadOnly finalICP,
                              FramePoint2DReadOnly referenceCMP,
                              FramePoint2DReadOnly unconstrainedFeedbackCMP,
                              FrameConvexPolygon2DReadOnly supportPolygon)
   {
      if (!useICPFeedForwardScaling.getValue() || !supportPolygon.isPointInside(currentICP) || !supportPolygon.isPointInside(referenceICP))
         return 0.0;

      this.finalICP.set(finalICP);
      if (this.finalICP.containsNaN())
         this.finalICP.set(referenceICP);

      directionToReferenceICP.sub(referenceICP, currentICP);

      directionToFinalICP.sub(this.finalICP, currentICP);

      double angleFromFinalToReference = directionToFinalICP.angle(directionToReferenceICP);
      icpControlAngle.set(Math.signum(angleFromFinalToReference) * Math.min(minimumICPControlAngle.getValue(), Math.abs(angleFromFinalToReference)));

      RotationMatrixTools.applyYawRotation(icpControlAngle.getDoubleValue(), directionToFinalICP, convergenceVector);

      if(referenceICP.distanceSquared(currentICP) < 1e-3)
      {
         return 0.0;
      }
      
      icpErrorLine.set(referenceICP, currentICP);
      cmpFeedbackLine.set(referenceCMP, unconstrainedFeedbackCMP);

      amountOfCMPFeedback.sub(unconstrainedFeedbackCMP, referenceCMP);
      cmpWithNoFeedForward.add(referenceICP, amountOfCMPFeedback);
      cmpWithAllFeedForward.set(unconstrainedFeedbackCMP);

      // compute the extreme points, and where they interesct the support polygon

      // this point is guaranteed to only have one intersection, since we know the start of the ray is inside the polygoin.
      if (supportPolygon.intersectionWithRay(icpErrorLine, pointOnEdgeWithNoFeedForward, pointToThrowAway) != 1)
      {
         LogTools.error("The ICP line shouldn't have more than one intersection with the edge");
         return 0.0;
      }

      // saturate the feedforward term
      if (referenceICP.distanceSquared(pointOnEdgeWithNoFeedForward) < referenceICP.distanceSquared(cmpWithNoFeedForward))
         cmpWithNoFeedForward.set(pointOnEdgeWithNoFeedForward);

      // saturate the feedback term
      int numberOfIntersections = supportPolygon.intersectionWithRay(cmpFeedbackLine, pointOnEdgeWithAllFeedForward, pointToThrowAway);
      if (numberOfIntersections != 1)
      {
         supportPolygon.intersectionWith(cmpFeedbackLine, pointOnEdgeWithAllFeedForward, pointToThrowAway);
         if (pointOnEdgeWithAllFeedForward.distanceSquared(referenceCMP) > pointToThrowAway.distanceSquared(referenceCMP))
            pointOnEdgeWithAllFeedForward.set(pointToThrowAway);

         cmpWithAllFeedForward.set(pointOnEdgeWithAllFeedForward);
      }
      else
      {
         // saturate the feedback term
         if (pointOnEdgeWithAllFeedForward.distanceSquared(referenceCMP) < cmpWithAllFeedForward.distanceSquared(referenceCMP))
            cmpWithAllFeedForward.set(pointOnEdgeWithAllFeedForward);
      }

      // compute the convergent point
      convergentLine.set(currentICP, convergenceVector);
      convergentLine.getDirection().negate();

      supportPolygon.intersectionWithRay(convergentLine, pointOnEdgeThatConverges, pointToThrowAway);

      if (finalLine != null)
      {
         finalLine.set(this.finalICP, currentICP);
         convergentLineSegment.getFirstEndpoint().set(convergentLine.getPoint());
         convergentLineSegment.getSecondEndpoint().add(convergentLine.getPoint(), convergentLine.getDirection());
         worstCaseEdge.set(referenceCMP, cmpWithAllFeedForward);
         errorEdge.set(referenceICP, cmpWithNoFeedForward);
      }

      lineToShiftAlong.set(unconstrainedFeedbackCMP, cmpWithNoFeedForward);

      tempDirection.sub(cmpWithNoFeedForward, cmpWithAllFeedForward);


//      if (EuclidGeometryTools.areLine2DsCollinear(cmpWithAllFeedForward, tempDirection, currentICP, directionToFinalICP, Math.toRadians(2.0), 1e-5))
//      {
//         feedForwardScaleBasedOnAngle.set(0.0);
//      }
//      else
//      {
         double alphaAngle = EuclidGeometryTools.percentageOfIntersectionBetweenTwoLine2Ds(cmpWithAllFeedForward, tempDirection, currentICP, convergenceVector);
         feedForwardScaleBasedOnAngle.set(MathTools.clamp(alphaAngle, 0.0, 1.0));
//      }

      double alphaError = (referenceICP.distance(currentICP) - errorToStartScalingFeedForward.getValue())
                          / (errorToCompleteScalingFeedForward.getValue() - errorToStartScalingFeedForward.getValue());
      feedForwardScaleBasedOnError.set(MathTools.clamp(alphaError, 0.0, 1.0));

      return feedForwardScaleBasedOnError.getValue() * feedForwardScaleBasedOnAngle.getValue();
   }

   FramePoint2DReadOnly getPointOnEdgeWithNoFeedForward()
   {
      return pointOnEdgeWithNoFeedForward;
   }

   FramePoint2DReadOnly getPointOnEdgeWithFullFeedForward()
   {
      return pointOnEdgeWithAllFeedForward;
   }

   FramePoint2DReadOnly getPointOnEdgeThatConverges()
   {
      return pointOnEdgeThatConverges;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (worstCaseEdge == null)
         return null;

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      ColorDefinition green = ColorDefinitions.Green();
      group.addChild(newYoGraphicPoint2D("No FeedForward", pointOnEdgeWithNoFeedForward, 0.01, green, DefaultPoint2DGraphic.CIRCLE_FILLED));
      group.addChild(newYoGraphicPoint2D("All FeedForward", pointOnEdgeWithAllFeedForward, 0.01, green, DefaultPoint2DGraphic.CIRCLE_FILLED));
      group.addChild(newYoGraphicPoint2D("Convergent Point", pointOnEdgeThatConverges, 0.01, green, DefaultPoint2DGraphic.CIRCLE_FILLED));
      group.addChild(newYoGraphicLineSegment2DDefinition("Worst Case Edge", worstCaseEdge, ColorDefinitions.Red()));
      group.addChild(newYoGraphicLineSegment2DDefinition("Error Edge", errorEdge, ColorDefinitions.Blue()));
      group.addChild(newYoGraphicLineSegment2DDefinition("Final Line", finalLine, ColorDefinitions.Yellow()));
      group.addChild(newYoGraphicLineSegment2DDefinition("Convergent Line", convergentLineSegment, green));
      return group;
   }
}
