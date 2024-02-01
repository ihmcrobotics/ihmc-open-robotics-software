package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_FILLED;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoPProjectionTowardsMidpoint implements ICPControllerParameters.FeedbackProjectionOperator
{
   private static final boolean VISUALIZE = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final String yoNamePrefix = "copProjection";

   private final DoubleProvider minICPPushDelta;
   private final DoubleProvider maxCoPProjectionInside;
   private final BooleanProvider useCoPProjection;

   private final YoDouble dotProductForFootEdgeProjection;
   private final YoDouble momentumShiftedICPOnProjection;
   private final YoDouble firstIntersectionOnProjection;
   private final YoDouble secondIntersectionOnProjection;
   private final YoDouble firstPerfectOnProjection;
   private final YoDouble secondPerfectOnProjection;
   private final YoDouble copAdjustmentAmount;

   private final YoFrameVector2D projectionVector;
   private final YoFrameLine2D projectionLine;

   private final YoFramePoint2D firstProjectionIntersection;
   private final YoFramePoint2D secondProjectionIntersection;
   private final YoFramePoint2D icpProjection;

   private final FrameVector2D tempVector = new FrameVector2D();

   private final FrameVector2D bestPerpendicularVectorAlongNearestFootEdge = new FrameVector2D();

   private final FramePoint2D unconstrainedFeedbackCoP = new FramePoint2D();

   public CoPProjectionTowardsMidpoint(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      minICPPushDelta = new DoubleParameter(yoNamePrefix + "MinICPPushDelta",
                                            "When projecting the CoP into the foot, make sure to not move the CMP any closer than this amount from the ICP",
                                            registry,
                                            0.05);
      maxCoPProjectionInside = new DoubleParameter(yoNamePrefix + "MaxCoPProjectionInside",
                                                   "When projecting the CoP into the foot, move up to this far from the edge if possible",
                                                   registry,
                                                   0.04);
      useCoPProjection = new BooleanParameter(yoNamePrefix + "UseCoPProjection", registry, false);

      dotProductForFootEdgeProjection = new YoDouble(yoNamePrefix + "DotProductForFootEdgeProjection", registry);
      momentumShiftedICPOnProjection = new YoDouble(yoNamePrefix + "momentumShiftedICPOnProjection", registry);
      firstIntersectionOnProjection = new YoDouble(yoNamePrefix + "FirstIntersectionOnProjection", registry);
      secondIntersectionOnProjection = new YoDouble(yoNamePrefix + "SecondIntersectionOnProjection", registry);
      firstPerfectOnProjection = new YoDouble(yoNamePrefix + "FirstPerfectOnProjection", registry);
      secondPerfectOnProjection = new YoDouble(yoNamePrefix + "SecondPerfectOnProjection", registry);
      copAdjustmentAmount = new YoDouble(yoNamePrefix + "COPAdjustmentAmount", registry);

      projectionVector = new YoFrameVector2D(yoNamePrefix + "ProjectionVector", worldFrame, registry);
      projectionLine = new YoFrameLine2D(yoNamePrefix + "ProjectionLine", worldFrame, registry);

      firstProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "FirstIntersection", worldFrame, registry);
      secondProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "SecondIntersection", worldFrame, registry);
      icpProjection = new YoFramePoint2D(yoNamePrefix + "ICPProjection", worldFrame, registry);

      if (yoGraphicsListRegistry == null)
         return;

      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      //TODO: Figure out a viz that works with the logger.

      YoArtifactLine2d projectionLineViz = new YoArtifactLine2d(yoNamePrefix + "ProjectionLine", this.projectionLine, YoAppearance.Aqua().getAwtColor());

      YoGraphicPosition firstIntersectionViz = new YoGraphicPosition(yoNamePrefix + "FirstIntersection",
                                                                     this.firstProjectionIntersection,
                                                                     0.004,
                                                                     YoAppearance.Green(),
                                                                     YoGraphicPosition.GraphicType.SOLID_BALL);

      YoGraphicPosition secondIntersectionViz = new YoGraphicPosition(yoNamePrefix + "SecondIntersection",
                                                                      this.secondProjectionIntersection,
                                                                      0.004,
                                                                      YoAppearance.Green(),
                                                                      YoGraphicPosition.GraphicType.SOLID_BALL);

      YoGraphicPosition icpProjectionViz = new YoGraphicPosition(yoNamePrefix + "ICPProjection",
                                                                 this.icpProjection,
                                                                 0.003,
                                                                 YoAppearance.Purple(),
                                                                 YoGraphicPosition.GraphicType.BALL);

      artifactList.add(firstIntersectionViz.createArtifact());
      artifactList.add(secondIntersectionViz.createArtifact());
      artifactList.add(projectionLineViz);
      artifactList.add(icpProjectionViz.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void projectFeedback(FramePoint2DReadOnly currentICP,
                               FramePoint2DReadOnly unconstrainedFeedbackCMP,
                               FrameVector2DReadOnly cmpOffset,
                               FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                               FixedFramePoint2DBasics feedbackCoPToPack,
                               FixedFramePoint2DBasics feedbackCMPToPack)
   {
      dotProductForFootEdgeProjection.setToNaN();
      icpProjection.setToNaN();
      firstProjectionIntersection.setToNaN();
      secondProjectionIntersection.setToNaN();
      projectionVector.setToNaN();
      projectionLine.setToNaN();

      if (!useCoPProjection.getValue())
         return;

      // Compute the projection vector and projection line.
      projectionVector.sub(currentICP, unconstrainedFeedbackCMP);

      unconstrainedFeedbackCoP.sub(unconstrainedFeedbackCMP, cmpOffset);

      // Here we say momentumShifted in momentumShiftedICPOnProjection, since it is not the actual ICP with respect to the CoP, but instead
      // the ICP with respect to the CMP. So, when we are making the adjustment to keep the CoP inside the support polygon,
      // we are also taking into consideration how the adjusted CMP will be pushing on the ICP.
      momentumShiftedICPOnProjection.set(projectionVector.length());

      // If the projection vector, and hence the error is small and already inside the foot, then do not project at all.
      // But if it is small and not inside the foot, then orthogonally project the CoP into the foot.
      // This prevents any NaN issues and any crazy discontinuities as you get near the foot.
      if (momentumShiftedICPOnProjection.getValue() < 0.002)
      {
         if (supportPolygonInWorld.isPointInside(unconstrainedFeedbackCoP))
         {
            feedbackCoPToPack.set(unconstrainedFeedbackCoP);
         }
         else
         {
            supportPolygonInWorld.orthogonalProjection(unconstrainedFeedbackCoP, feedbackCoPToPack);
         }

         feedbackCMPToPack.add(feedbackCoPToPack, cmpOffset);
         return;
      }

      projectionVector.normalize();
      projectionLine.set(unconstrainedFeedbackCoP, projectionVector);

      supportPolygonInWorld.intersectionWith(projectionLine, firstProjectionIntersection, secondProjectionIntersection);

      // If the projection line from the unconstrainedFeedbackCoP along the projectionVector does not intersect the foot
      // then there is no valid CoP that preserves the angle from the CMP to the ICP. In this case,
      // do a different projection that finds a suitable point on the edge of the foot polygon.
      if (firstProjectionIntersection.containsNaN() || (secondProjectionIntersection.containsNaN()))
      {
         projectCoPToVertex(supportPolygonInWorld, feedbackCoPToPack);
         feedbackCMPToPack.add(feedbackCoPToPack, cmpOffset);

         return;
      }

      // At this point, we have a projection line that intersects the foot in two locations.
      // We now consider how to move the CoP along this line, so that it is inside the foot,
      // away from the edge of the foot if possible, but does not get adjusted if it makes it be too
      // close to the ICP if possible.
      tempVector.sub(firstProjectionIntersection, unconstrainedFeedbackCoP);
      firstIntersectionOnProjection.set(tempVector.dot(projectionVector));

      tempVector.sub(secondProjectionIntersection, unconstrainedFeedbackCoP);
      secondIntersectionOnProjection.set(tempVector.dot(projectionVector));

      // Switch the intersections so that the first one is closest.
      if (firstIntersectionOnProjection.getValue() > secondIntersectionOnProjection.getValue())
      {
         double temp = firstIntersectionOnProjection.getValue();
         firstIntersectionOnProjection.set(secondIntersectionOnProjection.getValue());
         secondIntersectionOnProjection.set(temp);

         tempVector.set(firstProjectionIntersection);
         firstProjectionIntersection.set(secondProjectionIntersection);
         secondProjectionIntersection.set(tempVector);
      }

      //TODO: Decide whether or not to use the perfect points in the projection.
      //TODO: Right now we are just setting them to the intersections.
      firstPerfectOnProjection.set(firstIntersectionOnProjection.getValue());
      secondPerfectOnProjection.set(secondIntersectionOnProjection.getValue());

      copAdjustmentAmount.set(HeuristicICPControllerHelper.computeAdjustmentDistance(momentumShiftedICPOnProjection.getValue(),
                                                                                     firstIntersectionOnProjection.getValue(),
                                                                                     secondIntersectionOnProjection.getValue(),
                                                                                     firstPerfectOnProjection.getValue(),
                                                                                     secondPerfectOnProjection.getValue(),
                                                                                     minICPPushDelta.getValue(),
                                                                                     maxCoPProjectionInside.getValue()));

      // If the adjustment distance is greater than the adjustedICPOnProjection, then that means you are pushing directly backwards on the ICP.
      // Instead, in that case, do a smarter projection, as if the projectionLine does not intersect the foot.
      if (copAdjustmentAmount.getValue() > momentumShiftedICPOnProjection.getValue())
      {
         projectCoPWhenProjectionLineDoesNotIntersectFoot(currentICP, unconstrainedFeedbackCoP, supportPolygonInWorld, feedbackCoPToPack);
         feedbackCMPToPack.add(feedbackCoPToPack, cmpOffset);
         return;
      }

      feedbackCoPToPack.scaleAdd(copAdjustmentAmount.getValue(), projectionVector, unconstrainedFeedbackCoP);
      feedbackCMPToPack.add(feedbackCoPToPack, cmpOffset);
   }

   private void projectCoPToVertex(FrameConvexPolygon2DReadOnly supportPolygonInWorld, FixedFramePoint2DBasics feedbackCoPToPack)
   {
      supportPolygonInWorld.getClosestVertex(projectionLine, feedbackCoPToPack);
   }

   /**
    * Projects the CoP into the foot when the projection line does not intersect the foot. This
    * typically means that the robot is having lots of trouble with balance. At this point, either a
    * higher level step adjustment needs to be kicking in, or it is likely that the robot will fall
    * soon. Uses the following algorithm: Project the ICP onto the foot. Then figure out which
    * direction is best to move perpendicular from there to try to help things out. Then move in that
    * perpendicular direction a little bit more than the distance from the foot to the ICP times a
    * scale factor. (Note this distance if made infinite, would just find the line of sight point,
    * which is a pretty good answer most of the time. However, the line of sight point can be
    * unnecessarily too far if it is not helping much. By going just a little more than a scaled amount
    * of the distance, you make sure you do not fight the badness that comes after 45 degrees, where
    * you increase the ICP expected velocity a lot, just to get a little bit more angle.) Then project
    * that point back into the foot, which is necessary since you might have gone around a "corner" in
    * the foot. If the corner is shallow, things will be pretty much just like there was no corner. If
    * the corner is sharp, then you will not unnecessarily move "around the corner" too much, ruining
    * the angle. Since this is only called when the projection line does not intersect the foot, it is
    * not entirely critical to have this be perfect, since this method only gets called when the robot
    * is having lots of balance trouble, and is likely about to fall if it does not take a really quick
    * recovery step. So a higher level thing should be taking over instead. Note that this method
    * ignores the difference between the perfectCMP and the perfectCoP and just finds a CoP that is not
    * horrible, given the circumstances.
    */
   private void projectCoPWhenProjectionLineDoesNotIntersectFoot(FramePoint2DReadOnly currentICP,
                                                                 FramePoint2DReadOnly unconstrainedFeedbackCoP,
                                                                 FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                                                                 FixedFramePoint2DBasics feedbackCoPToPack)
   {
      supportPolygonInWorld.orthogonalProjection(currentICP, icpProjection);

      tempVector.sub(currentICP, unconstrainedFeedbackCoP);
      tempVector.normalize();
      if (tempVector.containsNaN())
      {
         feedbackCoPToPack.set(icpProjection);
         return;
      }

      bestPerpendicularVectorAlongNearestFootEdge.sub(currentICP, icpProjection);
      double distanceFromProjectionToICP = bestPerpendicularVectorAlongNearestFootEdge.length();

      if (distanceFromProjectionToICP < 0.001)
      {
         feedbackCoPToPack.set(icpProjection);
         return;
      }

      bestPerpendicularVectorAlongNearestFootEdge.scale(1.0 / distanceFromProjectionToICP);
      bestPerpendicularVectorAlongNearestFootEdge.set(-bestPerpendicularVectorAlongNearestFootEdge.getY(), bestPerpendicularVectorAlongNearestFootEdge.getX());

      dotProductForFootEdgeProjection.set(tempVector.dot(bestPerpendicularVectorAlongNearestFootEdge));

      if (dotProductForFootEdgeProjection.getValue() > 0.0)
         bestPerpendicularVectorAlongNearestFootEdge.scale(-1.0);
      else
      {
         dotProductForFootEdgeProjection.mul(-1.0);
      }

      // These parameters are to smooth the transition from jumping all the way to a line of sight point. Line of sight point will give the best angle,
      // but at the expense of large distance, thereby speeding up the ICP a lot. So only move towards a line of sight point when the distance is going to
      // be large anyway. These parameters are not terribly critical and should not be user tuned since other high level things should be kicking in
      // instead when we are at this point. They are just here to help when the ICP is still close to the foot and therefore, slowing down the
      // velocity of the ICP might be beneficial to help provide time for a recovery step.
      double scaleDistanceFromICP = 2.5;
      double addDistanceToPerpendicular = 0.06;
      double dotProductThresholdBeforeMovingInPerpendicularDirection = 0.25;

      double amountToMoveInPerpendicularDirection = (scaleDistanceFromICP * distanceFromProjectionToICP) + addDistanceToPerpendicular;
      double amountToScaleFromDotProduct = (dotProductForFootEdgeProjection.getValue() - dotProductThresholdBeforeMovingInPerpendicularDirection)
                                           / (1.0 - dotProductThresholdBeforeMovingInPerpendicularDirection);
      amountToScaleFromDotProduct = MathTools.clamp(amountToScaleFromDotProduct, 0.0, 1.0);

      amountToMoveInPerpendicularDirection = amountToMoveInPerpendicularDirection * amountToScaleFromDotProduct;

      bestPerpendicularVectorAlongNearestFootEdge.scale(amountToMoveInPerpendicularDirection);
      feedbackCoPToPack.add(icpProjection, bestPerpendicularVectorAlongNearestFootEdge);

      supportPolygonInWorld.orthogonalProjection(feedbackCoPToPack);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicPoint2D("FirstIntersection", firstProjectionIntersection, 0.008, ColorDefinitions.Green(), CIRCLE_FILLED));
      group.addChild(newYoGraphicPoint2D("SecondIntersection", secondProjectionIntersection, 0.008, ColorDefinitions.Green(), CIRCLE_FILLED));
      group.addChild(newYoGraphicPoint2D("ICPProjection", icpProjection, 0.006, ColorDefinitions.Purple(), CIRCLE));
      // TODO Need to implement infinite line 2D?
      //      group.addChild(newYoGraphicLineSegment2DDefinition("ProjectionLine", lineSegment, strokeColor));
      return group;
   }
}
