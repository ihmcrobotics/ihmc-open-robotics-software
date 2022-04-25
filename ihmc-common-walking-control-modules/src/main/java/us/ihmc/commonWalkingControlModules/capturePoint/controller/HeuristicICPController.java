package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * HeuristicICPController controls the ICP using a few simple heuristics, including: a) Use a simple
 * proportional controller on the ICP error for a feedback term. b) Use the perfect CMP for a
 * feedforward term. c) If there is a large perpendicular error, ignore the feedforward term and
 * only do the feedback term. d) Project the unconstrained CoP answer into the foot along the Vector
 * from the unconstrained CMP to the ICP, but do not project too far into the foot, and also do not
 * project closer to the ICP than a certain threshold.
 **/
public class HeuristicICPController implements ICPControllerInterface
{
   //TODO: Similar to the optimization ICP controller, if control cannot be achieved well due to the large difference between perfectCoP and perfectCMP, then change that distance and set a YoBoolean, like useCMPFeedback and useAngularMomentum.
   //TODO: Perhaps add either an integrator, or a constant feedback term to better achieve the control?
   //TODO: Perhaps add a LPF or rate limit, or add them outside this class?

   private static final boolean VISUALIZE = true;

   private final String yoNamePrefix = "heuristic";
   private final YoRegistry registry = new YoRegistry("HeuristicICPController");
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // Control Parameters:
   private final YoDouble pureFeedbackErrorThreshold = new YoDouble(yoNamePrefix + "PureFeedbackErrorThresh",
                                                                    "Amount of ICP error before feedforward terms are ignored.",
                                                                    registry);
   private final YoDouble minICPPushDelta = new YoDouble(yoNamePrefix
         + "MinICPPushDelta", "When projecting the CoP into the foot, make sure to not move the CMP any closer than this amount from the ICP", registry);
   private final YoDouble maxCoPProjectionInside = new YoDouble(yoNamePrefix + "MaxCoPProjectionInside",
                                                                "When projecting the CoP into the foot, move up to this far from the edge if possible",
                                                                registry);

   private final ICPControlGainsReadOnly feedbackGains;

   // Algorithm Inputs:
   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D parallelDirection = new FrameVector2D();
   private final FrameVector2D perpDirection = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FrameVector2D currentCoMVelocity = new FrameVector2D();

   private final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   private final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   // Feedback control computations before projection (unconstrained)
   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoDouble icpErrorMagnitude = new YoDouble(yoNamePrefix + "ICPErrorMagnitude", registry);
   private final YoDouble icpParallelError = new YoDouble(yoNamePrefix + "ICPParallelError", registry);
   private final YoDouble icpPerpError = new YoDouble(yoNamePrefix + "ICPPerpError", registry);

   private final YoFrameVector2D pureFeedforwardControl = new YoFrameVector2D(yoNamePrefix + "PureFeedforwardControl", "", worldFrame, registry);
   private final YoDouble pureFeedforwardMagnitude = new YoDouble(yoNamePrefix + "PureFeedforwardMagnitude", registry);

   private final YoFrameVector2D pureFeedbackControl = new YoFrameVector2D(yoNamePrefix + "PureFeedbackControl", "", worldFrame, registry);
   private final YoDouble pureFeedbackMagnitude = new YoDouble(yoNamePrefix + "PureFeedbackMagnitude", registry);

   private final YoDouble feedbackFeedforwardAlpha = new YoDouble(yoNamePrefix + "FeedbackFeedforwardAlpha", registry);

   private final YoDouble icpParallelFeedback = new YoDouble(yoNamePrefix + "ICPParallelFeedback", registry);
   private final YoDouble icpPerpFeedback = new YoDouble(yoNamePrefix + "ICPPerpFeedback", registry);

   private final YoFrameVector2D unconstrainedFeedback = new YoFrameVector2D(yoNamePrefix + "UnconstrainedFeedback", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCMP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCMP", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCoP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCoP", worldFrame, registry);

   // Projection computation variables:
   private final YoDouble dotProductForFootEdgeProjection = new YoDouble(yoNamePrefix + "DotProductForFootEdgeProjection", registry);
   private final YoDouble momentumShiftedICPOnProjection = new YoDouble(yoNamePrefix + "momentumShiftedICPOnProjection", registry);
   private final YoDouble firstIntersectionOnProjection = new YoDouble(yoNamePrefix + "FirstIntersectionOnProjection", registry);
   private final YoDouble secondIntersectionOnProjection = new YoDouble(yoNamePrefix + "SecondIntersectionOnProjection", registry);
   private final YoDouble firstPerfectOnProjection = new YoDouble(yoNamePrefix + "FirstPerfectOnProjection", registry);
   private final YoDouble secondPerfectOnProjection = new YoDouble(yoNamePrefix + "SecondPerfectOnProjection", registry);
   private final YoDouble copAdjustmentAmount = new YoDouble(yoNamePrefix + "COPAdjustmentAmount", registry);

   private final YoFrameVector2D projectionVector = new YoFrameVector2D(yoNamePrefix + "ProjectionVector", worldFrame, registry);
   private final YoFrameLine2D projectionLine = new YoFrameLine2D(yoNamePrefix + "ProjectionLine", worldFrame, registry);

   private final YoFramePoint2D firstProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "FirstIntersection", worldFrame, registry);
   private final YoFramePoint2D secondProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "SecondIntersection", worldFrame, registry);
   private final YoFramePoint2D closestPointWithProjectionLine = new YoFramePoint2D(yoNamePrefix + "ClosestPointWithProjectionLine", worldFrame, registry);

   private final YoFramePoint2D icpProjection = new YoFramePoint2D(yoNamePrefix + "ICPProjection", worldFrame, registry);

   // Outputs:
   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   private final YoFrameVector2D expectedControlICPVelocity = new YoFrameVector2D(yoNamePrefix + "ExpectedControlICPVelocity", worldFrame, registry);

   private final YoFrameVector2D residualError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final FrameVector2D tempVector = new FrameVector2D();
   private final FrameVector2D tempVectorTwo = new FrameVector2D();

   private final FrameVector2D bestPerpendicularVectorAlongNearestFootEdge = new FrameVector2D();

   public HeuristicICPController(ICPControllerParameters icpControllerParameters,
                                 double controlDT,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      pureFeedbackErrorThreshold.set(icpControllerParameters.getPureFeedbackErrorThreshold());
      minICPPushDelta.set(icpControllerParameters.getMinICPPushDelta());
      maxCoPProjectionInside.set(icpControllerParameters.getMaxCoPProjectionInside());

      pureFeedbackErrorThreshold.set(0.06);
      minICPPushDelta.set(0.05);
      maxCoPProjectionInside.set(0.04);

      feedbackGains = new ParameterizedICPControlGains("", icpControllerParameters.getICPFeedbackGains(), registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public ICPControlGainsReadOnly getFeedbackGains()
   {
      return feedbackGains;
   }

   private final FrameVector2D desiredCMPOffsetToThrowAway = new FrameVector2D();

   @Override
   public void compute(FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                       FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly finalICP,
                       FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, finalICP, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentCoMPosition, omega0);
   }

   @Override
   public void compute(FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                       FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly finalICP,
                       FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      //TODO: Try working in velocity and angle space instead of xy space.
      // Have a gain on the velocity based on whether leading or lagging. 
      // Then have a gain on the angle or something to push towards the ICP direction line.
      // This should reduce the dependence on the perfect CoP/CMP, which can throw things off
      // when there is a big error. And also should not cause so much outside to project fixes.
      // Especially if you limit the amount of velocity increase you can have.
      controllerTimer.startMeasurement();

      this.desiredICP.setMatchingFrame(desiredICP);
      this.desiredICPVelocity.setMatchingFrame(desiredICPVelocity);
      this.perfectCMPOffset.setMatchingFrame(perfectCMPOffset);
      this.currentICP.setMatchingFrame(currentICP);
      this.currentCoMPosition.setMatchingFrame(currentCoMPosition);

      CapturePointTools.computeCenterOfMassVelocity(currentCoMPosition, currentICP, omega0, currentCoMVelocity);

      this.perfectCoP.setMatchingFrame(perfectCoP);
      this.perfectCMP.add(this.perfectCoP, this.perfectCMPOffset);

      this.icpError.sub(currentICP, desiredICP);
      this.icpErrorMagnitude.set(icpError.length());

      this.parallelDirection.set(this.desiredICPVelocity);

      if (parallelDirection.lengthSquared() > 1e-7)
      {
         parallelDirection.normalize();
         perpDirection.set(-parallelDirection.getY(), parallelDirection.getX());

         icpParallelError.set(icpError.dot(parallelDirection));
         icpPerpError.set(icpError.dot(perpDirection));
      }
      else
      {
         parallelDirection.setToNaN();

         perpDirection.set(icpError);
         perpDirection.normalize();
         if (perpDirection.containsNaN())
         {
            perpDirection.set(1.0, 0.0);
         }

         icpPerpError.set(icpError.length());
         icpParallelError.set(0.0);
      }

      icpParallelFeedback.set(icpParallelError.getValue());
      icpParallelFeedback.mul(feedbackGains.getKpParallelToMotion());

      icpPerpFeedback.set(icpPerpError.getValue());
      icpPerpFeedback.mul(feedbackGains.getKpOrthogonalToMotion());

      pureFeedbackControl.set(icpError);
      pureFeedbackControl.scale(feedbackGains.getKpOrthogonalToMotion());
      pureFeedbackMagnitude.set(pureFeedbackControl.length());

      pureFeedforwardControl.sub(perfectCMP, desiredICP);
      pureFeedforwardMagnitude.set(pureFeedforwardControl.length());

      // Compute feedbackFeedforwardAlpha, which if 1.0 means to ignore the feedforward terms from the perfectCoP/CMP.
      // If it equals 0.0, then use all of the feedforward.
      // As the perpendicular error grows, start ignoring the feedforward at a certain percentage of the threshold.
      // Ignore the feedforward more and more as the perpendicular error grows. If the perpendicular error is greater
      // than pureFeedbackErrorThreshold, then apply only feedback.
      double percentOfPerpendicularThresholdToStartIgnoringFeedforward = 0.5;
      double perpendicularErrorToStartIgnoringFeedforward = pureFeedbackErrorThreshold.getValue() * percentOfPerpendicularThresholdToStartIgnoringFeedforward;
      double perpendicularErrorMagnitude = Math.abs(icpPerpError.getValue());
      feedbackFeedforwardAlpha.set(computePercentageOfRangeClampedBetweenZeroAndOne(perpendicularErrorMagnitude, perpendicularErrorToStartIgnoringFeedforward, pureFeedbackErrorThreshold.getValue()));

      icpParallelFeedback.set(MathTools.clamp(icpParallelFeedback.getValue(), feedbackGains.getFeedbackPartMaxValueParallelToMotion()));
      icpPerpFeedback.set(MathTools.clamp(icpPerpFeedback.getValue(), feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion()));

      // Add the scaled amount of the feedforward to the feedback control based on the perpendicular error.
      unconstrainedFeedback.scaleAdd(1.0 - feedbackFeedforwardAlpha.getValue(), pureFeedforwardControl, pureFeedbackControl);

      unconstrainedFeedbackCMP.add(currentICP, unconstrainedFeedback);
      unconstrainedFeedbackCoP.sub(unconstrainedFeedbackCMP, perfectCMPOffset);

      projectCoPIntoFootTowardsMidpointIfPossible(supportPolygonInWorld);

      feedbackCMP.add(feedbackCoP, perfectCMPOffset);

      expectedControlICPVelocity.sub(currentICP, feedbackCMP);
      expectedControlICPVelocity.scale(omega0);

      controllerTimer.stopMeasurement();
   }
   
   private static double computePercentageOfRangeClampedBetweenZeroAndOne(double value, double lowerBoundOfRange, double upperBoundOfRange)
   {
      double percent = (value - lowerBoundOfRange) / (upperBoundOfRange - lowerBoundOfRange);
      return EuclidCoreTools.clamp(percent, 0.0, 1.0);
   }

   /**
    * Project the CoP further into the interior of the support polygon. If we can, we will move it such
    * that the angle from the CMP to the ICP does not change, but only the distance changes. Therefore
    * the projectionVector will be in that direction. We need to make sure that the CoP ends up inside
    * the foot. So we will be moving the CoP along the projectionVector, to get it away from the edge
    * of the foot a bit. If the CoP is outside the foot, then we will project it inside the foot using
    * the projectionVector. However, if the line defined by the CoP and the projectionVector does not
    * intersect the foot, then there is no valid CoP inside the foot that will preserve the angle from
    * the CMP to the ICP. In this case, we will use some heuristics to find a point on the edge of the
    * foot that achieves a tradeoff between how much the angle changes and how fast the ICP will move
    * in the end. When we have to do this projection, things are probably looking pretty bad for the
    * balance of the robot, so a point that is close to the ICP is beneficial, to slow it down so that
    * a higher level has time for something like a step adjustment.
    * 
    */
   private void projectCoPIntoFootTowardsMidpointIfPossible(FrameConvexPolygon2DReadOnly supportPolygonInWorld)
   {
      dotProductForFootEdgeProjection.setToNaN();
      icpProjection.setToNaN();
      firstProjectionIntersection.setToNaN();
      secondProjectionIntersection.setToNaN();
      closestPointWithProjectionLine.setToNaN();
      projectionVector.setToNaN();
      projectionLine.setToNaN();

      // Compute the projection vector and projection line. 
      projectionVector.sub(currentICP, unconstrainedFeedbackCMP);

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
            feedbackCoP.set(unconstrainedFeedbackCoP);
            return;
         }
         else
         {
            supportPolygonInWorld.orthogonalProjection(unconstrainedFeedbackCoP, feedbackCoP);
            return;
         }
      }

      projectionVector.normalize();
      projectionLine.set(unconstrainedFeedbackCoP, projectionVector);

      supportPolygonInWorld.intersectionWith(projectionLine, firstProjectionIntersection, secondProjectionIntersection);

      // If the projection line from the unconstrainedFeedbackCoP along the projectionVector does not intersect the foot
      // then there is no valid CoP that preserves the angle from the CMP to the ICP. In this case, 
      // do a different projection that finds a suitable point on the edge of the foot polygon.
      if (firstProjectionIntersection.containsNaN() || (secondProjectionIntersection.containsNaN()))
      {
         projectCoPWhenProjectionLineDoesNotIntersectFoot(supportPolygonInWorld);
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
         projectCoPWhenProjectionLineDoesNotIntersectFoot(supportPolygonInWorld);
         return;
      }

      feedbackCoP.scaleAdd(copAdjustmentAmount.getValue(), projectionVector, unconstrainedFeedbackCoP);
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
   private void projectCoPWhenProjectionLineDoesNotIntersectFoot(FrameConvexPolygon2DReadOnly supportPolygonInWorld)
   {
      supportPolygonInWorld.orthogonalProjection(currentICP, icpProjection);

      tempVector.sub(currentICP, unconstrainedFeedbackCoP);
      tempVector.normalize();
      if (tempVector.containsNaN())
      {
         feedbackCoP.set(icpProjection);
         return;
      }

      bestPerpendicularVectorAlongNearestFootEdge.sub(currentICP, icpProjection);
      double distanceFromProjectionToICP = bestPerpendicularVectorAlongNearestFootEdge.length();

      if (distanceFromProjectionToICP < 0.001)
      {
         feedbackCoP.set(icpProjection);
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
      double amountToScaleFromDotProduct = (dotProductForFootEdgeProjection.getValue() - dotProductThresholdBeforeMovingInPerpendicularDirection) / (1.0 - dotProductThresholdBeforeMovingInPerpendicularDirection);
      amountToScaleFromDotProduct = MathTools.clamp(amountToScaleFromDotProduct, 0.0, 1.0);

      amountToMoveInPerpendicularDirection = amountToMoveInPerpendicularDirection * amountToScaleFromDotProduct;

      bestPerpendicularVectorAlongNearestFootEdge.scale(amountToMoveInPerpendicularDirection);
      feedbackCoP.add(icpProjection, bestPerpendicularVectorAlongNearestFootEdge);

      supportPolygonInWorld.orthogonalProjection(feedbackCoP);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition feedbackCoPViz = new YoGraphicPosition(yoNamePrefix
            + "FeedbackCoP", this.feedbackCoP, 0.005, YoAppearance.Darkorange(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCMPViz = new YoGraphicPosition(yoNamePrefix
            + "UnconstrainedFeedbackCMP", this.unconstrainedFeedbackCMP, 0.008, Purple(), GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCoPViz = new YoGraphicPosition(yoNamePrefix
            + "UnconstrainedFeedbackCoP", this.unconstrainedFeedbackCoP, 0.004, YoAppearance.Green(), GraphicType.BALL_WITH_ROTATED_CROSS);

      //TODO: Figure out a viz that works with the logger.
      YoArtifactLine2d projectionLineViz = new YoArtifactLine2d(yoNamePrefix + "ProjectionLine", this.projectionLine, YoAppearance.Aqua().getAwtColor());

      YoGraphicPosition firstIntersectionViz = new YoGraphicPosition(yoNamePrefix
            + "FirstIntersection", this.firstProjectionIntersection, 0.004, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition secondIntersectionViz = new YoGraphicPosition(yoNamePrefix
            + "SecondIntersection", this.secondProjectionIntersection, 0.004, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition closestPointWithProjectionLineViz = new YoGraphicPosition(yoNamePrefix
            + "ClosestToProjectionLine", this.closestPointWithProjectionLine, 0.003, YoAppearance.Green(), GraphicType.SOLID_BALL);

      //      YoGraphicPosition copProjectionViz = new YoGraphicPosition(yoNamePrefix + "CoPProjection", this.coPProjection, 0.002, YoAppearance.Green(), GraphicType.SOLID_BALL);
      YoGraphicPosition icpProjectionViz = new YoGraphicPosition(yoNamePrefix
            + "ICPProjection", this.icpProjection, 0.003, YoAppearance.Purple(), GraphicType.BALL);

      artifactList.add(feedbackCoPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCMPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCoPViz.createArtifact());
      //      artifactList.add(projectionLineViz);

      artifactList.add(firstIntersectionViz.createArtifact());
      artifactList.add(secondIntersectionViz.createArtifact());
      artifactList.add(closestPointWithProjectionLineViz.createArtifact());
      //      artifactList.add(copProjectionViz.createArtifact());
      artifactList.add(icpProjectionViz.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
   }

   @Override
   public void getDesiredCMP(FixedFramePoint2DBasics desiredCMPToPack)
   {
      desiredCMPToPack.set(feedbackCMP);
   }

   @Override
   public void getDesiredCoP(FixedFramePoint2DBasics desiredCoPToPack)
   {
      desiredCoPToPack.set(feedbackCoP);
   }

   @Override
   public void getExpectedControlICPVelocity(FixedFrameVector2DBasics expectedControlICPVelocityToPack)
   {
      expectedControlICPVelocityToPack.set(expectedControlICPVelocity);
   }

   @Override
   public boolean useAngularMomentum()
   {
      return false;
   }

   @Override
   public FrameVector2DReadOnly getResidualError()
   {
      return residualError;
   }

}
