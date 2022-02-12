package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeuristicICPController implements ICPControllerInterface
{
   private static final boolean VISUALIZE = true;

   private final String yoNamePrefix = "heuristic";
   private final YoRegistry registry = new YoRegistry("HeuristicICPController");

   private final YoDouble adjustedICP = new YoDouble(yoNamePrefix + "AdjustedICP", registry);
   private final YoDouble firstIntersection = new YoDouble(yoNamePrefix + "FirstIntersection", registry);
   private final YoDouble secondIntersection = new YoDouble(yoNamePrefix + "SecondIntersection", registry);
   private final YoDouble firstPerfect = new YoDouble(yoNamePrefix + "FirstPerfect", registry);
   private final YoDouble secondPerfect = new YoDouble(yoNamePrefix + "SecondPerfect", registry);
   private final YoDouble minICPPushDelta = new YoDouble(yoNamePrefix + "MinICPPushDelta", registry);
   private final YoDouble adjustmentDistance = new YoDouble(yoNamePrefix + "AdjustmentDistance", registry);

   private final BooleanProvider useCMPFeedback;
   private final BooleanProvider useAngularMomentum;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ICPControlGainsReadOnly feedbackGains;

   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoDouble icpErrorMagnitude = new YoDouble(yoNamePrefix + "ICPErrorMagnitude", registry);
   private final YoDouble icpParallelError = new YoDouble(yoNamePrefix + "ICPParallelError", registry);
   private final YoDouble icpPerpError = new YoDouble(yoNamePrefix + "ICPPerpError", registry);

   private final YoFrameVector2D pureFeedforwardControl = new YoFrameVector2D(yoNamePrefix + "PureFeedforwardControl", "", worldFrame, registry);
   private final YoDouble pureFeedforwardMagnitude = new YoDouble(yoNamePrefix + "PureFeedforwardMagnitude", registry);

   private final YoFrameVector2D pureFeedbackControl = new YoFrameVector2D(yoNamePrefix + "PureFeedbackControl", "", worldFrame, registry);
   private final YoDouble pureFeedbackMagnitude = new YoDouble(yoNamePrefix + "PureFeedbackMagnitude", registry);

   private final YoDouble feedbackFeedforwardAlpha = new YoDouble(yoNamePrefix + "FeedbackFeedforwardAlpha", registry);
   private final YoDouble pureFeedbackThreshError = new YoDouble(yoNamePrefix + "PureFeedbackThreshError", registry);

   private final YoDouble icpParallelFeedback = new YoDouble(yoNamePrefix + "ICPParallelFeedback", registry);
   private final YoDouble icpPerpFeedback = new YoDouble(yoNamePrefix + "ICPPerpFeedback", registry);

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D parallelDirection = new FrameVector2D();
   private final FrameVector2D perpDirection = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FrameVector2D currentCoMVelocity = new FrameVector2D();

   private final YoFrameVector2D unconstrainedFeedback = new YoFrameVector2D(yoNamePrefix + "UnconstrainedFeedback", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCMP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCMP", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCoP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCoP", worldFrame, registry);

   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D coPProjection = new YoFramePoint2D(yoNamePrefix + "CoPProjection", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);

   final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   private final YoFrameVector2D projectionVector = new YoFrameVector2D(yoNamePrefix + "ProjectionVector", worldFrame, registry);
   private final YoFrameLine2D projectionLine = new YoFrameLine2D(yoNamePrefix + "ProjectionLine", worldFrame, registry);

   private final YoFramePoint2D firstProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "FirstIntersection", worldFrame, registry);
   private final YoFramePoint2D secondProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "SecondIntersection", worldFrame, registry);
   private final YoFramePoint2D closestPointWithProjectionLine = new YoFramePoint2D(yoNamePrefix + "ClosestPointWithProjectionLine", worldFrame, registry);

   private final YoFrameVector2D residualError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final double controlDT;
   private final double controlDTSquare;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   private final BipedSupportPolygons bipedSupportPolygons;

   private final FrameVector2D tempVector = new FrameVector2D();

   public HeuristicICPController(WalkingControllerParameters walkingControllerParameters,
                                 BipedSupportPolygons bipedSupportPolygons,
                                 ICPControlPolygons icpControlPolygons,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                 double controlDT,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons, contactableFeet, controlDT, parentRegistry,
           yoGraphicsListRegistry);
   }

   public HeuristicICPController(WalkingControllerParameters walkingControllerParameters,
                                 ICPOptimizationParameters icpOptimizationParameters,
                                 BipedSupportPolygons bipedSupportPolygons,
                                 ICPControlPolygons icpControlPolygons,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                 double controlDT,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      pureFeedbackThreshError.set(0.2);
      minICPPushDelta.set(0.03);

      this.controlDT = controlDT;
      this.controlDTSquare = controlDT * controlDT;

      this.bipedSupportPolygons = bipedSupportPolygons;

      useCMPFeedback = new BooleanParameter(yoNamePrefix + "UseCMPFeedback", registry, icpOptimizationParameters.useCMPFeedback());
      useAngularMomentum = new BooleanParameter(yoNamePrefix + "UseAngularMomentum", registry, icpOptimizationParameters.useAngularMomentum());

      feedbackGains = new ParameterizedICPControlGains("", icpOptimizationParameters.getICPFeedbackGains(), registry);

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
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(desiredICP, desiredICPVelocity, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentCoMPosition, omega0);
   }

   @Override
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
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
      //      pureFeedbackControl.clipToMaxLength(maxLength);

      pureFeedforwardControl.sub(perfectCMP, desiredICP);
      pureFeedforwardMagnitude.set(pureFeedforwardControl.length());

      //      if (icpErrorMagnitude.getValue() >= pureFeedbackThreshError.getValue())
      if (Math.abs(icpPerpError.getValue()) >= pureFeedbackThreshError.getValue())
      {
         feedbackFeedforwardAlpha.set(1.0);
      }
      else
      {
         //         feedbackFeedforwardAlpha.set(icpErrorMagnitude.getValue()/pureFeedbackThreshError.getValue());
         double perpErrorAdjusted = Math.abs(icpPerpError.getValue()) - pureFeedbackThreshError.getValue() / 2.0;
         if (perpErrorAdjusted < 0.0)
            perpErrorAdjusted = 0.0;

         feedbackFeedforwardAlpha.set(perpErrorAdjusted / (pureFeedbackThreshError.getValue() / 2.0));
      }

      limitAbsoluteValue(icpParallelFeedback, feedbackGains.getFeedbackPartMaxValueParallelToMotion());
      limitAbsoluteValue(icpPerpFeedback, feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion());

      //      unconstrainedFeedback.interpolate(pureFeedforwardControl, pureFeedbackControl, feedbackFeedforwardAlpha.getValue());

      unconstrainedFeedback.set(pureFeedforwardControl);
      unconstrainedFeedback.scale(1.0 - feedbackFeedforwardAlpha.getValue());
      unconstrainedFeedback.add(pureFeedbackControl);

      unconstrainedFeedbackCMP.set(currentICP);
      unconstrainedFeedbackCMP.add(unconstrainedFeedback);

      //      unconstrainedFeedback.setToZero();
      //      if (!parallelDirection.containsNaN())
      //      {
      //         tempVector.set(parallelDirection);
      //         tempVector.scale(icpParallelFeedback.getValue());
      //         unconstrainedFeedback.add(tempVector);
      //      }
      //
      //      if (!perpDirection.containsNaN())
      //      {
      //         tempVector.set(perpDirection);
      //         tempVector.scale(icpPerpFeedback.getValue());
      //         unconstrainedFeedback.add(tempVector);
      //      }
      //
      //      unconstrainedFeedbackCMP.add(perfectCoP, perfectCMPOffset);
      //      unconstrainedFeedbackCMP.add(icpError);
      //      unconstrainedFeedbackCMP.add(unconstrainedFeedback);

      unconstrainedFeedbackCoP.set(unconstrainedFeedbackCMP);
      unconstrainedFeedbackCoP.sub(perfectCMPOffset);

      projectTowardsMidpoint();

      feedbackCMP.set(feedbackCoP);
      feedbackCMP.add(perfectCMPOffset);

      controllerTimer.stopMeasurement();
   }

   private void projectTowardsMidpoint()
   {
      // Project the CoP onto the support polygon, using a projection vector.
      coPProjection.setToNaN();
      firstProjectionIntersection.setToNaN();
      secondProjectionIntersection.setToNaN();
      closestPointWithProjectionLine.setToNaN();
      projectionVector.setToNaN();
      projectionLine.setToNaN();

      // Determine the closest point, intersection point, etc.
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      //Compute the projection vector and projection line:
      projectionVector.set(currentICP);
      projectionVector.sub(unconstrainedFeedbackCMP);

      adjustedICP.set(projectionVector.length());

      //TODO: What if projection vector length is small?
      projectionVector.normalize();

      projectionLine.set(unconstrainedFeedbackCoP, projectionVector);

      supportPolygonInWorld.intersectionWith(projectionLine, firstProjectionIntersection, secondProjectionIntersection);

      if (firstProjectionIntersection.containsNaN() || (secondProjectionIntersection.containsNaN()))
      {
         projectWhenProjectionLineDoesNotIntersectFoot(supportPolygonInWorld);
         return;
      }

      tempVector.set(firstProjectionIntersection);
      tempVector.sub(unconstrainedFeedbackCoP);
      firstIntersection.set(tempVector.dot(projectionVector));

      tempVector.set(secondProjectionIntersection);
      tempVector.sub(unconstrainedFeedbackCoP);
      secondIntersection.set(tempVector.dot(projectionVector));

      if (firstIntersection.getValue() > secondIntersection.getValue())
      {
         double temp = firstIntersection.getValue();
         firstIntersection.set(secondIntersection.getValue());
         secondIntersection.set(temp);
         
         tempVector.set(firstProjectionIntersection);
         firstProjectionIntersection.set(secondProjectionIntersection);
         secondProjectionIntersection.set(tempVector);
      }

      firstPerfect.set(firstIntersection.getValue());
      secondPerfect.set(secondIntersection.getValue());

      //      tempVector.set(perfectCoP);
      //      tempVector.sub(unconstrainedFeedbackCMP);
      //      firstPerfect.set(tempVector.dot(projectionVector));
      //      secondPerfect.set(firstPerfect.getValue());
      //
      //      firstPerfect.sub(0.01);
      //      secondPerfect.add(0.01);

      adjustmentDistance.set(HeuristicICPControllerHelper.computeAdjustmentDistance(adjustedICP.getValue(),
                                                                                    firstIntersection.getValue(),
                                                                                    secondIntersection.getValue(),
                                                                                    firstPerfect.getValue(),
                                                                                    secondPerfect.getValue(),
                                                                                    minICPPushDelta.getValue()));

      tempVector.set(projectionVector);
      tempVector.scale(adjustmentDistance.getValue());
      feedbackCoP.set(unconstrainedFeedbackCoP);
      feedbackCoP.add(tempVector);
   }

   private void projectWhenProjectionLineDoesNotIntersectFoot(FrameConvexPolygon2DReadOnly supportPolygonInWorld)
   {
      coPProjection.set(unconstrainedFeedbackCoP);
      supportPolygonInWorld.orthogonalProjection(coPProjection);

      supportPolygonInWorld.getClosestPointWithRay(projectionLine, closestPointWithProjectionLine);

      tempVector.set(currentICP);
      tempVector.sub(coPProjection);
      tempVector.normalize();
      double copProjectionDot = tempVector.dot(projectionVector);

      tempVector.set(currentICP);
      tempVector.sub(closestPointWithProjectionLine);
      tempVector.normalize();
      double closestPointWithProjectionLineDot = tempVector.dot(projectionVector);

      // TODO: Not sure if this is what we want to be doing here. Might be something smarter.
      if (copProjectionDot >= closestPointWithProjectionLineDot)
         feedbackCoP.set(coPProjection);
      else
         feedbackCoP.set(closestPointWithProjectionLine);

      //            double coPProjectionDistanceSquared = coPProjection.distanceSquared(unconstrainedFeedbackCoP);
      //            double closestPointWithProjectionLineDistanceSquared = closestPointWithProjectionLine.distanceSquared(unconstrainedFeedbackCoP);

      //            if (coPProjectionDistanceSquared <= closestPointWithProjectionLineDistanceSquared)
      //               feedbackCoP.set(coPProjection);
      //            else
      //               feedbackCoP.set(closestPointWithProjectionLine);

   }

   private void projectIntoFoot()
   {
      // Project the CoP onto the support polygon, using a projection vector.
      coPProjection.setToNaN();
      firstProjectionIntersection.setToNaN();
      secondProjectionIntersection.setToNaN();
      closestPointWithProjectionLine.setToNaN();
      projectionVector.setToNaN();
      projectionLine.setToNaN();

      // Determine the closest point, intersection point, etc.
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      if (supportPolygonInWorld.isPointInside(unconstrainedFeedbackCoP))
      {
         //TODO: Move more inside the foot if there is a large enough distance from the CMP to the ICP, so that 
         // the foot doesn't unnecessarily rotate.
         // Maybe after all the stuff is computed, do this tweak inside to prevent foot rotation if you can.

         feedbackCoP.set(unconstrainedFeedbackCoP);
      }

      else
      {
         //Compute the projection vector and projection line:
         projectionVector.set(currentICP);
         projectionVector.sub(unconstrainedFeedbackCMP);

         //TODO: What if projection vector length is small?
         projectionVector.normalize();

         projectionLine.set(unconstrainedFeedbackCoP, projectionVector);

         supportPolygonInWorld.intersectionWith(projectionLine, firstProjectionIntersection, secondProjectionIntersection);
         if (!firstProjectionIntersection.containsNaN())
         {
            if (secondProjectionIntersection.containsNaN())
            {
               feedbackCoP.set(firstProjectionIntersection);
            }
            else
            {
               double firstDistanceSquared = firstProjectionIntersection.distanceSquared(unconstrainedFeedbackCoP);
               double secondDistanceSquared = secondProjectionIntersection.distanceSquared(unconstrainedFeedbackCoP);

               //TODO: Take somewhere inside the foot if there is a large enough distance from the CMP to the ICP after the projection, so that 
               // the foot doesn't unnecessarily rotate.
               if (firstDistanceSquared <= secondDistanceSquared)
               {
                  feedbackCoP.set(firstProjectionIntersection);
               }
               else
               {
                  feedbackCoP.set(secondProjectionIntersection);
               }
            }
         }
         else
         {
            coPProjection.set(unconstrainedFeedbackCoP);
            supportPolygonInWorld.orthogonalProjection(coPProjection);

            supportPolygonInWorld.getClosestPointWithRay(projectionLine, closestPointWithProjectionLine);

            tempVector.set(currentICP);
            tempVector.sub(coPProjection);
            tempVector.normalize();
            double copProjectionDot = tempVector.dot(projectionVector);

            tempVector.set(currentICP);
            tempVector.sub(closestPointWithProjectionLine);
            tempVector.normalize();
            double closestPointWithProjectionLineDot = tempVector.dot(projectionVector);

            // TODO: Not sure if this is what we want to be doing here. Might be something smarter.
            if (copProjectionDot >= closestPointWithProjectionLineDot)
               feedbackCoP.set(coPProjection);
            else
               feedbackCoP.set(closestPointWithProjectionLine);

            //            double coPProjectionDistanceSquared = coPProjection.distanceSquared(unconstrainedFeedbackCoP);
            //            double closestPointWithProjectionLineDistanceSquared = closestPointWithProjectionLine.distanceSquared(unconstrainedFeedbackCoP);

            //            if (coPProjectionDistanceSquared <= closestPointWithProjectionLineDistanceSquared)
            //               feedbackCoP.set(coPProjection);
            //            else
            //               feedbackCoP.set(closestPointWithProjectionLine);
         }
      }
   }

   private void limitAbsoluteValue(YoDouble yoDouble, double maxAbsoluteValue)
   {
      if (yoDouble.getValue() > maxAbsoluteValue)
         yoDouble.set(maxAbsoluteValue);
      else if (yoDouble.getValue() < -maxAbsoluteValue)
         yoDouble.set(-maxAbsoluteValue);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition feedbackCoPViz = new YoGraphicPosition(yoNamePrefix + "FeedbackCoP", this.feedbackCoP, 0.005, YoAppearance.Darkorange(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCMPViz = new YoGraphicPosition(yoNamePrefix + "UnconstrainedFeedbackCMP", this.unconstrainedFeedbackCMP, 0.008, Purple(), GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCoPViz = new YoGraphicPosition(yoNamePrefix
            + "UnconstrainedFeedbackCoP", this.unconstrainedFeedbackCoP, 0.004, YoAppearance.Green(), GraphicType.BALL_WITH_ROTATED_CROSS);

      //TODO: Figure out a viz that works with the logger.
      YoArtifactLine2d projectionLineViz = new YoArtifactLine2d(yoNamePrefix + "ProjectionLine", this.projectionLine, YoAppearance.Aqua().getAwtColor());

      YoGraphicPosition firstIntersectionViz = new YoGraphicPosition(yoNamePrefix + "FirstIntersection", this.firstProjectionIntersection, 0.004, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition secondIntersectionViz = new YoGraphicPosition(yoNamePrefix + "SecondIntersection", this.secondProjectionIntersection, 0.004, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition closestPointWithProjectionLineViz = new YoGraphicPosition(yoNamePrefix
            + "ClosestToProjectionLine", this.closestPointWithProjectionLine, 0.003, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition copProjectionViz = new YoGraphicPosition(yoNamePrefix + "CoPProjection", this.coPProjection, 0.002, YoAppearance.Green(), GraphicType.SOLID_BALL);

      artifactList.add(feedbackCoPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCMPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCoPViz.createArtifact());
      //      artifactList.add(projectionLineViz);

      artifactList.add(firstIntersectionViz.createArtifact());
      artifactList.add(secondIntersectionViz.createArtifact());
      artifactList.add(closestPointWithProjectionLineViz.createArtifact());
      artifactList.add(copProjectionViz.createArtifact());

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
