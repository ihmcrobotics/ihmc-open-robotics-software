package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.SaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.function.Supplier;

public class CoPTrajectoryGenerator extends SaveableModule<CoPTrajectoryGeneratorState>
{
   private final CoPTrajectoryParameters parameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble safeDistanceFromCoPToSupportEdgesWhenSteppingDown;
   private final YoDouble exitCoPForwardSafetyMarginOnToes;
   // FIXME this is kind of a state variable
   private final YoDouble percentageStandingWeightDistributionOnLeftFoot;

   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final SideDependentList<RecyclingArrayList<PoseReferenceFrame>> stepFrames = new SideDependentList<>();

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePose3D tempPose = new FramePose3D();

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint2D tempFramePoint2D = new FramePoint2D();
   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FramePoint2D tempPointForCoPCalculation = new FramePoint2D();
   private final FramePoint2D previousCoPPosition = new FramePoint2D();
   private final FramePoint2D midfootCoP = new FramePoint2D();

   private WaypointViewer viewer = null;

   public CoPTrajectoryGenerator(CoPTrajectoryParameters parameters,
                                 ConvexPolygon2DReadOnly defaultSupportPolygon,
                                 YoRegistry parentRegistry)
   {
      super(CoPTrajectoryGenerator.class, parentRegistry);

      this.parameters = parameters;
      this.defaultSupportPolygon.set(defaultSupportPolygon);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble("SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", parentRegistry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble("ExitCoPForwardSafetyMarginOnToes", parentRegistry);

      percentageStandingWeightDistributionOnLeftFoot = new YoDouble("PercentageStandingWeightDistributionOnLeftFoot", registry);
      percentageStandingWeightDistributionOnLeftFoot.set(0.5);

      for (RobotSide robotSide : RobotSide.values)
      {
         stepFrames.put(robotSide, new RecyclingArrayList<>(3, new Supplier<PoseReferenceFrame>()
         {
            private int frameIndexCounter = 0;

            @Override
            public PoseReferenceFrame get()
            {
               return new PoseReferenceFrame(robotSide.getLowerCaseName() + "StepFrame" + frameIndexCounter++, ReferenceFrame.getWorldFrame());
            }
         }));
      }

      parentRegistry.addChild(registry);
      clear();
   }

   public void setWaypointViewer(WaypointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   private void reset()
   {
      contactStateProviders.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         stepFrames.get(robotSide).clear();
      }
   }

   public void set(Point2DReadOnly constantCop)
   {
      clear();

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.getTimeInterval().setInterval(0.0, Double.POSITIVE_INFINITY);
      contactState.setStartCopPosition(constantCop);
      contactState.setEndCopPosition(constantCop);
   }

   private final FrameConvexPolygon2DBasics nextPolygon = new FrameConvexPolygon2D();

   public void compute(CoPTrajectoryGeneratorState state)
   {
      int numberOfUpcomingFootsteps = Math.min(parameters.getNumberOfStepsToConsider(), state.getNumberOfFootstep());

      reset();
      // Add initial support states of the feet and set the moving polygons
      for (RobotSide robotSide : RobotSide.values)
      {
         // Record the initial step frames. In case there is a step touching down this frame will be updated.
         PoseReferenceFrame stepFrame = stepFrames.get(robotSide).add();
         tempPose.setIncludingFrame(state.getFootPose(robotSide));
         tempPose.changeFrame(stepFrame.getParent());
         stepFrame.setPoseAndUpdate(tempPose);

         movingPolygonsInSole.get(robotSide).setIncludingFrame(state.getFootPolygonInSole(robotSide));
      }

      // compute cop waypoint location
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartTime(0.0);
      contactStateProvider.setStartCopPosition(state.getInitialCoP());

      // Put first CoP as per chicken support computations in case starting from rest
      if (numberOfUpcomingFootsteps == 0)
      {
         // just standing there
         computeCoPPointsForStanding(RobotSide.LEFT,
                                     state.getFinalTransferDuration(),
                                     state.getFinalTransferSplitFraction(),
                                     percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());
      }
      else
      {
         // Compute all the upcoming waypoints
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            PlanningFootstep footstep = state.getFootstep(footstepIndex);
            PlanningTiming timings = state.getTiming(footstepIndex);
            PlanningShiftFraction shiftFraction = state.getShiftFraction(footstepIndex);
            RobotSide swingSide = footstep.getRobotSide();
            RobotSide supportSide = swingSide.getOppositeSide();

            FrameConvexPolygon2DReadOnly previousPolygon = movingPolygonsInSole.get(swingSide);
            FrameConvexPolygon2DReadOnly currentPolygon = movingPolygonsInSole.get(swingSide.getOppositeSide());

            ReferenceFrame stepFrame = extractStepFrame(footstep);
            extractSupportPolygon(footstep, stepFrame, nextPolygon, defaultSupportPolygon);

            computeCoPPointsForFootstepTransfer(timings.getTransferTime(),
                                                shiftFraction.getTransferSplitFraction(),
                                                shiftFraction.getTransferWeightDistribution(),
                                                previousPolygon,
                                                currentPolygon,
                                                supportSide);
            computeCoPPointsForFootstepSwing(Math.min(timings.getSwingTime(), SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE),
                                             shiftFraction.getSwingDurationShiftFraction(),
                                             shiftFraction.getSwingSplitFraction(),
                                             currentPolygon,
                                             nextPolygon,
                                             supportSide);

            movingPolygonsInSole.get(swingSide).setIncludingFrame(nextPolygon);
         }
         computeCoPPointsForFinalTransfer(state.getFootstep(numberOfUpcomingFootsteps - 1).getRobotSide(),
                                          state.getFinalTransferDuration(),
                                          state.getFinalTransferSplitFraction(),
                                          state.getFinalTransferWeightDistribution());

         RobotSide lastStepSide = state.getFootstep(numberOfUpcomingFootsteps - 1).getRobotSide().getOppositeSide();
         if (lastStepSide == RobotSide.RIGHT)
            percentageStandingWeightDistributionOnLeftFoot.set(1.0 - state.getFinalTransferWeightDistribution());
         else
            percentageStandingWeightDistributionOnLeftFoot.set(state.getFinalTransferWeightDistribution());
      }

      if (viewer != null)
         viewer.updateWaypoints(contactStateProviders);
   }


   public void update(double time, FixedFramePoint3DBasics desiredCoP)
   {
      for (int i = 0; i < contactStateProviders.size(); i++)
      {
         ContactStateProvider contactStateProvider = contactStateProviders.get(i);
         if (contactStateProvider.getTimeInterval().intervalContains(time))
         {
            double alpha = (time - contactStateProvider.getTimeInterval().getStartTime()) / contactStateProvider.getTimeInterval().getDuration();
            desiredCoP.interpolate(contactStateProvider.getCopStartPosition(), contactStateProvider.getCopEndPosition(), alpha);
            break;
         }
         else
         {
            continue;
         }
      }
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   private ReferenceFrame extractStepFrame(PlanningFootstep footstep)
   {
      PoseReferenceFrame stepFrame = stepFrames.get(footstep.getRobotSide()).add();
      stepFrame.setPoseAndUpdate(footstep.getFootstepPose());

      return stepFrame;
   }

   private void extractSupportPolygon(PlanningFootstep footstep,
                                      ReferenceFrame stepFrame,
                                      FrameConvexPolygon2DBasics footSupportPolygonToPack,
                                      ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      if (footstep.hasPredictedContactPoints())
      {
         List<? extends Point2DReadOnly> predictedContactPoints = footstep.getPredictedContactPoints();

         footSupportPolygonToPack.clear(stepFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            footSupportPolygonToPack.addVertex(predictedContactPoints.get(i));
         footSupportPolygonToPack.update();
      }
      else
      {
         footSupportPolygonToPack.setIncludingFrame(stepFrame, defaultSupportPolygon);
      }
   }


   private void computeCoPPointsForStanding(RobotSide lastFootstepSide,
                                           double finalTransferDuration,
                                           double finalTransferSplitFraction,
                                           double finalTransferWeightDistribution)
   {
      tempPointForCoPCalculation.setIncludingFrame(movingPolygonsInSole.get(lastFootstepSide).getCentroid());
      tempPointForCoPCalculation.changeFrameAndProjectToXYPlane(worldFrame);
      tempFramePoint2D.setIncludingFrame(movingPolygonsInSole.get(lastFootstepSide.getOppositeSide()).getCentroid());
      tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);
      tempPointForCoPCalculation.interpolate(tempFramePoint2D, finalTransferWeightDistribution);

      double segmentDuration = finalTransferSplitFraction * finalTransferDuration;
      SettableContactStateProvider contactState = contactStateProviders.getLast();
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);

      segmentDuration = (1.0 - finalTransferSplitFraction) * finalTransferDuration;
      SettableContactStateProvider previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(Double.POSITIVE_INFINITY);
   }

   private void computeCoPPointsForFinalTransfer(RobotSide lastFootstepSide,
                                                 double finalTransferDuration,
                                                 double finalTransferSplitFraction,
                                                 double finalTransferWeightDistribution)
   {
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();

      tempPointForCoPCalculation.setIncludingFrame(movingPolygonsInSole.get(lastFootstepSide).getCentroid());
      tempPointForCoPCalculation.changeFrameAndProjectToXYPlane(worldFrame);
      tempFramePoint2D.setIncludingFrame(movingPolygonsInSole.get(lastFootstepSide.getOppositeSide()).getCentroid());
      tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);
      tempPointForCoPCalculation.interpolate(tempFramePoint2D, finalTransferWeightDistribution);

      double segmentDuration = finalTransferSplitFraction * finalTransferDuration;
      previousContactState.setEndCopPosition(tempPointForCoPCalculation);
      previousContactState.setDuration(segmentDuration);

      segmentDuration = (1.0 - finalTransferSplitFraction) * finalTransferDuration;
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);
   }

   private void computeCoPPointsForFootstepTransfer(double duration,
                                                    double splitFraction,
                                                    double weightDistribution,
                                                    FrameConvexPolygon2DReadOnly previousPolygon,
                                                    FrameConvexPolygon2DReadOnly nextPolygon,
                                                    RobotSide supportSide)
   {
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      previousCoPPosition.setIncludingFrame(previousContactState.getCopStartPosition());

      computeEntryCoPPointLocation(tempPointForCoPCalculation, previousPolygon, nextPolygon, supportSide);
      midfootCoP.interpolate(previousCoPPosition, tempPointForCoPCalculation, weightDistribution);

      previousContactState.setDuration(splitFraction * duration);
      previousContactState.setEndCopPosition(midfootCoP);

      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setDuration((1.0 - splitFraction) * duration);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
   }

   private void computeCoPPointsForFootstepSwing(double duration,
                                                 double shiftFraction,
                                                 double splitFraction,
                                                 FrameConvexPolygon2DReadOnly supportPolygon,
                                                 FrameConvexPolygon2DReadOnly nextPolygon,
                                                 RobotSide supportSide)
   {
      computeBallCoPLocation(tempPointForCoPCalculation, supportPolygon, nextPolygon, supportSide);
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setDuration(shiftFraction * splitFraction * duration);
      contactState.setEndCopPosition(tempPointForCoPCalculation);

      previousContactState = contactState;

      computeExitCoPLocation(tempPointForCoPCalculation, supportPolygon, nextPolygon, supportSide);

      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setDuration(shiftFraction * (1.0 - splitFraction) * duration);
      contactState.setEndCopPosition(tempPointForCoPCalculation);

      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setDuration((1.0 - shiftFraction) * duration);
      contactState.setEndCopPosition(tempPointForCoPCalculation);

      contactStateProviders.add().setStartFromEnd(contactState);
   }

   private void computeEntryCoPPointLocation(FramePoint2DBasics copLocationToPack,
                                             FrameConvexPolygon2DReadOnly previousFootPolygon,
                                             FrameConvexPolygon2DReadOnly footPolygon,
                                             RobotSide supportSide)
   {
      computeCoPLocation(copLocationToPack,
                         parameters.getEntryCMPLengthOffsetFactor(),
                         parameters.getEntryCMPOffset(),
                         parameters.getEntryCMPMinX(),
                         parameters.getEntryCMPMaxX(),
                         footPolygon,
                         previousFootPolygon,
                         supportSide);
   }

   private void computeBallCoPLocation(FramePoint2DBasics copLocationToPack,
                                       FrameConvexPolygon2DReadOnly footPolygon,
                                       FrameConvexPolygon2DReadOnly nextFootPolygon,
                                       RobotSide supportSide)
   {
      computeCoPLocation(copLocationToPack,
                         parameters.getBallCMPLengthOffsetFactor(),
                         parameters.getBallCMPOffset(),
                         parameters.getBallCMPMinX(),
                         parameters.getBallCMPMaxX(),
                         footPolygon,
                         nextFootPolygon,
                         supportSide);
   }

   private void computeExitCoPLocation(FramePoint2DBasics copLocationToPack,
                                       FrameConvexPolygon2DReadOnly footPolygon,
                                       FrameConvexPolygon2DReadOnly nextFootPolygon,
                                       RobotSide supportSide)
   {
      if (setExitCoPUnderSpecialCases(copLocationToPack, footPolygon, nextFootPolygon, supportSide))
         return;

      computeCoPLocation(copLocationToPack,
                         parameters.getExitCMPLengthOffsetFactor(),
                         parameters.getExitCMPOffset(),
                         parameters.getExitCMPMinX(),
                         parameters.getExitCMPMaxX(),
                         footPolygon,
                         nextFootPolygon,
                         supportSide);
   }

   private void computeCoPLocation(FramePoint2DBasics copLocationToPack,
                                   double lengthOffsetFactor,
                                   Vector2DReadOnly copOffset,
                                   double minXOffset,
                                   double maxXOffset,
                                   FrameConvexPolygon2DReadOnly basePolygon,
                                   FrameConvexPolygon2DReadOnly otherPolygon,
                                   RobotSide supportSide)
   {
      copLocationToPack.setIncludingFrame(basePolygon.getCentroid());

      double copXOffset = MathTools.clamp(copOffset.getX() + lengthOffsetFactor * getStepLength(otherPolygon, basePolygon),
                                          minXOffset,
                                          maxXOffset);
      copLocationToPack.add(copXOffset, supportSide.negateIfLeftSide(copOffset.getY()));

      constrainToPolygon(copLocationToPack, basePolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrameAndProjectToXYPlane(worldFrame);
   }

   /**
    * Checks for the following conditions to have been the case:
    * - the support polygon is empty
    * - the exit CoP goes in the toes
    * - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setExitCoPUnderSpecialCases(FramePoint2DBasics framePointToPack,
                                               FrameConvexPolygon2DReadOnly supportFootPolygon,
                                               FrameConvexPolygon2DReadOnly upcomingSwingFootPolygon,
                                               RobotSide supportSide)
   {
      tempFramePoint1.setIncludingFrame(upcomingSwingFootPolygon.getCentroid(), 0.0);
      tempFramePoint1.changeFrame(supportFootPolygon.getReferenceFrame());
      tempFramePoint2.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
      double supportToSwingStepLength = tempFramePoint1.getX() - tempFramePoint2.getX();
      double supportToSwingStepHeight = tempFramePoint1.getZ() - tempFramePoint2.getZ();
      if (supportFootPolygon.getArea() == 0.0)
      { // FIXME this is bad if it's a line, right?
         framePointToPack.setIncludingFrame(supportFootPolygon.getReferenceFrame(), supportFootPolygon.getVertex(0));
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else if (parameters.getPlanForToeOffCalculator().shouldPutCMPOnToes(supportToSwingStepLength, supportToSwingStepHeight))
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid());
         framePointToPack.add(supportFootPolygon.getMaxX() - exitCoPForwardSafetyMarginOnToes.getDoubleValue(),
                              supportSide.negateIfRightSide(parameters.getExitCMPOffset().getY()));
         constrainToPolygon(framePointToPack, supportFootPolygon, safeDistanceFromCoPToSupportEdgesWhenSteppingDown.getDoubleValue());
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else
      {
         return false;
      }
   }

   private final FramePoint2D mostForwardPointOnOtherPolygon = new FramePoint2D();

   private double getStepLength(FrameConvexPolygon2DReadOnly otherPolygon, FrameConvexPolygon2DReadOnly basePolygon)
   {
      mostForwardPointOnOtherPolygon.setIncludingFrame(otherPolygon.getVertex(EuclidGeometryPolygonTools.findVertexIndex(otherPolygon,
                                                                                                                         true,
                                                                                                                         Bound.MAX,
                                                                                                                         Bound.MAX)));
      mostForwardPointOnOtherPolygon.changeFrameAndProjectToXYPlane(basePolygon.getReferenceFrame());

     return mostForwardPointOnOtherPolygon.getX() - basePolygon.getMaxX();
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by
    * projection
    */
   private void constrainToPolygon(FramePoint2DBasics copPointToConstrain,
                                   FrameConvexPolygon2DReadOnly constraintPolygon,
                                   double safeDistanceFromSupportPolygonEdges)
   {
      // don't need to do anything if it's already inside
      if (constraintPolygon.signedDistance(copPointToConstrain) <= -safeDistanceFromSupportPolygonEdges)
         return;

      polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      copPointToConstrain.changeFrame(constraintPolygon.getReferenceFrame());
      tempPolygon.orthogonalProjection(copPointToConstrain);
   }
}