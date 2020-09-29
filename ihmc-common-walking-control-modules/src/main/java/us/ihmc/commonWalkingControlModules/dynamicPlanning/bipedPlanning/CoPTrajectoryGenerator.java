package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
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
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;
import java.util.function.Supplier;

public class CoPTrajectoryGenerator
{
   private final CoPTrajectoryParameters parameters;

   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble safeDistanceFromCoPToSupportEdgesWhenSteppingDown;
   private final YoDouble exitCoPForwardSafetyMarginOnToes;
   private final YoDouble percentageStandingWeightDistributionOnLeftFoot;

   // State variables
   private final SideDependentList<FrameConvexPolygon2D> footPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());
   private final SideDependentList<FramePose3D> footPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final SideDependentList<? extends ReferenceFrame> soleFrames;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames;
   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final SideDependentList<RecyclingArrayList<PoseReferenceFrame>> stepFrames = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberFootstepsToConsider;
   private final YoDouble finalTransferWeightDistribution;

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePose3D tempPose = new FramePose3D();

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FramePoint2D tempPointForCoPCalculation = new FramePoint2D();
   private final FramePoint2D previousCoPPosition = new FramePoint2D();
   private final FramePoint2D midfootCoP = new FramePoint2D();

   public CoPTrajectoryGenerator(String namePrefix,
                                 CoPTrajectoryParameters parameters,
                                 BipedSupportPolygons bipedSupportPolygons,
                                 ConvexPolygon2DReadOnly defaultSupportPolygon,
                                 YoInteger numberFootstepsToConsider,
                                 SideDependentList<? extends ReferenceFrame> soleFrames,
                                 SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                 YoRegistry parentRegistry)
   {
      this(namePrefix,
           parameters,
           bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(),
           defaultSupportPolygon,
           numberFootstepsToConsider,
           soleFrames,
           soleZUpFrames,
           parentRegistry);
   }

   public CoPTrajectoryGenerator(String namePrefix,
                                 CoPTrajectoryParameters parameters,
                                 SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames,
                                 ConvexPolygon2DReadOnly defaultSupportPolygon,
                                 YoInteger numberFootstepsToConsider,
                                 SideDependentList<? extends ReferenceFrame> soleFrames,
                                 SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                 YoRegistry parentRegistry)
   {
      this.numberFootstepsToConsider = numberFootstepsToConsider;
      this.feetInSoleZUpFrames = feetInSoleZUpFrames;
      this.parameters = parameters;
      this.defaultSupportPolygon.set(defaultSupportPolygon);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      String fullPrefix = namePrefix + "CoPTrajectoryGenerator";
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble(fullPrefix + "SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", parentRegistry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble(fullPrefix + "ExitCoPForwardSafetyMarginOnToes", parentRegistry);

      percentageStandingWeightDistributionOnLeftFoot = new YoDouble(namePrefix + "PercentageStandingWeightDistributionOnLeftFoot", registry);

      finalTransferWeightDistribution = new YoDouble("finalTransferWeightDistribution", registry);
      finalTransferWeightDistribution.setToNaN();

      this.soleFrames = soleFrames;
      this.soleZUpFrames = soleZUpFrames;

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

   /**
    * For the given side updates the current foot pose and the foothold of the robot to match the actual sole pose and
    * foothold.
    *
    * @param robotSide to be updated.
    */
   public void initializeStance(RobotSide robotSide)
   {
      if (feetInSoleZUpFrames == null)
         footPolygonsInSole.get(robotSide).setIncludingFrame(soleFrames.get(robotSide), defaultSupportPolygon);
      else
         footPolygonsInSole.get(robotSide).setIncludingFrame(feetInSoleZUpFrames.get(robotSide));
      footPoses.get(robotSide).setToZero(soleFrames.get(robotSide));
      footPoses.get(robotSide).changeFrame(soleZUpFrames.get(robotSide));
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

   public void update(List<Footstep> footsteps,
                      List<FootstepTiming> footstepTimings,
                      List<FootstepShiftFractions> footstepShiftFractions,
                      Point2DReadOnly initialCoP)
   {
      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), footsteps.size());

      reset();
      // Add initial support states of the feet and set the moving polygons
      for (RobotSide robotSide : RobotSide.values)
      {
         // Record the initial step frames. In case there is a step touching down this frame will be updated.
         PoseReferenceFrame stepFrame = stepFrames.get(robotSide).add();
         tempPose.setIncludingFrame(footPoses.get(robotSide));
         tempPose.changeFrame(stepFrame.getParent());
         stepFrame.setPoseAndUpdate(tempPose);

         movingPolygonsInSole.get(robotSide).setIncludingFrame(soleFrames.get(robotSide), footPolygonsInSole.get(robotSide));
      }

      // compute cop waypoint location
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartTime(0.0);
      contactStateProvider.setStartCopPosition(initialCoP);

      // Put first CoP as per chicken support computations in case starting from rest
      if (numberOfUpcomingFootsteps == 0)
      {
         // just standing there
         computeCoPPointsForStanding(footstepTimings, footstepShiftFractions);
      }
      else
      {
         // Compute all the upcoming waypoints
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            Footstep footstep = footsteps.get(footstepIndex);
            RobotSide swingSide = footstep.getRobotSide();
            RobotSide supportSide = swingSide.getOppositeSide();

            FrameConvexPolygon2DReadOnly previousPolygon = movingPolygonsInSole.get(swingSide);
            FrameConvexPolygon2DReadOnly currentPolygon = movingPolygonsInSole.get(swingSide.getOppositeSide());

            ReferenceFrame stepFrame = extractStepFrame(footstep);
            extractSupportPolygon(footstep, stepFrame, nextPolygon, defaultSupportPolygon);

            computeCoPPointsForFootstepTransfer(footstepTimings.get(footstepIndex).getTransferTime(),
                                                footstepShiftFractions.get(footstepIndex).getTransferSplitFraction(),
                                                footstepShiftFractions.get(footstepIndex).getTransferWeightDistribution(),
                                                previousPolygon,
                                                currentPolygon,
                                                supportSide);
            computeCoPPointsForFootstepSwing(Math.min(footstepTimings.get(footstepIndex).getSwingTime(), SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE),
                                             footstepShiftFractions.get(footstepIndex).getSwingDurationShiftFraction(),
                                             footstepShiftFractions.get(footstepIndex).getSwingSplitFraction(),
                                             currentPolygon,
                                             nextPolygon,
                                             supportSide);

            movingPolygonsInSole.get(swingSide).setIncludingFrame(nextPolygon);
         }
         computeCoPPointsForFinalTransfer(numberOfUpcomingFootsteps, footsteps.get(footsteps.size() - 1), footstepTimings, footstepShiftFractions);

         RobotSide lastStepSide = footsteps.get(numberOfUpcomingFootsteps - 1).getRobotSide().getOppositeSide();
         if (lastStepSide == RobotSide.RIGHT)
            percentageStandingWeightDistributionOnLeftFoot.set(1.0 - finalTransferWeightDistribution.getValue());
         else
            percentageStandingWeightDistributionOnLeftFoot.set(finalTransferWeightDistribution.getValue());
      }
   }

   private ReferenceFrame extractStepFrame(Footstep footstep)
   {
      PoseReferenceFrame stepFrame = stepFrames.get(footstep.getRobotSide()).add();
      stepFrame.setPoseAndUpdate(footstep.getFootstepPose());

      return stepFrame;
   }

   private void extractSupportPolygon(Footstep footstep,
                                      ReferenceFrame stepFrame,
                                      FrameConvexPolygon2DBasics footSupportPolygonToPack,
                                      ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      if (footstep.hasPredictedContactPoints())
      {
         List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();

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

   private void computeCoPPointsForStanding(List<FootstepTiming> footstepTimings, List<FootstepShiftFractions> footstepShiftFractions)
   {
      SettableContactStateProvider contactStateProvider = contactStateProviders.getLast();

      tempPointForCoPCalculation.interpolate(feetInSoleZUpFrames.get(RobotSide.LEFT).getCentroid(),
                                             feetInSoleZUpFrames.get(RobotSide.RIGHT).getCentroid(),
                                             percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      double transferDuration = footstepTimings.get(0).getTransferTime();
      double splitFraction = footstepShiftFractions.get(0).getTransferSplitFraction();
      double segmentDuration = splitFraction * transferDuration;
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setDuration(segmentDuration);

      SettableContactStateProvider previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);

      segmentDuration = (1.0 - splitFraction) * transferDuration;
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setDuration(segmentDuration);
   }

   private void computeCoPPointsForFinalTransfer(int footstepIndex,
                                                 Footstep lastFootstep,
                                                 List<FootstepTiming> footstepTimings,
                                                 List<FootstepShiftFractions> footstepShiftFractions)
   {
      ContactStateProvider previousContactState = contactStateProviders.getLast();

      tempPointForCoPCalculation.interpolate(movingPolygonsInSole.get(lastFootstep.getRobotSide()).getCentroid(),
                                             movingPolygonsInSole.get(lastFootstep.getRobotSide().getOppositeSide()).getCentroid(),
                                             finalTransferWeightDistribution.getValue());

      double transferDuration = footstepTimings.get(footstepIndex).getTransferTime();
      double splitFraction = footstepShiftFractions.get(footstepIndex).getTransferSplitFraction();
      double segmentDuration = splitFraction * transferDuration;
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);

      segmentDuration = (1.0 - splitFraction) * transferDuration;
      previousContactState = contactState;
      contactState = contactStateProviders.add();
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
      previousCoPPosition.setIncludingFrame(previousContactState.getCopEndPosition());

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
      SettableContactStateProvider contactState = contactStateProviders.getLast();
      contactState.setDuration(shiftFraction * splitFraction * duration);
      contactState.setEndCopPosition(tempPointForCoPCalculation);

      SettableContactStateProvider previousContactState = contactState;

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

      double copXOffset = MathTools.clamp(copOffset.getX() + getStepLengthToCoPOffset(lengthOffsetFactor, otherPolygon, basePolygon),
                                          minXOffset,
                                          maxXOffset);
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()));

      constrainToPolygon(copLocationToPack, basePolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
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

   private double getStepLengthToCoPOffset(double lengthOffsetFactor, FrameConvexPolygon2DReadOnly otherPolygon, FrameConvexPolygon2DReadOnly basePolygon)
   {
      mostForwardPointOnOtherPolygon.setIncludingFrame(otherPolygon.getVertex(EuclidGeometryPolygonTools.findVertexIndex(otherPolygon,
                                                                                                                         true,
                                                                                                                         Bound.MAX,
                                                                                                                         Bound.MAX)));
      mostForwardPointOnOtherPolygon.changeFrameAndProjectToXYPlane(basePolygon.getReferenceFrame());

      double length = mostForwardPointOnOtherPolygon.getX() - basePolygon.getMaxX();
      return lengthOffsetFactor * length;
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