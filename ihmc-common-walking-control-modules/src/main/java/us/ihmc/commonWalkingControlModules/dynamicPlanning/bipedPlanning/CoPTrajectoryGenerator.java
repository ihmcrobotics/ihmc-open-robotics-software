package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
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
   private final YoDouble additionalTimeForFinalTransfer;

   // State variables
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2DReadOnly> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2DReadOnly> defaultFootPolygons = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberFootstepsToConsider;
   private final IntegerProvider numberOfUpcomingFootsteps;

   private final YoDouble finalTransferWeightDistribution;

   private final YoBoolean holdDesiredState;

   // Runtime variables
   private final FramePoint3D heldCoPPosition = new FramePoint3D();
   private final PoseReferenceFrame footFrameAtStartOfSwing = new PoseReferenceFrame("footFrameAtStartOfSwing", worldFrame);
   private final ConvexPolygon2D footPolygonAtStartOfSwing = new ConvexPolygon2D();

   private final RecyclingArrayList<FrameConvexPolygon2D> transferringFromPolygon = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2D> transferringToPolygon = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2D> upcomingPolygon = new RecyclingArrayList<>(FrameConvexPolygon2D::new);

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   // Temp variables for computation only
   private final ConvexPolygon2D polygonReference = new ConvexPolygon2D();
   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FramePoint2D tempFramePoint2d = new FramePoint2D();
   private final FramePoint3D tempPointForCoPCalculation = new FramePoint3D();
   private final FramePoint3D previousCoPLocation = new FramePoint3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public CoPTrajectoryGenerator(String namePrefix, CoPTrajectoryParameters parameters,  BipedSupportPolygons bipedSupportPolygons,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                 IntegerProvider numberOfUpcomingFootsteps,SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                 YoRegistry parentRegistry)
   {
      this(namePrefix, parameters, bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), contactableFeet, numberFootstepsToConsider,
           numberOfUpcomingFootsteps, soleZUpFrames, parentRegistry);

   }

   public CoPTrajectoryGenerator(String namePrefix,  CoPTrajectoryParameters parameters, SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                 IntegerProvider numberOfUpcomingFootsteps,
                                 SideDependentList<? extends ReferenceFrame> soleZUpFrames, YoRegistry parentRegistry)
   {
      this.numberFootstepsToConsider = numberFootstepsToConsider;
      this.parameters = parameters;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      String fullPrefix = namePrefix + "CoPTrajectoryGenerator";
      additionalTimeForFinalTransfer = new YoDouble(fullPrefix + "AdditionalTimeForFinalTransfer", registry);
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble(fullPrefix + "SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", parentRegistry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble(fullPrefix + "ExitCoPForwardSafetyMarginOnToes", parentRegistry);

      percentageStandingWeightDistributionOnLeftFoot = new YoDouble(namePrefix + "PercentageStandingWeightDistributionOnLeftFoot", registry);

      this.numberOfUpcomingFootsteps = numberOfUpcomingFootsteps;

      finalTransferWeightDistribution = new YoDouble("finalTransferWeightDistribution", registry);
      finalTransferWeightDistribution.setToNaN();

      this.holdDesiredState = new YoBoolean(fullPrefix + "HoldDesiredState", parentRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactableFeet.get(robotSide).getContactPoints2d()));
         defaultFootPolygons.put(robotSide, defaultFootPolygon);

         supportFootPolygonsInSoleZUpFrames.put(robotSide, feetInSoleZUpFrames.get(robotSide));
      }

      this.soleZUpFrames = soleZUpFrames;

      parentRegistry.addChild(registry);
      clear();
   }

   public void holdPosition(FramePoint3DReadOnly desiredCoPPositionToHold)
   {
      holdDesiredState.set(true);
      heldCoPPosition.setIncludingFrame(desiredCoPPositionToHold);
   }

   private void clearHeldPosition()
   {
      holdDesiredState.set(false);
      heldCoPPosition.setToNaN(worldFrame);
   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   public int getNumberOfFootstepsRegistered()
   {
      return numberOfUpcomingFootsteps.getValue();
   }

   public void initializeForSwing(List<Footstep> footsteps)
   {
      RobotSide swingSide = footsteps.get(0).getRobotSide();
      if (!supportFootPolygonsInSoleZUpFrames.get(swingSide).isEmpty() && !(supportFootPolygonsInSoleZUpFrames.get(swingSide).getNumberOfVertices() < 3))
         footPolygonAtStartOfSwing.set(supportFootPolygonsInSoleZUpFrames.get(swingSide));
      else
         footPolygonAtStartOfSwing.set(defaultFootPolygons.get(swingSide));

      ReferenceFrame swingFootFrame = soleZUpFrames.get(swingSide);
      swingFootFrame.getTransformToDesiredFrame(tempTransform, worldFrame);
      footFrameAtStartOfSwing.setPoseAndUpdate(tempTransform);
   }

   /**
    * Remember this in case the plan is cleared but the planner was doing chicken support. In that
    * case the ICP should be offset towards the correct foot.
    */
   private RobotSide lastTransferToSide = RobotSide.LEFT;

   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide, RobotSide previousTransferToSide,
                                                             List<Footstep> footsteps,
                                                             List<FootstepTiming> footstepTimings,
                                                             List<FootstepShiftFractions> footstepShiftFractions)
   {
      contactStateProviders.clear();

      boolean transferringToSameSideAsStartingFrom = previousTransferToSide != null && previousTransferToSide.equals(transferToSide);
      lastTransferToSide = previousTransferToSide == null ? transferToSide.getOppositeSide() : previousTransferToSide;
      initializeAllFootPolygons(transferToSide, transferringToSameSideAsStartingFrom, false, footsteps);

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      // Put first CoP as per chicken support computations in case starting from rest
      if (atAStop && (holdDesiredState.getBooleanValue() || numberOfUpcomingFootsteps == 0))
      {
         if (holdDesiredState.getBooleanValue())
         {
            computeCoPPointsForHoldingPosition(footstepTimings, footstepShiftFractions);
         }
         else
         { // just standing there
            computeCoPPointsForStanding(footstepTimings, footstepShiftFractions);
         }
      }
      else
      {
         SettableContactStateProvider contactStateProvider = contactStateProviders.add();

         if (atAStop)
         {  // this guy is starting a series of steps from standing
            double fraction = transferToSide == RobotSide.LEFT ? percentageStandingWeightDistributionOnLeftFoot.getValue() : 1.0 - percentageStandingWeightDistributionOnLeftFoot.getValue();
            computeMidFeetPointByPositionFraction(previousCoPLocation, transferringFromPolygon.getFirst(), transferringToPolygon.getFirst(), fraction);
         }
         else
         {  // starting while currently executing a step cycle
            clearHeldPosition();

            // Put first CoP at the exitCoP of the swing foot when starting in motion
            computeExitCoPPointLocationForPreviousPlan(previousCoPLocation, transferToSide.getOppositeSide(), transferringToSameSideAsStartingFrom);
         }

         contactStateProvider.setStartTime(0.0);
         contactStateProvider.setStartCopPosition(previousCoPLocation);

         // compute all the upcoming footsteps
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            RobotSide supportSide = footsteps.get(footstepIndex).getRobotSide().getOppositeSide();
            FrameConvexPolygon2DReadOnly previousPolygon = transferringFromPolygon.get(footstepIndex);
            FrameConvexPolygon2DReadOnly currentPolygon = transferringToPolygon.get(footstepIndex);
            FrameConvexPolygon2DReadOnly nextPolygon = transferringToPolygon.get(footstepIndex + 1);

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
         }
         computeCoPPointsForFinalTransfer(numberOfUpcomingFootsteps, atAStop, footsteps, footstepTimings, footstepShiftFractions);
      }
   }

   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide,
                                                             List<Footstep> footsteps,
                                                             List<FootstepTiming> footstepTimings,
                                                             List<FootstepShiftFractions> footstepShiftFractions)
   {
      clearHeldPosition();

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      if (numberOfUpcomingFootsteps == 0)
      {
         return;
      }

      initializeAllFootPolygons(null, false, true, footsteps);

      // compute cop waypoint location
      computeExitCoPPointLocationForPreviousPlan(previousCoPLocation, supportSide.getOppositeSide(), false);
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartTime(0.0);
      contactStateProvider.setStartCopPosition(previousCoPLocation);

      if (footstepTimings.get(0).getSwingTime() == Double.POSITIVE_INFINITY)
      { // We're in flamingo support, so only do the current one
         RobotSide currentSupportSide = footsteps.get(0).getRobotSide().getOppositeSide();
         FrameConvexPolygon2DReadOnly previousPolygon = transferringFromPolygon.get(0);
         FrameConvexPolygon2DReadOnly currentPolygon = transferringToPolygon.get(0);
         FrameConvexPolygon2DReadOnly nextPolygon = transferringToPolygon.get(1);

         computeCoPPointsForFootstepTransfer(footstepTimings.get(0).getTransferTime(),
                                             footstepShiftFractions.get(0).getTransferSplitFraction(),
                                             footstepShiftFractions.get(0).getTransferWeightDistribution(),
                                             previousPolygon,
                                             currentPolygon,
                                             currentSupportSide);
         double splitFraction = footstepShiftFractions.get(0).getSwingDurationShiftFraction() * footstepShiftFractions.get(0).getSwingSplitFraction();
         computeCoPPointsForFlamingoSingleSupport(Math.min(footstepTimings.get(0).getSwingTime(), SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE),
                                                  splitFraction,
                                                  currentPolygon,
                                                  nextPolygon,
                                                  currentSupportSide);
      }
      else
      { // Compute all the upcoming waypoints
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            RobotSide currentSupportSide = footsteps.get(0).getRobotSide().getOppositeSide();
            FrameConvexPolygon2DReadOnly previousPolygon = transferringFromPolygon.get(footstepIndex);
            FrameConvexPolygon2DReadOnly currentPolygon = transferringToPolygon.get(footstepIndex);
            FrameConvexPolygon2DReadOnly nextPolygon = transferringToPolygon.get(footstepIndex + 1);

            computeCoPPointsForFootstepTransfer(footstepTimings.get(footstepIndex).getTransferTime(),
                                                footstepShiftFractions.get(footstepIndex).getTransferSplitFraction(),
                                                footstepShiftFractions.get(footstepIndex).getTransferWeightDistribution(),
                                                previousPolygon,
                                                currentPolygon,
                                                currentSupportSide);
            computeCoPPointsForFootstepSwing(Math.min(footstepTimings.get(footstepIndex).getSwingTime(), SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE),
                                             footstepShiftFractions.get(footstepIndex).getSwingDurationShiftFraction(),
                                             footstepShiftFractions.get(footstepIndex).getSwingSplitFraction(),
                                             currentPolygon,
                                             nextPolygon,
                                             currentSupportSide);
         }
         computeCoPPointsForFinalTransfer(numberOfUpcomingFootsteps, false, footsteps, footstepTimings, footstepShiftFractions);
      }
   }

   // do not use these temporary variables anywhere except this method to avoid modifying them in unwanted places.
   private final FramePoint3D fractionTempMidPoint = new FramePoint3D();
   private final FramePoint3D fractionTempPoint1 = new FramePoint3D();
   private final FramePoint3D fractionTempPoint2 = new FramePoint3D();

   private void computeMidFeetPointByPositionFraction(FramePoint3DBasics framePointToPack,
                                                      FrameConvexPolygon2DReadOnly footPolygonA,
                                                      FrameConvexPolygon2DReadOnly footPolygonB,
                                                      double fraction)
   {
      // FIXME this method shouldn't be necessary
      getDoubleSupportPolygonCentroid(fractionTempMidPoint, footPolygonA, footPolygonB);

      fractionTempPoint1.setMatchingFrame(footPolygonA.getCentroid(), 0.0);
      fractionTempPoint2.setMatchingFrame(footPolygonB.getCentroid(), 0.0);

      framePointToPack.setToZero(worldFrame);

      // fixme we really want to interpolate from the exit cop to the next entry cop
      fraction = MathTools.clamp(fraction, 0.0, 1.0);
      if (fraction < 0.5)
      {
         framePointToPack.interpolate(fractionTempPoint1, fractionTempMidPoint, 2.0 * fraction);
      }
      else
      {
         framePointToPack.interpolate(fractionTempMidPoint, fractionTempPoint2, 2.0 * (fraction - 0.5));
      }
   }

   private void computeCoPPointsForHoldingPosition(List<FootstepTiming> footstepTimings, List<FootstepShiftFractions> footstepShiftFractions)
   {
      previousCoPLocation.setMatchingFrame(heldCoPPosition);
      clearHeldPosition();

      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(previousCoPLocation);
      contactStateProvider.setStartTime(0.0);

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringToPolygon.getFirst(),
                                            transferringFromPolygon.getFirst(),
                                            percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      double transferDuration = footstepTimings.get(0).getTransferTime();
      double splitFraction = footstepShiftFractions.get(0).getTransferSplitFraction();

      double segmentDuration = splitFraction * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setDuration(segmentDuration);

      // FIXME figure out the right fraction for this.
      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation, transferringFromPolygon.getLast(), transferringToPolygon.getLast(), finalTransferWeightDistribution.getValue());

      tempPointForCoPCalculation.changeFrame(worldFrame);
      segmentDuration = (1.0 - splitFraction) * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();

      SettableContactStateProvider previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setDuration(segmentDuration);
   }

   private void computeCoPPointsForStanding(List<FootstepTiming> footstepTimings, List<FootstepShiftFractions> footstepShiftFractions)
   {
      getDoubleSupportPolygonCentroid(previousCoPLocation, transferringToPolygon.getFirst(), transferringFromPolygon.getFirst());

      SettableContactStateProvider contactStateProvider = contactStateProviders.add();

      contactStateProvider.setStartCopPosition(previousCoPLocation);
      contactStateProvider.setStartTime(0.0);

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringToPolygon.getFirst(),
                                            transferringFromPolygon.getFirst(),
                                            percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      double transferDuration = footstepTimings.get(0).getTransferTime();
      double splitFraction = footstepShiftFractions.get(0).getTransferSplitFraction();
      double segmentDuration = splitFraction * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setDuration(segmentDuration);

      SettableContactStateProvider previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringToPolygon.getFirst(),
                                            transferringFromPolygon.getFirst(),
                                            percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      segmentDuration = (1.0 - splitFraction) * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setDuration(segmentDuration);
   }

   private void computeCoPPointsForFinalTransfer(int footstepIndex, boolean atAStop, List<Footstep> footsteps, List<FootstepTiming> footstepTimings,
                                                 List<FootstepShiftFractions> footstepShiftFractions)
   {
      ContactStateProvider previousContactState = contactStateProviders.getLast();

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringFromPolygon.getLast(),
                                            transferringToPolygon.getLast(),
                                            finalTransferWeightDistribution.getValue());

      double transferDuration = footstepTimings.get(footstepIndex).getTransferTime();
      double splitFraction = footstepShiftFractions.get(footstepIndex).getTransferSplitFraction();
      double segmentDuration = splitFraction * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);

      segmentDuration = (1.0 - splitFraction) * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);

      if (numberOfUpcomingFootsteps.getValue() > 0 && !atAStop)
      {
         RobotSide lastStepSide = footsteps.get(footsteps.size() - 1).getRobotSide().getOppositeSide();
         if (lastStepSide == RobotSide.RIGHT)
            percentageStandingWeightDistributionOnLeftFoot.set(1.0 - finalTransferWeightDistribution.getValue());
         else
            percentageStandingWeightDistributionOnLeftFoot.set(finalTransferWeightDistribution.getValue());
      }
   }

   private static void convertToFramePointRetainingZ(FramePoint3DBasics framePointToPack, FramePoint2DReadOnly framePoint2dToCopy,
                                                     ReferenceFrame referenceFrameToConvertTo)
   {
      framePointToPack.setIncludingFrame(framePoint2dToCopy, 0.0);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
   }

   private void computeCoPPointsForFootstepTransfer(double duration,
                                                    double splitFraction,
                                                    double weightDistribution,
                                                    FrameConvexPolygon2DReadOnly previousPolygon,
                                                    FrameConvexPolygon2DReadOnly nextPolygon,
                                                    RobotSide supportSide)
   {
      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation, previousPolygon, nextPolygon, weightDistribution);
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      previousContactState.setDuration(splitFraction * duration);
      previousContactState.setEndCopPosition(tempPointForCoPCalculation);

      computeEntryCoPPointLocation(tempPointForCoPCalculation, previousPolygon, nextPolygon, supportSide);
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

   private void computeCoPPointsForFlamingoSingleSupport(double duration,
                                                         double shiftFraction,
                                                         FrameConvexPolygon2DReadOnly supportPolygon,
                                                         FrameConvexPolygon2DReadOnly nextPolygon,
                                                         RobotSide supportSide)
   {
      double shiftDuration = shiftFraction * duration;
      double constantDuration = duration - shiftDuration;

      computeBallCoPLocation(tempPointForCoPCalculation, supportPolygon, nextPolygon, supportSide);
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(shiftDuration);

      computeFlamingoStanceCoPLocation(tempPointForCoPCalculation, supportPolygon);
      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(constantDuration);
   }

   private void computeExitCoPPointLocationForPreviousPlan(FramePoint3D exitCoPFromLastPlanToPack,
                                                           RobotSide swingSide,
                                                           boolean transferringToSameSideAsStartingFrom)
   {
      RobotSide previousSupportSide = transferringToSameSideAsStartingFrom ? swingSide.getOppositeSide() : swingSide;

      // checks if the previous CoP goes to a special location. If so, use this guy, and complete the function
      if (setExitCoPUnderSpecialCases(exitCoPFromLastPlanToPack,
                                      transferringFromPolygon.getFirst(),
                                      transferringToPolygon.getFirst(),
                                      previousSupportSide))
      {
         return;
      }

      // get the base CoP location, which the origin of the side that the robot is transferring from
      convertToFramePointRetainingZ(exitCoPFromLastPlanToPack, transferringFromPolygon.getFirst().getCentroid(),
                                    transferringFromPolygon.getFirst().getReferenceFrame());

      // add the offset, which is the sum of the static offset value, and a ratio of factor of the current step length
      Vector2DReadOnly copOffset = parameters.getExitCMPOffset();
      double copXOffset = copOffset.getX() + getStepLengthToCoPOffset(parameters.getExitCMPLengthOffsetFactor(), transferringToPolygon.getFirst(), transferringFromPolygon.getFirst());

      // clamp the offset value
      copXOffset = MathTools.clamp(copXOffset, parameters.getExitCMPMinX(), parameters.getExitCMPMaxX());

      // add the offset to the origin point
      exitCoPFromLastPlanToPack.add(copXOffset, previousSupportSide.negateIfRightSide(copOffset.getY()), 0.0);
      constrainToPolygon(exitCoPFromLastPlanToPack, transferringFromPolygon.getFirst(), parameters.getMinimumDistanceInsidePolygon());

      exitCoPFromLastPlanToPack.changeFrame(worldFrame);
   }

   private void computeEntryCoPPointLocation(FramePoint3DBasics copLocationToPack, FrameConvexPolygon2DReadOnly previousFootPolygon, FrameConvexPolygon2DReadOnly footPolygon, RobotSide supportSide)
   {
      convertToFramePointRetainingZ(copLocationToPack, footPolygon.getCentroid(), footPolygon.getReferenceFrame());

      Vector2DReadOnly copOffset = parameters.getEntryCMPOffset();
      double copXOffset = copOffset.getX() + getStepLengthToCoPOffset(parameters.getEntryCMPLengthOffsetFactor(), previousFootPolygon, footPolygon);
      copXOffset = MathTools.clamp(copXOffset, parameters.getEntryCMPMinX(), parameters.getEntryCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()), 0.0);

      constrainToPolygon(copLocationToPack, footPolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeBallCoPLocation(FramePoint3D copLocationToPack,
                                       FrameConvexPolygon2DReadOnly footPolygon,
                                       FrameConvexPolygon2DReadOnly nextFootPolygon,
                                       RobotSide supportSide)
   {
      convertToFramePointRetainingZ(copLocationToPack, footPolygon.getCentroid(), footPolygon.getReferenceFrame());

      Vector2DReadOnly copOffset = parameters.getBallCMPOffset();
      double copXOffset = copOffset.getX() + getStepLengthToCoPOffset(parameters.getBallCMPLengthOffsetFactor(), nextFootPolygon, footPolygon);
      copXOffset = MathTools.clamp(copXOffset, parameters.getBallCMPMinX(), parameters.getBallCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()), 0.0);

      constrainToPolygon(copLocationToPack, footPolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeExitCoPLocation(FramePoint3DBasics copLocationToPack,
                                       FrameConvexPolygon2DReadOnly footPolygon,
                                       FrameConvexPolygon2DReadOnly nextFootPolygon,
                                       RobotSide supportSide)
   {
      if (setExitCoPUnderSpecialCases(copLocationToPack, footPolygon, nextFootPolygon, supportSide))
         return;

      convertToFramePointRetainingZ(copLocationToPack, footPolygon.getCentroid(), footPolygon.getReferenceFrame());

      Vector2DReadOnly copOffset = parameters.getExitCMPOffset();
      double copXOffset = copOffset.getX() + getStepLengthToCoPOffset(parameters.getExitCMPLengthOffsetFactor(), nextFootPolygon, footPolygon);

      copXOffset = MathTools.clamp(copXOffset, parameters.getExitCMPMinX(), parameters.getExitCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()), 0.0);

      constrainToPolygon(copLocationToPack, footPolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeFlamingoStanceCoPLocation(FramePoint3DBasics copLocationToPack, FrameConvexPolygon2DReadOnly footPolygon)
   {
      convertToFramePointRetainingZ(copLocationToPack, footPolygon.getCentroid(), footPolygon.getReferenceFrame());

      constrainToPolygon(copLocationToPack, footPolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   /**
    * Checks for the following conditions to have been the case:
    *  - the support polygon is empty
    *  - the exit CoP goes in the toes
    *  - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setExitCoPUnderSpecialCases(FramePoint3DBasics framePointToPack,
                                               FrameConvexPolygon2DReadOnly supportFootPolygon,
                                               FrameConvexPolygon2DReadOnly upcomingSwingFootPolygon,
                                               RobotSide supportSide)
   {
      convertToFramePointRetainingZ(tempFramePoint1, upcomingSwingFootPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
      convertToFramePointRetainingZ(tempFramePoint2, supportFootPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
      double supportToSwingStepLength = tempFramePoint1.getX() - tempFramePoint2.getX();
      double supportToSwingStepHeight = tempFramePoint1.getZ() - tempFramePoint2.getZ();
      if (supportFootPolygon.getArea() == 0.0)
      {
         framePointToPack.setToZero(supportFootPolygon.getReferenceFrame());
         framePointToPack.set(supportFootPolygon.getVertex(0));
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else if (parameters.getPlanForToeOffCalculator().shouldPutCMPOnToes(supportToSwingStepLength, supportToSwingStepHeight))
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
         framePointToPack.add(supportFootPolygon.getMaxX() - exitCoPForwardSafetyMarginOnToes.getDoubleValue(),
                              supportSide.negateIfRightSide(parameters.getExitCMPOffset().getY()),
                              0.0);
         constrainToPolygon(framePointToPack, supportFootPolygon, safeDistanceFromCoPToSupportEdgesWhenSteppingDown.getDoubleValue());
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else
      {
         return false;
      }
   }

   private final FramePoint2D mostForwardPointOnNextStep = new FramePoint2D();

   private double getStepLengthToCoPOffset(double stepLengthToCoPOffsetFactor,
                                           FrameConvexPolygon2DReadOnly nextPolygon,
                                           FrameConvexPolygon2DReadOnly previousPolygon)
   {
      mostForwardPointOnNextStep.setIncludingFrame(nextPolygon.getVertex(
            EuclidGeometryPolygonTools.findVertexIndex(nextPolygon, true, Bound.MAX, Bound.MAX)));
      mostForwardPointOnNextStep.changeFrameAndProjectToXYPlane(previousPolygon.getReferenceFrame());

      double stepLength = mostForwardPointOnNextStep.getX() - previousPolygon.getMaxX();
      return stepLengthToCoPOffsetFactor * stepLength;
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by
    * projection
    */
   private void constrainToPolygon(FramePoint3DBasics copPointToConstrain,
                                   FrameConvexPolygon2DReadOnly constraintPolygon,
                                   double safeDistanceFromSupportPolygonEdges)
   {
      tempFramePoint2d.setIncludingFrame(copPointToConstrain);

      // don't need to do anything if it's already inside
      if (constraintPolygon.signedDistance(tempFramePoint2d) <= -safeDistanceFromSupportPolygonEdges)
         return;

      polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      copPointToConstrain.changeFrame(constraintPolygon.getReferenceFrame());
      tempPolygon.orthogonalProjection(tempFramePoint2d);
      copPointToConstrain.setIncludingFrame(tempFramePoint2d, 0.0);
   }

   // do not use these temporary variables anywhere except this method to avoid modifying them in unwanted places.
   private final FramePoint3D doubleSupportCentroidTempPoint1 = new FramePoint3D();
   private final FramePoint3D doubleSupportCentroidTempPoint2 = new FramePoint3D();

   /**
    * Updates the variable {@code currentDoubleSupportPolygon} from the specified swing and support
    * polygons
    */
   private void getDoubleSupportPolygonCentroid(FixedFramePoint3DBasics framePointToPack,
                                                FrameConvexPolygon2DReadOnly supportFootPolygon,
                                                FrameConvexPolygon2DReadOnly swingFootPolygon)
   {
      doubleSupportCentroidTempPoint1.setMatchingFrame(swingFootPolygon.getCentroid(), 0.0);
      doubleSupportCentroidTempPoint2.setMatchingFrame(supportFootPolygon.getCentroid(), 0.0);
      framePointToPack.interpolate(doubleSupportCentroidTempPoint1, doubleSupportCentroidTempPoint2, 0.5);
   }

   private void initializeAllFootPolygons(RobotSide upcomingSupportSide, boolean transferringToSameSideAsStartingFrom, boolean planningFromSwing,
                                          List<Footstep> footsteps)
   {
      transferringFromPolygon.clear();
      transferringToPolygon.clear();
      upcomingPolygon.clear();

      // in final transfer, or in stand
      if (footsteps.size() == 0)
      {
         if (transferringToSameSideAsStartingFrom)
         {
            setFootPolygonFromCurrentState(transferringFromPolygon.add(), upcomingSupportSide.getOppositeSide());
            setFootPolygonFromCurrentState(upcomingPolygon.add(), upcomingSupportSide);
            setFootPolygonFromCurrentState(transferringToPolygon.add(), upcomingSupportSide.getOppositeSide());
         }
         else
         {
            setFootPolygonFromCurrentState(transferringFromPolygon.add(), upcomingSupportSide.getOppositeSide());
            setFootPolygonFromCurrentState(upcomingPolygon.add(), upcomingSupportSide.getOppositeSide());
            setFootPolygonFromCurrentState(transferringToPolygon.add(), upcomingSupportSide);
         }
         return;
      }

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      RobotSide transferringFromSide = transferringToSameSideAsStartingFrom ? footsteps.get(0).getRobotSide().getOppositeSide() : footsteps.get(0).getRobotSide();

      if(planningFromSwing)
         setFootPolygonDirectly(transferringFromPolygon.add(), footFrameAtStartOfSwing, footPolygonAtStartOfSwing);
      else
         setFootPolygonFromCurrentState(transferringFromPolygon.add(), transferringFromSide);

      setFootPolygonFromCurrentState(transferringToPolygon.add(), footsteps.get(0).getRobotSide().getOppositeSide());
      setFootPolygonFromFootstep(upcomingPolygon.add(), footsteps.get(0));

      int footstepIndex = 1;
      for (; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
      {
         transferringFromPolygon.add().setIncludingFrame(transferringToPolygon.get(footstepIndex - 1));

         if (footsteps.get(footstepIndex).getRobotSide().equals(footsteps.get(footstepIndex - 1).getRobotSide()))
         { // stepping to the same side
            transferringToPolygon.add().setIncludingFrame(transferringFromPolygon.getLast());
         }
         else
         {
            transferringToPolygon.add().setIncludingFrame(upcomingPolygon.get(footstepIndex - 1));
         }
         setFootPolygonFromFootstep(upcomingPolygon.add(), footsteps.get(footstepIndex));
      }

      transferringFromPolygon.add().setIncludingFrame(transferringToPolygon.get(footstepIndex - 1));
      transferringToPolygon.add().setIncludingFrame(upcomingPolygon.get(footstepIndex - 1));
   }

   private void setFootPolygonFromFootstep(FrameConvexPolygon2DBasics framePolygonToPack, Footstep upcomingFootstep)
   {
      framePolygonToPack.clear(upcomingFootstep.getSoleReferenceFrame());

      if (upcomingFootstep.getPredictedContactPoints() != null && upcomingFootstep.getPredictedContactPoints().size() > 0)
      {
         polygonReference.clear();
         polygonReference.addVertices(Vertex2DSupplier.asVertex2DSupplier(upcomingFootstep.getPredictedContactPoints()));
         polygonReference.update();
         framePolygonToPack.addVertices(polygonReference);
      }
      else
         framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstep.getRobotSide()));
      framePolygonToPack.update();
   }

   private void setFootPolygonFromCurrentState(FrameConvexPolygon2D framePolygonToPack, RobotSide robotSide)
   {
      if (!supportFootPolygonsInSoleZUpFrames.get(robotSide).isEmpty() && !(supportFootPolygonsInSoleZUpFrames.get(robotSide).getNumberOfVertices() < 3))
         framePolygonToPack.setIncludingFrame(supportFootPolygonsInSoleZUpFrames.get(robotSide));
      else
      {
         framePolygonToPack.clear(soleZUpFrames.get(robotSide));
         framePolygonToPack.addVertices(defaultFootPolygons.get(robotSide));
      }
      framePolygonToPack.update();
   }

   private void setFootPolygonDirectly(FrameConvexPolygon2D frameConvexPolygonToPack, ReferenceFrame referenceFrame, Vertex2DSupplier vertexSupplier)
   {
      frameConvexPolygonToPack.clear(referenceFrame);
      frameConvexPolygonToPack.addVertices(vertexSupplier);
      frameConvexPolygonToPack.update();
   }
}