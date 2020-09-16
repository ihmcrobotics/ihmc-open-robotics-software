package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CoPPointPlanningParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
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
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.function.Supplier;

public class CoPTrajectoryGenerator
{

   private final CoPTrajectoryParameters parameters;

   private final String fullPrefix;
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();


   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble safeDistanceFromCoPToSupportEdgesWhenSteppingDown;
   private final YoDouble exitCoPForwardSafetyMarginOnToes;
   private final YoDouble percentageStandingWeightDistributionOnLeftFoot;
   private CoPPointName exitCoPName;
   private final YoDouble additionalTimeForFinalTransfer;

   // State variables
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2DReadOnly> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2DReadOnly> defaultFootPolygons = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberFootstepsToConsider;
   private final IntegerProvider numberOfUpcomingFootsteps;

   private final YoDouble finalTransferWeightDistribution;
   private final List<YoDouble> transferWeightDistributions;
   private final List<YoDouble> swingDurations;
   private final List<YoDouble> transferDurations;
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> swingShiftFractions;
   private final List<YoDouble> transferSplitFractions;
   private final List<FootstepData> upcomingFootstepsData;

   private final YoBoolean holdDesiredState;

   // Output variables
   private final RecyclingArrayList<CoPPointsInFoot> copLocationWaypoints;

   // Runtime variables
   private final FramePoint3D heldCoPPosition = new FramePoint3D();
   private final PoseReferenceFrame footFrameAtStartOfSwing = new PoseReferenceFrame("footFrameAtStartOfSwing", worldFrame);
   private final ConvexPolygon2D footPolygonAtStartOfSwing = new ConvexPolygon2D();

   private final FrameConvexPolygon2D tempPolygonA = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempPolygonB = new FrameConvexPolygon2D();

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

   public CoPTrajectoryGenerator(String namePrefix, CoPTrajectoryParameters parameters,
                                 int maxNumberOfFootstepsToConsider, BipedSupportPolygons bipedSupportPolygons,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                 IntegerProvider numberOfUpcomingFootsteps,
                                 List<FootstepData> upcomingFootstepsData, SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                 YoRegistry parentRegistry)
   {
      this(namePrefix, parameters, maxNumberOfFootstepsToConsider, bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), contactableFeet, numberFootstepsToConsider,
           numberOfUpcomingFootsteps, upcomingFootstepsData, soleZUpFrames, parentRegistry);

   }

   public CoPTrajectoryGenerator(String namePrefix,  CoPTrajectoryParameters parameters,
                                 int maxNumberOfFootstepsToConsider, SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                 IntegerProvider numberOfUpcomingFootsteps, List<FootstepData> upcomingFootstepsData,
                                 SideDependentList<? extends ReferenceFrame> soleZUpFrames, YoRegistry parentRegistry)
   {
      this.numberFootstepsToConsider = numberFootstepsToConsider;
      this.parameters = parameters;
      this.fullPrefix = namePrefix + "CoPTrajectoryGenerator";
      additionalTimeForFinalTransfer = new YoDouble(fullPrefix + "AdditionalTimeForFinalTransfer", registry);
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble(fullPrefix + "SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", parentRegistry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble(fullPrefix + "ExitCoPForwardSafetyMarginOnToes", parentRegistry);

      percentageStandingWeightDistributionOnLeftFoot = new YoDouble(namePrefix + "PercentageStandingWeightDistributionOnLeftFoot", registry);

      this.numberOfUpcomingFootsteps = numberOfUpcomingFootsteps;
      this.upcomingFootstepsData = upcomingFootstepsData;

      swingDurations = new ArrayList<>();
      transferDurations = new ArrayList<>();
      transferWeightDistributions = new ArrayList<>();
      transferSplitFractions = new ArrayList<>();
      swingSplitFractions = new ArrayList<>();
      swingShiftFractions = new ArrayList<>();

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDuration = new YoDouble("swingDuration" + i, registry);
         YoDouble transferDuration = new YoDouble("transferDuration" + i, registry);
         swingDuration.setToNaN();
         transferDuration.setToNaN();
         swingDurations.add(swingDuration);
         transferDurations.add(transferDuration);

         YoDouble transferWeightDistribution = new YoDouble("transferWeightDistribution" + i, registry);
         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         YoDouble swingShiftFraction = new YoDouble("swingShiftFraction" + i, registry);
         transferWeightDistribution.setToNaN();
         transferSplitFraction.setToNaN();
         swingSplitFraction.setToNaN();
         swingShiftFraction.setToNaN();
         transferWeightDistributions.add(transferWeightDistribution);
         transferSplitFractions.add(transferSplitFraction);
         swingSplitFractions.add(swingSplitFraction);
         swingShiftFractions.add(swingShiftFraction);
      }
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

      copLocationWaypoints = new RecyclingArrayList<>(maxNumberOfFootstepsToConsider + 2, new CoPPointsInFootSupplier());
      copLocationWaypoints.clear();
      copPointsConstructed = true;

      parentRegistry.addChild(registry);
      clear();
   }

   private int copSegmentNumber = 0;
   private boolean copPointsConstructed = false;

   private class CoPPointsInFootSupplier implements Supplier<CoPPointsInFoot>
   {

      @Override
      public CoPPointsInFoot get()
      {
         CoPPointsInFoot pointsInFoot;
         if (!copPointsConstructed)
            pointsInFoot = new CoPPointsInFoot(fullPrefix, copSegmentNumber, registry);
         else
            pointsInFoot = new CoPPointsInFoot(fullPrefix, copSegmentNumber, null);

         copSegmentNumber++;
         return pointsInFoot;
      }
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
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).reset();
      copLocationWaypoints.clear();
   }

   public int getNumberOfFootstepsRegistered()
   {
      return numberOfUpcomingFootsteps.getValue();
   }

   public Footstep getFootstep(int footstepIndex)
   {
      return upcomingFootstepsData.get(footstepIndex).getFootstep();
   }

   public void initializeForSwing()
   {
      RobotSide swingSide = upcomingFootstepsData.get(0).getSwingSide();
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

   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide, RobotSide previousTransferToSide)
   {
      copLocationWaypoints.clear();
      contactStateProviders.clear();

      boolean transferringToSameSideAsStartingFrom = previousTransferToSide != null && previousTransferToSide.equals(transferToSide);
      lastTransferToSide = previousTransferToSide == null ? transferToSide.getOppositeSide() : previousTransferToSide;
      initializeAllFootPolygons(transferToSide, transferringToSameSideAsStartingFrom, false);

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      // Put first CoP as per chicken support computations in case starting from rest
      if (atAStop && (holdDesiredState.getBooleanValue() || numberOfUpcomingFootsteps == 0))
      {
         if (holdDesiredState.getBooleanValue())
         {
            computeCoPPointsForHoldingPosition();
         }
         else
         { // just standing there
            computeCoPPointsForStanding(0.0);
         }
      }
      else
      {
         CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

         if (atAStop)
         {  // this guy is starting a series of steps from standing
            double fraction = transferToSide == RobotSide.LEFT ? percentageStandingWeightDistributionOnLeftFoot.getValue() : 1.0 - percentageStandingWeightDistributionOnLeftFoot.getValue();
            computeMidFeetPointByFractionForInitialTransfer(previousCoPLocation, fraction);
            copLocationWaypoint.addWaypoint(CoPPointName.START_COP, 0.0, previousCoPLocation);
         }
         else
         {  // starting while currently executing a step cycle
            clearHeldPosition();

            // Put first CoP at the exitCoP of the swing foot when starting in motion
            computeExitCoPPointLocationForPreviousPlan(previousCoPLocation, transferToSide.getOppositeSide(), transferringToSameSideAsStartingFrom);
            copLocationWaypoint.addWaypoint(exitCoPName, 0.0, previousCoPLocation);
         }


         // compute all the upcoming footsteps
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            computeCoPPointsForFootstepTransfer(footstepIndex);
            computeCoPPointsForFootstepSwing(footstepIndex);
         }
         computeCoPPointsForFinalTransfer(numberOfUpcomingFootsteps, atAStop);
      }
   }

   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      copLocationWaypoints.clear();
      clearHeldPosition();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(0);

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      if (numberOfUpcomingFootsteps == 0)
      {
         return;
      }

      initializeAllFootPolygons(null, false, true);
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      // compute cop waypoint location
      computeExitCoPPointLocationForPreviousPlan(previousCoPLocation, supportSide.getOppositeSide(), false);
      copLocationWaypoint.addWaypoint(exitCoPName, 0.0, previousCoPLocation);

      if (upcomingFootstepData.getSwingTime() == Double.POSITIVE_INFINITY)
      { // We're in flamingo support, so only do the current one
         computeCoPPointsForFootstepTransfer(0);
         computeCoPPointsForFlamingoSingleSupport();
      }
      else
      { // Compute all the upcoming waypoints
         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         {
            computeCoPPointsForFootstepTransfer(footstepIndex);
            computeCoPPointsForFootstepSwing(footstepIndex);
         }
         computeCoPPointsForFinalTransfer(numberOfUpcomingFootsteps, false);
      }
   }

   private void computeMidFeetPointByFractionForInitialTransfer(FramePoint3D framePointToPack, double fraction)
   {
      computeMidFeetPointByPositionFraction(framePointToPack, transferringFromPolygon.getFirst(), transferringToPolygon.getFirst(), fraction);
   }

   private void computeMidFeetPointByFractionForFinalTransfer(FramePoint3D framePointToPack, double fraction)
   {
      computeMidFeetPointByPositionFraction(framePointToPack, transferringFromPolygon.getLast(), transferringToPolygon.getLast(), fraction);
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
      getDoubleSupportPolygonCentroid(fractionTempMidPoint, footPolygonA, footPolygonB);

      fractionTempPoint1.setMatchingFrame(footPolygonA.getCentroid(), 0.0);
      fractionTempPoint2.setMatchingFrame(footPolygonB.getCentroid(), 0.0);

      framePointToPack.setToZero(worldFrame);

      fraction = MathTools.clamp(fraction, 0.0, 1.0);
      if (fraction < 0.5)
      {
         framePointToPack.interpolate(fractionTempPoint1, fractionTempMidPoint, 2.0 * fraction);
      }
      else
      {
         framePointToPack.interpolate(fractionTempMidPoint, fractionTempPoint2, 2.0 * (fraction - 0.5));
      }
      framePointToPack.changeFrame(worldFrame);
   }

   private void computeCoPPointsForHoldingPosition()
   {
      previousCoPLocation.setMatchingFrame(heldCoPPosition);
      clearHeldPosition();

      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(previousCoPLocation);

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringToPolygon.getFirst(),
                                            transferringFromPolygon.getFirst(),
                                            percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      double transferDuration = transferDurations.get(0).getDoubleValue();
      double splitFraction = transferSplitFractions.get(0).getDoubleValue();

      double startTime = 0.0;

      double segmentDuration = splitFraction * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(startTime, startTime + segmentDuration);

      startTime += segmentDuration;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(tempPointForCoPCalculation);

      // FIXME figure out the right fraction for this.
      computeMidFeetPointByFractionForFinalTransfer(tempPointForCoPCalculation, finalTransferWeightDistribution.getValue());
      tempPointForCoPCalculation.changeFrame(worldFrame);
      segmentDuration = (1.0 - splitFraction) * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(startTime, startTime + segmentDuration);
   }

   private double computeCoPPointsForStanding(double startTime)
   {
      double time = startTime;
      getDoubleSupportPolygonCentroid(previousCoPLocation, transferringToPolygon.getFirst(), transferringFromPolygon.getFirst());

      SettableContactStateProvider contactStateProvider = contactStateProviders.add();

      contactStateProvider.setStartCopPosition(previousCoPLocation);

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringToPolygon.getFirst(),
                                            transferringFromPolygon.getFirst(),
                                            percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      double transferDuration = transferDurations.get(0).getDoubleValue();
      double splitFraction = transferSplitFractions.get(0).getDoubleValue();
      double segmentDuration = splitFraction * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(time, segmentDuration);

      time += segmentDuration;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(tempPointForCoPCalculation);

      computeMidFeetPointByPositionFraction(tempPointForCoPCalculation,
                                            transferringToPolygon.getFirst(),
                                            transferringFromPolygon.getFirst(),
                                            percentageStandingWeightDistributionOnLeftFoot.getDoubleValue());

      segmentDuration = (1.0 - splitFraction) * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(time, time + segmentDuration);
      time += segmentDuration;

      return time;
   }

   private void computeCoPPointsForFinalTransfer(int footstepIndex, boolean atAStop)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      computeMidFeetPointByFractionForFinalTransfer(tempPointForCoPCalculation, finalTransferWeightDistribution.getValue());

      double transferDuration = transferDurations.get(footstepIndex).getDoubleValue();
      double splitFraction = transferSplitFractions.get(footstepIndex).getDoubleValue();
      double segmentDuration = splitFraction * transferDuration;
      segmentDuration += 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      copLocationWaypoint.addWaypoint(CoPPointName.MIDFEET_COP, segmentDuration, tempPointForCoPCalculation);


      computeMidFeetPointByFractionForFinalTransfer(tempPointForCoPCalculation, finalTransferWeightDistribution.getValue());
      tempPointForCoPCalculation.changeFrame(worldFrame);
      segmentDuration = (1.0 - splitFraction) * transferDuration + 0.5 * additionalTimeForFinalTransfer.getDoubleValue();
      copLocationWaypoint.addWaypoint(CoPPointName.FINAL_COP, segmentDuration, tempPointForCoPCalculation);

      if (numberOfUpcomingFootsteps.getValue() > 0 && !atAStop)
      {
         RobotSide lastStepSide = upcomingFootstepsData.get(upcomingFootstepsData.size() - 1).getSupportSide();
         if (lastStepSide == RobotSide.RIGHT)
         {
            percentageStandingWeightDistributionOnLeftFoot.set(1.0 - finalTransferWeightDistribution.getValue());
         }
         else
            {
               percentageStandingWeightDistributionOnLeftFoot.set(finalTransferWeightDistribution.getValue());
            }
      }
   }

   private static void convertToFramePointRetainingZ(FramePoint3DBasics framePointToPack, FramePoint2DReadOnly framePoint2dToCopy,
                                                     ReferenceFrame referenceFrameToConvertTo)
   {
      framePointToPack.setIncludingFrame(framePoint2dToCopy, 0.0);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
   }

   private void computeCoPPointsForFootstepTransfer(int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.getLast();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      double transferDuration = transferDurations.get(footstepIndex).getDoubleValue();
      double splitFraction = transferSplitFractions.get(footstepIndex).getDoubleValue();

      computeMidfeetCoPPointLocation(tempPointForCoPCalculation, footstepIndex);
      copLocationWaypoint.addWaypoint(CoPPointName.MIDFEET_COP, splitFraction * transferDuration, tempPointForCoPCalculation);

      computeEntryCoPPointLocation(tempPointForCoPCalculation, upcomingFootstepData.getSupportSide(), footstepIndex);
      copLocationWaypoint.addWaypoint(CoPPointName.ENTRY_COP, (1.0 - splitFraction) * transferDuration, tempPointForCoPCalculation);
   }

   private void computeCoPPointsForFootstepSwing(int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.getLast();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      computeBallCoPLocation(tempPointForCoPCalculation, upcomingFootstepData.getSupportSide(), footstepIndex);
      copLocationWaypoint.addWaypoint(CoPPointName.MIDFOOT_COP, getShiftToBallDuration(footstepIndex), tempPointForCoPCalculation);
      computeExitCoPLocation(tempPointForCoPCalculation, upcomingFootstepData.getSupportSide(), footstepIndex);

      copLocationWaypoint.addWaypoint(CoPPointName.EXIT_COP, getShiftToToeDuration(footstepIndex), tempPointForCoPCalculation);
      copLocationWaypoint.addWaypoint(CoPPointName.EXIT_COP, getDurationOnToe(footstepIndex), tempPointForCoPCalculation);
   }

   private double computeCoPPointsForFlamingoSingleSupport()
   {
      SettableContactStateProvider lastContactStateProvider = contactStateProviders.getLast();
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(lastContactStateProvider.getCopEndPosition());
      double startTime = lastContactStateProvider.getTimeInterval().getEndTime();

      RobotSide supportSide = upcomingFootstepsData.get(0).getSupportSide();

      computeBallCoPLocation(tempPointForCoPCalculation, supportSide, 0);
      double duration = getShiftToBallDuration(0);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(startTime, startTime + duration);

      startTime += duration;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartCopPosition(tempPointForCoPCalculation);

      duration = getShiftToToeDuration(0);
      computeFlamingoStanceCoPLocation(tempPointForCoPCalculation);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(startTime, startTime + duration);

      startTime += duration;
      contactStateProvider = contactStateProviders.add();
      duration = getDurationOnToe(0);
      contactStateProvider.setStartCopPosition(tempPointForCoPCalculation);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);
      contactStateProvider.getTimeInterval().setInterval(startTime, startTime + duration);

      startTime += duration;
      return startTime;
   }

   private void computeExitCoPPointLocationForPreviousPlan(FramePoint3D exitCoPFromLastPlanToPack,
                                                           RobotSide swingSide,
                                                           boolean transferringToSameSideAsStartingFrom)
   {
      RobotSide previousSupportSide = transferringToSameSideAsStartingFrom ? swingSide.getOppositeSide() : swingSide;

      // checks if the previous CoP goes to a special location. If so, use this guy, and complete the function
      if (setInitialExitCoPUnderSpecialCases(exitCoPFromLastPlanToPack, previousSupportSide, transferringToPolygon.getFirst(),
                                             transferringFromPolygon.getFirst()))
      {
         return;
      }

      // get the base CoP location, which the origin of the side that the robot is transferring from
      convertToFramePointRetainingZ(exitCoPFromLastPlanToPack, transferringFromPolygon.getFirst().getCentroid(),
                                    transferringFromPolygon.getFirst().getReferenceFrame());

      // add the offset, which is the sum of the static offset value, and a ratio of factor of the current step length
      Vector2DReadOnly copOffset = parameters.getExitCMPOffset();
      double copXOffset = copOffset.getX() + getPreviousStepLengthToCoPOffset(parameters.getExitCMPLengthOffsetFactor());

      // clamp the offset value
      copXOffset = MathTools.clamp(copXOffset, parameters.getExitCMPMinX(), parameters.getExitCMPMaxX());

      // add the offset to the origin point
      exitCoPFromLastPlanToPack.add(copXOffset, previousSupportSide.negateIfRightSide(copOffset.getY()), 0.0);
      constrainToPolygon(exitCoPFromLastPlanToPack, transferringFromPolygon.getFirst(), parameters.getMinimumDistanceInsidePolygon());

      exitCoPFromLastPlanToPack.changeFrame(worldFrame);
   }

   private void computeMidfeetCoPPointLocation(FramePoint3DBasics copLocationToPack, int footstepIndex)
   {
      computeMidFeetPointByPositionFraction(copLocationToPack,
                                            transferringFromPolygon.get(footstepIndex),
                                            transferringToPolygon.get(footstepIndex),
                                            transferWeightDistributions.get(footstepIndex).getDoubleValue());

      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeEntryCoPPointLocation(FramePoint3DBasics copLocationToPack, RobotSide supportSide, int footstepIndex)
   {
      convertToFramePointRetainingZ(copLocationToPack, transferringToPolygon.get(footstepIndex).getCentroid(),
                                    transferringToPolygon.get(footstepIndex).getReferenceFrame());

      Vector2DReadOnly copOffset = parameters.getEntryCMPOffset();
      double copXOffset = copOffset.getX() + getEntryStepLengthToCoPOffset(parameters.getEntryCMPLengthOffsetFactor(), footstepIndex);
      copXOffset = MathTools.clamp(copXOffset, parameters.getEntryCMPMinX(), parameters.getEntryCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()), 0.0);

      constrainToPolygon(copLocationToPack, transferringToPolygon.get(footstepIndex), parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeBallCoPLocation(FramePoint3D copLocationToPack, RobotSide supportSide, int footstepIndex)
   {
      convertToFramePointRetainingZ(copLocationToPack, transferringToPolygon.get(footstepIndex).getCentroid(),
                                    transferringToPolygon.get(footstepIndex).getReferenceFrame());

      Vector2DReadOnly copOffset = parameters.getBallCMPOffset();
      double copXOffset = copOffset.getX() + getExitStepLengthToCoPOffset(parameters.getBallCMPLengthOffsetFactor(), footstepIndex);
      copXOffset = MathTools.clamp(copXOffset, parameters.getBallCMPMinX(), parameters.getBallCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()), 0.0);

      constrainToPolygon(copLocationToPack, transferringToPolygon.get(footstepIndex), parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeExitCoPLocation(FramePoint3DBasics copLocationToPack, RobotSide supportSide, int footstepIndex)
   {
      if (setExitCoPUnderSpecialCases(copLocationToPack, supportSide, footstepIndex))
         return;

      convertToFramePointRetainingZ(copLocationToPack, transferringToPolygon.get(footstepIndex).getCentroid(),
                                    transferringToPolygon.get(footstepIndex).getReferenceFrame());

      Vector2DReadOnly copOffset = parameters.getExitCMPOffset();
      double copXOffset = copOffset.getX() + getExitStepLengthToCoPOffset(parameters.getExitCMPLengthOffsetFactor(), footstepIndex);

      copXOffset = MathTools.clamp(copXOffset, parameters.getExitCMPMinX(), parameters.getExitCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfRightSide(copOffset.getY()), 0.0);

      constrainToPolygon(copLocationToPack, transferringToPolygon.get(footstepIndex), parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private void computeFlamingoStanceCoPLocation(FramePoint3D copLocationToPack)
   {
      convertToFramePointRetainingZ(copLocationToPack, transferringToPolygon.getFirst().getCentroid(), transferringToPolygon.getFirst().getReferenceFrame());

      constrainToPolygon(copLocationToPack, transferringToPolygon.getFirst(), parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrame(worldFrame);
   }

   private double getShiftToBallDuration(int footstepIndex)
   {
      double swingDuration = swingDurations.get(footstepIndex).getDoubleValue();
      if (Double.isInfinite(swingDuration))
         swingDuration = SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE;

      return swingShiftFractions.get(footstepIndex).getDoubleValue() * swingSplitFractions.get(footstepIndex).getDoubleValue() * swingDuration;
   }

   private double getShiftToToeDuration(int footstepIndex)
   {
      double swingDuration = swingDurations.get(footstepIndex).getDoubleValue();
      if (Double.isInfinite(swingDuration))
         swingDuration = SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE;

      return (1.0 - swingShiftFractions.get(footstepIndex).getDoubleValue()) * swingSplitFractions.get(footstepIndex).getDoubleValue() * swingDuration;
   }

   private double getDurationOnToe(int footstepIndex)
   {
      double swingDuration = swingDurations.get(footstepIndex).getDoubleValue();
      if (Double.isInfinite(swingDuration))
         swingDuration = SmoothCMPBasedICPPlanner.SUFFICIENTLY_LARGE;

      return (1.0 - swingSplitFractions.get(footstepIndex).getDoubleValue()) * swingDuration;
   }

   /**
    * Checks for the following conditions to have been the case:
    *  - the support polygon is empty
    *  - the exit CoP goes in the toes
    *  - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setInitialExitCoPUnderSpecialCases(FramePoint3D exitCoPToPack,
                                                      RobotSide previousSupportSide,
                                                      FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringTo,
                                                      FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringFrom)
   {
      return setExitCoPUnderSpecialCases(exitCoPToPack, footPolygonOfSideTransferringFrom, footPolygonOfSideTransferringTo, previousSupportSide);
   }

   /**
    * Checks for the following conditions to have been the case:
    *  - the support polygon is empty
    *  - the exit CoP goes in the toes
    *  - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setExitCoPUnderSpecialCases(FramePoint3DBasics exitCoPToPack, RobotSide supportSide, int footstepIndex)
   {
      return setExitCoPUnderSpecialCases(exitCoPToPack, transferringToPolygon.get(footstepIndex), transferringToPolygon.get(footstepIndex + 1), supportSide);
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

   private double getPreviousStepLengthToCoPOffset(double stepLengthToCoPOffsetFactor)
   {
      tempFramePoint2d.setIncludingFrame(transferringToPolygon.getFirst().getVertex(
            EuclidGeometryPolygonTools.findVertexIndex(transferringToPolygon.getFirst(), true, Bound.MAX, Bound.MAX)));
      convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, transferringFromPolygon.getFirst().getReferenceFrame());
      return getStepLengthBasedOffset(transferringFromPolygon.getFirst(), tempFramePoint1, stepLengthToCoPOffsetFactor);
   }

   private double getEntryStepLengthToCoPOffset(double stepLengthToCoPOffsetFactor, int footstepIndex)
   {
      tempFramePoint2d.setIncludingFrame(transferringFromPolygon.get(footstepIndex).getVertex(
            EuclidGeometryPolygonTools.findVertexIndex(transferringFromPolygon.get(footstepIndex), true, Bound.MAX, Bound.MAX)));
      convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, transferringToPolygon.get(footstepIndex).getReferenceFrame());
      return getStepLengthBasedOffset(transferringToPolygon.get(footstepIndex), tempFramePoint1, stepLengthToCoPOffsetFactor);
   }

   private double getExitStepLengthToCoPOffset(double stepLengthToCoPOffsetFactor, int footstepIndex)
   {
      tempFramePoint2d.setIncludingFrame(transferringToPolygon.get(footstepIndex + 1).getVertex(
            EuclidGeometryPolygonTools.findVertexIndex(transferringToPolygon.get(footstepIndex + 1), true, Bound.MAX, Bound.MAX)));
      convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, transferringToPolygon.get(footstepIndex).getReferenceFrame());
      return getStepLengthBasedOffset(transferringToPolygon.get(footstepIndex), tempFramePoint1, stepLengthToCoPOffsetFactor);
   }

   private static double getStepLengthBasedOffset(FrameConvexPolygon2DReadOnly supportPolygon, FramePoint3DReadOnly referencePoint,
                                                  double stepLengthToCoPOffsetFactor)
   {
      supportPolygon.checkReferenceFrameMatch(referencePoint);
      return stepLengthToCoPOffsetFactor * (referencePoint.getX() - supportPolygon.getMaxX());
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

   private void initializeAllFootPolygons(RobotSide upcomingSupportSide, boolean transferringToSameSideAsStartingFrom, boolean planningFromSwing)
   {
      transferringFromPolygon.clear();
      transferringToPolygon.clear();
      upcomingPolygon.clear();

      // in final transfer, or in stand
      if (upcomingFootstepsData.size() == 0)
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

      RobotSide transferringFromSide = transferringToSameSideAsStartingFrom ? upcomingFootstepsData.get(0).getSupportSide() : upcomingFootstepsData.get(0).getSwingSide();

      if(planningFromSwing)
         setFootPolygonDirectly(transferringFromPolygon.add(), footFrameAtStartOfSwing, footPolygonAtStartOfSwing);
      else
         setFootPolygonFromCurrentState(transferringFromPolygon.add(), transferringFromSide);

      setFootPolygonFromCurrentState(transferringToPolygon.add(), upcomingFootstepsData.get(0).getSupportSide());
      setFootPolygonFromFootstep(upcomingPolygon.add(), 0);

      int footstepIndex = 1;
      for (; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
      {
         transferringFromPolygon.add().setIncludingFrame(transferringToPolygon.get(footstepIndex - 1));

         if (upcomingFootstepsData.get(footstepIndex).getSwingSide().equals(upcomingFootstepsData.get(footstepIndex - 1).getSwingSide()))
         { // stepping to the same side
            transferringToPolygon.add().setIncludingFrame(transferringFromPolygon.getLast());
         }
         else
         {
            transferringToPolygon.add().setIncludingFrame(upcomingPolygon.get(footstepIndex - 1));
         }
         setFootPolygonFromFootstep(upcomingPolygon.add(), footstepIndex);
      }

      transferringFromPolygon.add().setIncludingFrame(transferringToPolygon.get(footstepIndex - 1));
      transferringToPolygon.add().setIncludingFrame(upcomingPolygon.get(footstepIndex - 1));
   }

   private void setFootPolygonFromFootstep(FrameConvexPolygon2D framePolygonToPack, int footstepIndex)
   {
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);
      Footstep upcomingFootstep = upcomingFootstepData.getFootstep();

      framePolygonToPack.clear(upcomingFootstep.getSoleReferenceFrame());

      if (footstepIndex < upcomingFootstepsData.size() && upcomingFootstep.getPredictedContactPoints() != null
            && upcomingFootstep.getPredictedContactPoints().size() > 0)
      {
         polygonReference.clear();
         polygonReference.addVertices(Vertex2DSupplier.asVertex2DSupplier(upcomingFootstep.getPredictedContactPoints()));
         polygonReference.update();
         framePolygonToPack.addVertices(polygonReference);
      }
      else
         framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstepData.getSwingSide()));
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


   public void getDoubleSupportPolygonCentroid(FixedFramePoint3DBasics copPositionToPack)
   {
      setFootPolygonFromCurrentState(tempPolygonA, lastTransferToSide.getOppositeSide());
      setFootPolygonFromCurrentState(tempPolygonB, lastTransferToSide);

      computeMidFeetPointByPositionFraction(tempFramePoint1, tempPolygonA, tempPolygonB, finalTransferWeightDistribution.getValue());
      tempFramePoint1.changeFrame(copPositionToPack.getReferenceFrame());
      copPositionToPack.set(tempFramePoint1);
   }
}