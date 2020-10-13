package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.*;

public interface FootstepPlannerParametersBasics extends FootstepPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void set(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      setAll(footstepPlannerParameters.getAll());
   }

   default void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      set(FootstepPlannerParameterKeys.checkForBodyBoxCollisions, checkForBodyBoxCollisions);
   }

   default void setCheckForPathCollisions(boolean checkForPathCollisions)
   {
      set(FootstepPlannerParameterKeys.checkForPathCollisions, checkForPathCollisions);
   }

   default void setMinimumDistanceFromCliffBottoms(double distance)
   {
      set(FootstepPlannerParameterKeys.minimumDistanceFromCliffBottoms, distance);
   }

   default void setCliffBaseHeightToAvoid(double height)
   {
      set(FootstepPlannerParameterKeys.cliffBaseHeightToAvoid, height);
   }

   default void setMinimumDistanceFromCliffTops(double distance)
   {
      set(FootstepPlannerParameterKeys.minimumDistanceFromCliffTops, distance);
   }

   default void setCliffTopHeightToAvoid(double height)
   {
      set(FootstepPlannerParameterKeys.cliffTopHeightToAvoid, height);
   }

   default void setMaximumStepReach(double reach)
   {
      set(FootstepPlannerParameterKeys.maxStepReach, reach);
   }

   default void setMaximumStepWidth(double width)
   {
      set(FootstepPlannerParameterKeys.maxStepWidth, width);
   }

   default void setMaximumStepYaw(double yaw)
   {
      set(FootstepPlannerParameterKeys.maxStepYaw, yaw);
   }

   default void setMinimumStepYaw(double yaw)
   {
      set(FootstepPlannerParameterKeys.minStepYaw, yaw);
   }

   default void setMaximumStepZ(double maxStepZ)
   {
      set(FootstepPlannerParameterKeys.maxStepZ, maxStepZ);
   }

   default void setMaximumSwingZ(double maxSwingZ)
   {
      set(FootstepPlannerParameterKeys.maxSwingZ, maxSwingZ);
   }

   default void setMaximumSwingReach(double maxSwingReach)
   {
      set(FootstepPlannerParameterKeys.maxSwingReach, maxSwingReach);
   }

   default void setMinimumStepZWhenFullyPitched(double stepZ)
   {
      set(FootstepPlannerParameterKeys.minStepZWhenFullyPitched, stepZ);
   }

   default void setMaximumStepXWhenFullyPitched(double stepX)
   {
      set(FootstepPlannerParameterKeys.maxStepXWhenFullyPitched, stepX);
   }

   default void setStepYawReductionFactorAtMaxReach(double factor)
   {
      set(FootstepPlannerParameterKeys.stepYawReductionFactorAtMaxReach, factor);
   }

   default void setMaximumXYWiggleDistance(double wiggleDistance)
   {
      set(FootstepPlannerParameterKeys.maximumXYWiggleDistance, wiggleDistance);
   }

   default void setMaximumYawWiggle(double wiggleDistance)
   {
      set(FootstepPlannerParameterKeys.maximumYawWiggle, wiggleDistance);
   }

   default void setMinimumFootholdPercent(double footholdPercent)
   {
      set(FootstepPlannerParameterKeys.minFootholdPercent, footholdPercent);
   }

   default void setMinimumStepLength(double minimumStepLength)
   {
      set(FootstepPlannerParameterKeys.minStepLength, minimumStepLength);
   }

   default void setMinimumStepWidth(double minimumStepWidth)
   {
      set(FootstepPlannerParameterKeys.minStepWidth, minimumStepWidth);
   }

   default void setMinimumSurfaceInclineRadians(double surfaceInclineRadians)
   {
      set(FootstepPlannerParameterKeys.minSurfaceIncline, surfaceInclineRadians);
   }

   default void setMinClearanceFromStance(double clearance)
   {
      set(FootstepPlannerParameterKeys.minClearanceFromStance, clearance);
   }

   default void setWiggleInsideDeltaTarget(double wiggleInsideDeltaTarget)
   {
      set(FootstepPlannerParameterKeys.wiggleInsideDeltaTarget, wiggleInsideDeltaTarget);
   }

   default void setWiggleInsideDeltaMinimum(double wiggleInsideDeltaMinimum)
   {
      set(FootstepPlannerParameterKeys.wiggleInsideDeltaMinimum, wiggleInsideDeltaMinimum);
   }

   default void setMaximumStepZWhenSteppingUp(double maxStepZ)
   {
      set(FootstepPlannerParameterKeys.maximumStepZWhenSteppingUp, maxStepZ);
   }

   default void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      set(FootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   default void setMaximumStepWidthWhenSteppingUp(double maximumStepWidthWhenSteppingUp)
   {
      set(FootstepPlannerParameterKeys.maximumStepWidthWhenSteppingUp, maximumStepWidthWhenSteppingUp);
   }

   default void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   default void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }

   default void setMaximumStepYWhenForwardAndDown(double maximumStepYWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepYWhenForwardAndDown, maximumStepYWhenForwardAndDown);
   }

   default void setIdealFootstepWidth(double idealFootstepWidth)
   {
      set(FootstepPlannerParameterKeys.idealFootstepWidth, idealFootstepWidth);
   }

   default void setIdealFootstepLength(double idealFootstepLength)
   {
      set(FootstepPlannerParameterKeys.idealFootstepLength, idealFootstepLength);
   }

   default void setIdealSideStepWidth(double idealSideStepWidth)
   {
      set(FootstepPlannerParameterKeys.idealSideStepWidth, idealSideStepWidth);
   }

   default void setIdealBackStepLength(double idealBackStepLength)
   {
      set(FootstepPlannerParameterKeys.idealBackStepLength, idealBackStepLength);
   }

   default void setIdealStepLengthAtMaxStepZ(double idealStepLengthAtMaxStepZ)
   {
      set(FootstepPlannerParameterKeys.idealStepLengthAtMaxStepZ, idealStepLengthAtMaxStepZ);
   }

   default void setWiggleWhilePlanning(boolean wiggleWhilePlanning)
   {
      set(FootstepPlannerParameterKeys.wiggleWhilePlanning, wiggleWhilePlanning);
   }

   default void setEnableConcaveHullWiggler(boolean enableConcaveHullWiggler)
   {
      set(FootstepPlannerParameterKeys.enableConcaveHullWiggler, enableConcaveHullWiggler);
   }

   default void setMaximumZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      set(FootstepPlannerParameterKeys.maximumZPenetrationOnValleyRegions, maximumZPenetrationOnValleyRegions);
   }

   default void setBodyBoxHeight(double bodyBoxHeight)
   {
      set(FootstepPlannerParameterKeys.bodyBoxHeight, bodyBoxHeight);
   }

   default void setBodyBoxDepth(double bodyBoxDepth)
   {
      set(FootstepPlannerParameterKeys.bodyBoxDepth, bodyBoxDepth);
   }

   default void setBodyBoxWidth(double bodyBoxWidth)
   {
      set(FootstepPlannerParameterKeys.bodyBoxWidth, bodyBoxWidth);
   }

   default void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      set(FootstepPlannerParameterKeys.bodyBoxBaseX, bodyBoxBaseX);
   }

   default void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      set(FootstepPlannerParameterKeys.bodyBoxBaseY, bodyBoxBaseY);
   }

   default void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      set(FootstepPlannerParameterKeys.bodyBoxBaseZ, bodyBoxBaseZ);
   }

   default void setMaximumSnapHeight(double maximumSnapHeight)
   {
      set(FootstepPlannerParameterKeys.maximumSnapHeight, maximumSnapHeight);
   }

   default void setAStarHeuristicsWeight(double aStarHeuristicsWeight)
   {
      set(FootstepPlannerParameterKeys.aStarHeuristicsWeight, aStarHeuristicsWeight);
   }

   default void setYawWeight(double yawWeight)
   {
      set(FootstepPlannerParameterKeys.yawWeight, yawWeight);
   }

   default void setPitchWeight(double pitchWeight)
   {
      set(FootstepPlannerParameterKeys.pitchWeight, pitchWeight);
   }

   default void setRollWeight(double rollWeight)
   {
      set(FootstepPlannerParameterKeys.rollWeight, rollWeight);
   }

   default void setForwardWeight(double forwardWeight)
   {
      set(FootstepPlannerParameterKeys.forwardWeight, forwardWeight);
   }

   default void setLateralWeight(double lateralWeight)
   {
      set(FootstepPlannerParameterKeys.lateralWeight, lateralWeight);
   }

   default void setStepUpWeight(double stepUpWeight)
   {
      set(FootstepPlannerParameterKeys.stepUpWeight, stepUpWeight);
   }

   default void setStepDownWeight(double stepDownWeight)
   {
      set(FootstepPlannerParameterKeys.stepDownWeight, stepDownWeight);
   }

   default void setCostPerStep(double costPerStep)
   {
      set(FootstepPlannerParameterKeys.costPerStep, costPerStep);
   }

   default void setNumberOfBoundingBoxChecks(int numberOfBoundingBoxChecks)
   {
      set(FootstepPlannerParameterKeys.numberOfBoundingBoxChecks, numberOfBoundingBoxChecks);
   }

   default void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      set(FootstepPlannerParameterKeys.maximum2dDistanceFromBoundingBoxToPenalize, maximum2dDistanceFromBoundingBoxToPenalize);
   }

   default void setFinalTurnProximity(double finalTurnProximity)
   {
      set(FootstepPlannerParameterKeys.finalTurnProximity, finalTurnProximity);
   }

   default void setFootholdAreaWeight(double footholdAreaWeight)
   {
      set(FootstepPlannerParameterKeys.footholdAreaWeight, footholdAreaWeight);
   }

   default void setDistanceFromPathTolerance(double tolerance)
   {
      set(distanceFromPathTolerance, tolerance);
   }

   default void setDeltaYawFromReferenceTolerance(double tolerance)
   {
      set(deltaYawFromReferenceTolerance, tolerance);
   }

   default void setMaximumBranchFactor(int maximumBranchFactor)
   {
      set(FootstepPlannerParameterKeys.maximumBranchFactor, maximumBranchFactor);
   }

   default void setEnableExpansionMask(boolean enableExpansionMask)
   {
      set(FootstepPlannerParameterKeys.enableExpansionMask, enableExpansionMask);
   }

   default void setEnableShinCollisionCheck(boolean enableShinCollisionCheck)
   {
      set(FootstepPlannerParameterKeys.enableShinCollisionCheck, enableShinCollisionCheck);
   }

   default void setShinToeClearance(double shinToeClearance)
   {
      set(FootstepPlannerParameterKeys.shinToeClearance, shinToeClearance);
   }

   default void setShinHeelClearance(double shinHeelClearance)
   {
      set(FootstepPlannerParameterKeys.shinHeelClearance, shinHeelClearance);
   }

   default void setShinLength(double shinLength)
   {
      set(FootstepPlannerParameterKeys.shinLength, shinLength);
   }

   default void setShinHeightOffset(double shinHeightOffet)
   {
      set(FootstepPlannerParameterKeys.shinHeightOffet, shinHeightOffet);
   }

   default void set(FootstepPlannerParametersPacket parametersPacket)
   {
      double noValue = FootstepPlannerParametersPacket.DEFAULT_NO_VALUE;
      setCheckForBodyBoxCollisions(parametersPacket.getCheckForBodyBoxCollisions());
      setCheckForPathCollisions(parametersPacket.getCheckForPathCollisions());
      setNumberOfBoundingBoxChecks((int) parametersPacket.getNumberOfBoundingBoxChecks());
      if (parametersPacket.getIdealFootstepWidth() != noValue)
         setIdealFootstepWidth(parametersPacket.getIdealFootstepWidth());
      if (parametersPacket.getIdealFootstepLength() != noValue)
         setIdealFootstepLength(parametersPacket.getIdealFootstepLength());
      if (parametersPacket.getIdealSideStepWidth() != noValue)
         setIdealSideStepWidth(parametersPacket.getIdealSideStepWidth());
      if (parametersPacket.getIdealBackStepLength() != noValue)
         setIdealBackStepLength(parametersPacket.getIdealBackStepLength());
      if (parametersPacket.getIdealStepLengthAtMaxStepZ() != noValue)
         setIdealStepLengthAtMaxStepZ(parametersPacket.getIdealStepLengthAtMaxStepZ());
      if (parametersPacket.getWiggleInsideDeltaTarget() != noValue)
         setWiggleInsideDeltaTarget(parametersPacket.getWiggleInsideDeltaTarget());
      if (parametersPacket.getWiggleInsideDeltaMinimum() != noValue)
         setWiggleInsideDeltaMinimum(parametersPacket.getWiggleInsideDeltaMinimum());
      if (parametersPacket.getMaximumStepReach() != noValue)
         setMaximumStepReach(parametersPacket.getMaximumStepReach());
      if (parametersPacket.getMaximumStepYaw() != noValue)
         setMaximumStepYaw(parametersPacket.getMaximumStepYaw());
      if (parametersPacket.getMinimumStepWidth() != noValue)
         setMinimumStepWidth(parametersPacket.getMinimumStepWidth());
      if (parametersPacket.getMinimumStepLength() != noValue)
         setMinimumStepLength(parametersPacket.getMinimumStepLength());
      if (parametersPacket.getMinimumStepYaw() != noValue)
         setMinimumStepYaw(parametersPacket.getMinimumStepYaw());
      if (parametersPacket.getMaximumStepReachWhenSteppingUp() != noValue)
         setMaximumStepReachWhenSteppingUp(parametersPacket.getMaximumStepReachWhenSteppingUp());
      if (parametersPacket.getMaximumStepWidthWhenSteppingUp() != noValue)
         setMaximumStepWidthWhenSteppingUp(parametersPacket.getMaximumStepWidthWhenSteppingUp());
      if (parametersPacket.getMaximumStepZWhenSteppingUp() != noValue)
         setMaximumStepZWhenSteppingUp(parametersPacket.getMaximumStepZWhenSteppingUp());
      if (parametersPacket.getMaximumStepXWhenForwardAndDown() != noValue)
         setMaximumStepXWhenForwardAndDown(parametersPacket.getMaximumStepXWhenForwardAndDown());
      if (parametersPacket.getMaximumStepYWhenForwardAndDown() != noValue)
         setMaximumStepYWhenForwardAndDown(parametersPacket.getMaximumStepYWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZWhenForwardAndDown() != noValue)
         setMaximumStepZWhenForwardAndDown(parametersPacket.getMaximumStepZWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZ() != noValue)
         setMaximumStepZ(parametersPacket.getMaximumStepZ());
      if (parametersPacket.getMaximumSwingZ() != noValue)
         setMaximumSwingZ(parametersPacket.getMaximumSwingZ());
      if (parametersPacket.getMaximumSwingReach() != noValue)
         setMaximumSwingReach(parametersPacket.getMaximumSwingReach());
      if (parametersPacket.getMinimumStepZWhenFullyPitched() != noValue)
         setMinimumStepZWhenFullyPitched(parametersPacket.getMinimumStepZWhenFullyPitched());
      if (parametersPacket.getMaximumStepXWhenFullyPitched() != noValue)
         setMaximumStepXWhenFullyPitched(parametersPacket.getMaximumStepXWhenFullyPitched());
      if (parametersPacket.getStepYawReductionFactorAtMaxReach() != noValue)
         setStepYawReductionFactorAtMaxReach(parametersPacket.getStepYawReductionFactorAtMaxReach());
      if (parametersPacket.getMinimumFootholdPercent() != noValue)
         setMinimumFootholdPercent(parametersPacket.getMinimumFootholdPercent());
      if (parametersPacket.getMinimumSurfaceInclineRadians() != noValue)
         setMinimumSurfaceInclineRadians(parametersPacket.getMinimumSurfaceInclineRadians());
      setWiggleWhilePlanning(parametersPacket.getWiggleWhilePlanning());
      setEnableConcaveHullWiggler(parametersPacket.getEnableConcaveHullWiggler());
      if (parametersPacket.getMaximumXyWiggleDistance() != noValue)
         setMaximumXYWiggleDistance(parametersPacket.getMaximumXyWiggleDistance());
      if (parametersPacket.getMaximumYawWiggle() != noValue)
         setMaximumYawWiggle(parametersPacket.getMaximumYawWiggle());
      if (parametersPacket.getMaximumZPenetrationOnValleyRegions() != noValue)
         setMaximumZPenetrationOnValleyRegions(parametersPacket.getMaximumZPenetrationOnValleyRegions());
      if (parametersPacket.getMaximumStepWidth() != noValue)
         setMaximumStepWidth(parametersPacket.getMaximumStepWidth());
      if (parametersPacket.getCliffBaseHeightToAvoid() != noValue)
         this.setCliffBaseHeightToAvoid(parametersPacket.getCliffBaseHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffBottoms() != noValue)
         setMinimumDistanceFromCliffBottoms(parametersPacket.getMinimumDistanceFromCliffBottoms());
      if (parametersPacket.getCliffTopHeightToAvoid() != noValue)
         this.setCliffTopHeightToAvoid(parametersPacket.getCliffTopHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffTops() != noValue)
         setMinimumDistanceFromCliffTops(parametersPacket.getMinimumDistanceFromCliffTops());
      if (parametersPacket.getBodyBoxHeight() != noValue)
         setBodyBoxHeight(parametersPacket.getBodyBoxHeight());
      if (parametersPacket.getBodyBoxDepth() != noValue)
         setBodyBoxDepth(parametersPacket.getBodyBoxDepth());
      if (parametersPacket.getBodyBoxWidth() != noValue)
         setBodyBoxWidth(parametersPacket.getBodyBoxWidth());
      if (parametersPacket.getBodyBoxBaseX() != noValue)
         setBodyBoxBaseX(parametersPacket.getBodyBoxBaseX());
      if (parametersPacket.getBodyBoxBaseY() != noValue)
         setBodyBoxBaseY(parametersPacket.getBodyBoxBaseY());
      if (parametersPacket.getBodyBoxBaseZ() != noValue)
         setBodyBoxBaseZ(parametersPacket.getBodyBoxBaseZ());
      if (parametersPacket.getMaximumSnapHeight() != noValue)
         setMaximumSnapHeight(parametersPacket.getMaximumSnapHeight());
      if (parametersPacket.getMinClearanceFromStance() != noValue)
         setMinClearanceFromStance(parametersPacket.getMinClearanceFromStance());
      if (parametersPacket.getFinalTurnProximity() != noValue)
         setFinalTurnProximity(parametersPacket.getFinalTurnProximity());
      setMaximumBranchFactor(parametersPacket.getMaximumBranchFactor());
      setEnableExpansionMask(parametersPacket.getEnableExpansionMask());

      if (parametersPacket.getAStarHeuristicsWeight() != noValue)
         setAStarHeuristicsWeight(parametersPacket.getAStarHeuristicsWeight());

      if (parametersPacket.getYawWeight() != noValue)
         setYawWeight(parametersPacket.getYawWeight());
      if (parametersPacket.getPitchWeight() != noValue)
         setPitchWeight(parametersPacket.getPitchWeight());
      if (parametersPacket.getRollWeight() != noValue)
         setRollWeight(parametersPacket.getRollWeight());
      if (parametersPacket.getForwardWeight() != noValue)
         setForwardWeight(parametersPacket.getForwardWeight());
      if (parametersPacket.getLateralWeight() != noValue)
         setLateralWeight(parametersPacket.getLateralWeight());
      if (parametersPacket.getStepUpWeight() != noValue)
         setStepUpWeight(parametersPacket.getStepUpWeight());
      if (parametersPacket.getStepDownWeight() != noValue)
         setStepDownWeight(parametersPacket.getStepDownWeight());
      if (parametersPacket.getCostPerStep() != noValue)
         setCostPerStep(parametersPacket.getCostPerStep());
      if (parametersPacket.getMaximum2dDistanceFromBoundingBoxToPenalize() != noValue)
         setMaximum2dDistanceFromBoundingBoxToPenalize(parametersPacket.getMaximum2dDistanceFromBoundingBoxToPenalize());

      if (parametersPacket.getFootholdAreaWeight() != noValue)
         setFootholdAreaWeight(parametersPacket.getFootholdAreaWeight());
      if (parametersPacket.getDistanceFromPathTolerance() != noValue)
         setDistanceFromPathTolerance(parametersPacket.getDistanceFromPathTolerance());
      if (parametersPacket.getDeltaYawFromReferenceTolerance() != noValue)
         setDeltaYawFromReferenceTolerance(parametersPacket.getDeltaYawFromReferenceTolerance());
      setEnableShinCollisionCheck(parametersPacket.getEnableShinCollisionCheck());
      if (parametersPacket.getShinLength() != noValue)
         setShinLength(parametersPacket.getShinLength());
      if (parametersPacket.getShinToeClearance() != noValue)
         setShinToeClearance(parametersPacket.getShinToeClearance());
      if (parametersPacket.getShinHeelClearance() != noValue)
         setShinHeelClearance(parametersPacket.getShinHeelClearance());
      if (parametersPacket.getShinHeightOffet() != noValue)
         setShinHeightOffset(parametersPacket.getShinHeightOffet());
   }
}