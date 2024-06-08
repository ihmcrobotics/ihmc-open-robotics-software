package us.ihmc.footstepPlanning.graphSearch.parameters;

import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface DefaultFootstepPlannerParametersBasics extends DefaultFootstepPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void set(DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      setAll(footstepPlannerParameters.getAll());
   }

   default void setAStarHeuristicsWeight(double aStarHeuristicsWeight)
   {
      set(DefaultFootstepPlannerParameters.astarHeuristicsWeight, aStarHeuristicsWeight);
   }

   default void setMaxBranchFactor(int maximumBranchFactor)
   {
      set(DefaultFootstepPlannerParameters.maxBranchFactor, maximumBranchFactor);
   }

   default void setEnableExpansionMask(boolean enableExpansionMask)
   {
      set(DefaultFootstepPlannerParameters.enableExpansionMask, enableExpansionMask);
   }

   default void setUseReachabilityMap(boolean useReachabilityMap)
   {
      set(DefaultFootstepPlannerParameters.useReachabilityMap, useReachabilityMap);
   }

   default void setSolutionQualityThreshold(double solutionQualityThreshold)
   {
      set(DefaultFootstepPlannerParameters.solutionQualityThreshold, solutionQualityThreshold);
   }

   default void setIdealFootstepWidth(double idealFootstepWidth)
   {
      set(DefaultFootstepPlannerParameters.idealFootstepWidth, idealFootstepWidth);
   }

   default void setIdealFootstepLength(double idealFootstepLength)
   {
      set(DefaultFootstepPlannerParameters.idealFootstepLength, idealFootstepLength);
   }

   default void setIdealSideStepWidth(double idealSideStepWidth)
   {
      set(DefaultFootstepPlannerParameters.idealSideStepWidth, idealSideStepWidth);
   }

   default void setIdealBackStepLength(double idealBackStepLength)
   {
      set(DefaultFootstepPlannerParameters.idealBackStepLength, idealBackStepLength);
   }

   default void setIdealStepLengthAtMaxStepZ(double idealStepLengthAtMaxStepZ)
   {
      set(DefaultFootstepPlannerParameters.idealStepLengthAtMaxStepZ, idealStepLengthAtMaxStepZ);
   }

   default void setMinStepWidth(double minimumStepWidth)
   {
      set(DefaultFootstepPlannerParameters.minStepWidth, minimumStepWidth);
   }

   default void setMinStepLength(double minimumStepLength)
   {
      set(DefaultFootstepPlannerParameters.minStepLength, minimumStepLength);
   }

   default void setMinSurfaceInclineRadians(double surfaceInclineRadians)
   {
      set(DefaultFootstepPlannerParameters.minSurfaceIncline, surfaceInclineRadians);
   }

   default void setMinStepYaw(double yaw)
   {
      set(DefaultFootstepPlannerParameters.minStepYaw, yaw);
   }

   default void setMinStepZWhenFullyPitched(double stepZ)
   {
      set(DefaultFootstepPlannerParameters.minStepZWhenFullyPitched, stepZ);
   }

   default void setMinFootholdPercent(double footholdPercent)
   {
      set(DefaultFootstepPlannerParameters.minFootholdPercent, footholdPercent);
   }

   default void setMinClearanceFromStance(double clearance)
   {
      set(DefaultFootstepPlannerParameters.minClearanceFromStance, clearance);
   }

   default void setMaxStepWidth(double width)
   {
      set(DefaultFootstepPlannerParameters.maxStepWidth, width);
   }

   default void setMaxStepReach(double reach)
   {
      set(DefaultFootstepPlannerParameters.maxStepReach, reach);
   }

   default void setMaxStepYaw(double yaw)
   {
      set(DefaultFootstepPlannerParameters.maxStepYaw, yaw);
   }

   default void setMaxStepXWhenFullyPitched(double stepX)
   {
      set(DefaultFootstepPlannerParameters.maxStepXWhenFullyPitched, stepX);
   }

   default void setMaxStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      set(DefaultFootstepPlannerParameters.maxStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }

   default void setMaxStepYWhenForwardAndDown(double maximumStepYWhenForwardAndDown)
   {
      set(DefaultFootstepPlannerParameters.maxStepYWhenForwardAndDown, maximumStepYWhenForwardAndDown);
   }

   default void setMaxStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      set(DefaultFootstepPlannerParameters.maxStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   default void setMaxStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      set(DefaultFootstepPlannerParameters.maxStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   default void setMaxStepWidthWhenSteppingUp(double maximumStepWidthWhenSteppingUp)
   {
      set(DefaultFootstepPlannerParameters.maxStepWidthWhenSteppingUp, maximumStepWidthWhenSteppingUp);
   }

   default void setMaxStepZWhenSteppingUp(double maxStepZ)
   {
      set(DefaultFootstepPlannerParameters.maxStepZWhenSteppingUp, maxStepZ);
   }

   default void setMaxStepZ(double maxStepZ)
   {
      set(DefaultFootstepPlannerParameters.maxStepZ, maxStepZ);
   }

   default void setMaxSwingZ(double maxSwingZ)
   {
      set(DefaultFootstepPlannerParameters.maxSwingZ, maxSwingZ);
   }

   default void setMaxSwingReach(double maxSwingReach)
   {
      set(DefaultFootstepPlannerParameters.maxSwingReach, maxSwingReach);
   }

   default void setStepYawReductionFactorAtMaxReach(double factor)
   {
      set(DefaultFootstepPlannerParameters.stepYawReductionFactorAtMaxReach, factor);
   }

   default void setYawWeight(double yawWeight)
   {
      set(DefaultFootstepPlannerParameters.yawWeight, yawWeight);
   }

   default void setForwardWeight(double forwardWeight)
   {
      set(DefaultFootstepPlannerParameters.forwardWeight, forwardWeight);
   }

   default void setLateralWeight(double lateralWeight)
   {
      set(DefaultFootstepPlannerParameters.lateralWeight, lateralWeight);
   }

   default void setCostPerStep(double costPerStep)
   {
      set(DefaultFootstepPlannerParameters.costPerStep, costPerStep);
   }

   default void setStepUpWeight(double stepUpWeight)
   {
      set(DefaultFootstepPlannerParameters.stepUpWeight, stepUpWeight);
   }

   default void setStepDownWeight(double stepDownWeight)
   {
      set(DefaultFootstepPlannerParameters.stepDownWeight, stepDownWeight);
   }

   default void setRollWeight(double rollWeight)
   {
      set(DefaultFootstepPlannerParameters.rollWeight, rollWeight);
   }

   default void setPitchWeight(double pitchWeight)
   {
      set(DefaultFootstepPlannerParameters.pitchWeight, pitchWeight);
   }

   default void setFootholdAreaWeight(double footholdAreaWeight)
   {
      set(DefaultFootstepPlannerParameters.footholdAreaWeight, footholdAreaWeight);
   }

   default void setRMSErrorThreshold(double rmsErrorThreshold)
   {
      set(DefaultFootstepPlannerParameters.rmsErrorThreshold, rmsErrorThreshold);
   }

   default void setRMSErrorCost(double rmsErrorCost)
   {
      set(DefaultFootstepPlannerParameters.rmsErrorCost, rmsErrorCost);
   }

   default void setRMSMinErrorToPenalize(double rmsMinErrorToPenalize)
   {
      set(DefaultFootstepPlannerParameters.rmsMinErrorToPenalize, rmsMinErrorToPenalize);
   }

   default void setHeightMapSnapThreshold(double heightMapSnapThreshold)
   {
      set(DefaultFootstepPlannerParameters.heightMapSnapThreshold, heightMapSnapThreshold);
   }

   default void setReferencePlanAlpha(double referencePlanAlpha)
   {
      set(DefaultFootstepPlannerParameters.referencePlanAlpha, referencePlanAlpha);
   }

   default void setWiggleInsideDeltaTarget(double wiggleInsideDeltaTarget)
   {
      set(DefaultFootstepPlannerParameters.wiggleInsideDeltaTarget, wiggleInsideDeltaTarget);
   }

   default void setWiggleInsideDeltaMinimum(double wiggleInsideDeltaMinimum)
   {
      set(DefaultFootstepPlannerParameters.wiggleInsideDeltaMinimum, wiggleInsideDeltaMinimum);
   }

   default void setEnableConcaveHullWiggler(boolean enableConcaveHullWiggler)
   {
      set(DefaultFootstepPlannerParameters.enableConcaveHullWiggler, enableConcaveHullWiggler);
   }

   default void setWiggleWhilePlanning(boolean wiggleWhilePlanning)
   {
      set(DefaultFootstepPlannerParameters.wiggleWhilePlanning, wiggleWhilePlanning);
   }

   default void setMaxXYWiggleDistance(double wiggleDistance)
   {
      set(DefaultFootstepPlannerParameters.maxXYWiggleDistance, wiggleDistance);
   }

   default void setMaxYawWiggle(double wiggleDistance)
   {
      set(DefaultFootstepPlannerParameters.maxYawWiggle, wiggleDistance);
   }

   default void setMaxZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      set(DefaultFootstepPlannerParameters.maxZPenetrationOnValleyRegions, maximumZPenetrationOnValleyRegions);
   }

   default void setMaximumSnapHeight(double maximumSnapHeight)
   {
      set(DefaultFootstepPlannerParameters.maximumSnapHeight, maximumSnapHeight);
   }

   default void setFinalTurnProximity(double finalTurnProximity)
   {
      set(DefaultFootstepPlannerParameters.finalTurnProximity, finalTurnProximity);
   }

   default void setDistanceFromPathTolerance(double tolerance)
   {
      set(DefaultFootstepPlannerParameters.distanceFromPathTolerance, tolerance);
   }

   default void setDeltaYawFromReferenceTolerance(double tolerance)
   {
      set(DefaultFootstepPlannerParameters.deltaYawFromReferenceTolerance, tolerance);
   }

   default void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      set(DefaultFootstepPlannerParameters.checkForBodyBoxCollisions, checkForBodyBoxCollisions);
   }

   default void setBodyBoxWidth(double bodyBoxWidth)
   {
      set(DefaultFootstepPlannerParameters.bodyBoxWidth, bodyBoxWidth);
   }

   default void setBodyBoxHeight(double bodyBoxHeight)
   {
      set(DefaultFootstepPlannerParameters.bodyBoxHeight, bodyBoxHeight);
   }

   default void setBodyBoxDepth(double bodyBoxDepth)
   {
      set(DefaultFootstepPlannerParameters.bodyBoxDepth, bodyBoxDepth);
   }

   default void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      set(DefaultFootstepPlannerParameters.bodyBoxBaseX, bodyBoxBaseX);
   }

   default void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      set(DefaultFootstepPlannerParameters.bodyBoxBaseY, bodyBoxBaseY);
   }

   default void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      set(DefaultFootstepPlannerParameters.bodyBoxBaseZ, bodyBoxBaseZ);
   }

   default void setIntermediateBodyBoxChecks(int intermediateBodyBoxChecks)
   {
      set(DefaultFootstepPlannerParameters.intermediateBodyBoxChecks, intermediateBodyBoxChecks);
   }

   default void setEnableShinCollisionCheck(boolean enableShinCollisionCheck)
   {
      set(DefaultFootstepPlannerParameters.enableShinCollisionCheck, enableShinCollisionCheck);
   }

   default void setShinToeClearance(double shinToeClearance)
   {
      set(DefaultFootstepPlannerParameters.shinToeClearance, shinToeClearance);
   }

   default void setShinHeelClearance(double shinHeelClearance)
   {
      set(DefaultFootstepPlannerParameters.shinHeelClearance, shinHeelClearance);
   }

   default void setShinLength(double shinLength)
   {
      set(DefaultFootstepPlannerParameters.shinLength, shinLength);
   }

   default void setShinHeightOffset(double shinHeightOffet)
   {
      set(DefaultFootstepPlannerParameters.shinHeightOffset, shinHeightOffet);
   }

   default void setCheckForPathCollisions(boolean checkForPathCollisions)
   {
      set(DefaultFootstepPlannerParameters.checkForPathCollisions, checkForPathCollisions);
   }

   default void setCliffBaseHeightToAvoid(double height)
   {
      set(DefaultFootstepPlannerParameters.cliffBottomHeightToAvoid, height);
   }

   default void setMinDistanceFromCliffBottoms(double distance)
   {
      set(DefaultFootstepPlannerParameters.minDistanceFromCliffBottoms, distance);
   }

   default void setCliffTopHeightToAvoid(double height)
   {
      set(DefaultFootstepPlannerParameters.cliffTopHeightToAvoid, height);
   }

   default void setMinDistanceFromCliffTops(double distance)
   {
      set(DefaultFootstepPlannerParameters.minDistanceFromCliffTops, distance);
   }

   default void setScaledFootPolygonPercentage(double scaledFootPolygonPercentage)
   {
      set(DefaultFootstepPlannerParameters.scaledFootPolygonPercentage, scaledFootPolygonPercentage);
   }

   default void setCliffHeightThreshold(double cliffHeightThreshold)
   {
      set(DefaultFootstepPlannerParameters.cliffHeightThreshold, cliffHeightThreshold);
   }

   default void set(FootstepPlannerParametersPacket parametersPacket)
   {
      double noValue = FootstepPlannerParametersPacket.DEFAULT_NO_VALUE;
      setCheckForBodyBoxCollisions(parametersPacket.getCheckForBodyBoxCollisions());
      setCheckForPathCollisions(parametersPacket.getCheckForPathCollisions());
      setIntermediateBodyBoxChecks((int) parametersPacket.getIntermediateBodyBoxChecks());
      if (parametersPacket.getReferencePlanAlpha() != noValue)
         setReferencePlanAlpha(parametersPacket.getReferencePlanAlpha());
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
         setMaxStepReach(parametersPacket.getMaximumStepReach());
      if (parametersPacket.getMaximumStepYaw() != noValue)
         setMaxStepYaw(parametersPacket.getMaximumStepYaw());
      setUseReachabilityMap(parametersPacket.getUseReachabilityMap());
      if (parametersPacket.getSolutionQualityThreshold() != noValue)
         setSolutionQualityThreshold(parametersPacket.getSolutionQualityThreshold());
      if (parametersPacket.getMinimumStepWidth() != noValue)
         setMinStepWidth(parametersPacket.getMinimumStepWidth());
      if (parametersPacket.getMinimumStepLength() != noValue)
         setMinStepLength(parametersPacket.getMinimumStepLength());
      if (parametersPacket.getMinimumStepYaw() != noValue)
         setMinStepYaw(parametersPacket.getMinimumStepYaw());
      if (parametersPacket.getMaximumStepReachWhenSteppingUp() != noValue)
         setMaxStepReachWhenSteppingUp(parametersPacket.getMaximumStepReachWhenSteppingUp());
      if (parametersPacket.getMaximumStepWidthWhenSteppingUp() != noValue)
         setMaxStepWidthWhenSteppingUp(parametersPacket.getMaximumStepWidthWhenSteppingUp());
      if (parametersPacket.getMaximumStepZWhenSteppingUp() != noValue)
         setMaxStepZWhenSteppingUp(parametersPacket.getMaximumStepZWhenSteppingUp());
      if (parametersPacket.getMaximumStepXWhenForwardAndDown() != noValue)
         setMaxStepXWhenForwardAndDown(parametersPacket.getMaximumStepXWhenForwardAndDown());
      if (parametersPacket.getMaximumStepYWhenForwardAndDown() != noValue)
         setMaxStepYWhenForwardAndDown(parametersPacket.getMaximumStepYWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZWhenForwardAndDown() != noValue)
         setMaxStepZWhenForwardAndDown(parametersPacket.getMaximumStepZWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZ() != noValue)
         setMaxStepZ(parametersPacket.getMaximumStepZ());
      if (parametersPacket.getMaximumSwingZ() != noValue)
         setMaxSwingZ(parametersPacket.getMaximumSwingZ());
      if (parametersPacket.getMaximumSwingReach() != noValue)
         setMaxSwingReach(parametersPacket.getMaximumSwingReach());
      if (parametersPacket.getMinimumStepZWhenFullyPitched() != noValue)
         setMinStepZWhenFullyPitched(parametersPacket.getMinimumStepZWhenFullyPitched());
      if (parametersPacket.getMaximumStepXWhenFullyPitched() != noValue)
         setMaxStepXWhenFullyPitched(parametersPacket.getMaximumStepXWhenFullyPitched());
      if (parametersPacket.getStepYawReductionFactorAtMaxReach() != noValue)
         setStepYawReductionFactorAtMaxReach(parametersPacket.getStepYawReductionFactorAtMaxReach());
      if (parametersPacket.getMinimumFootholdPercent() != noValue)
         setMinFootholdPercent(parametersPacket.getMinimumFootholdPercent());
      if (parametersPacket.getMinimumSurfaceInclineRadians() != noValue)
         setMinSurfaceInclineRadians(parametersPacket.getMinimumSurfaceInclineRadians());
      setWiggleWhilePlanning(parametersPacket.getWiggleWhilePlanning());
      setEnableConcaveHullWiggler(parametersPacket.getEnableConcaveHullWiggler());
      if (parametersPacket.getMaximumXyWiggleDistance() != noValue)
         setMaxXYWiggleDistance(parametersPacket.getMaximumXyWiggleDistance());
      if (parametersPacket.getMaximumYawWiggle() != noValue)
         setMaxYawWiggle(parametersPacket.getMaximumYawWiggle());
      if (parametersPacket.getMaximumZPenetrationOnValleyRegions() != noValue)
         setMaxZPenetrationOnValleyRegions(parametersPacket.getMaximumZPenetrationOnValleyRegions());
      if (parametersPacket.getMaximumStepWidth() != noValue)
         setMaxStepWidth(parametersPacket.getMaximumStepWidth());
      if (parametersPacket.getCliffBaseHeightToAvoid() != noValue)
         this.setCliffBaseHeightToAvoid(parametersPacket.getCliffBaseHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffBottoms() != noValue)
         setMinDistanceFromCliffBottoms(parametersPacket.getMinimumDistanceFromCliffBottoms());
      if (parametersPacket.getCliffTopHeightToAvoid() != noValue)
         this.setCliffTopHeightToAvoid(parametersPacket.getCliffTopHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffTops() != noValue)
         setMinDistanceFromCliffTops(parametersPacket.getMinimumDistanceFromCliffTops());
      if (parametersPacket.getBodyBoxHeight() != noValue)
         setBodyBoxHeight(parametersPacket.getBodyBoxHeight());
      if (parametersPacket.getBodyBoxBaseZ() != noValue)
         setBodyBoxBaseZ(parametersPacket.getBodyBoxBaseZ());
      if (parametersPacket.getMaximumSnapHeight() != noValue)
         setMaximumSnapHeight(parametersPacket.getMaximumSnapHeight());
      if (parametersPacket.getMinClearanceFromStance() != noValue)
         setMinClearanceFromStance(parametersPacket.getMinClearanceFromStance());
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

      if (parametersPacket.getBodyBoxDepth() != noValue)
      {
         setBodyBoxDepth(parametersPacket.getBodyBoxDepth());
      }
      if (parametersPacket.getBodyBoxWidth() != noValue)
      {
         setBodyBoxWidth(parametersPacket.getBodyBoxWidth());
      }
      if (parametersPacket.getBodyBoxBaseX() != noValue)
      {
         setBodyBoxBaseX(parametersPacket.getBodyBoxBaseX());
      }
      if (parametersPacket.getBodyBoxBaseY() != noValue)
      {
         setBodyBoxBaseY(parametersPacket.getBodyBoxBaseY());
      }
      if (parametersPacket.getFinalTurnProximity() != noValue)
      {
         setFinalTurnProximity(parametersPacket.getFinalTurnProximity());
      }
      setMaxBranchFactor(parametersPacket.getMaximumBranchFactor());
      setEnableExpansionMask(parametersPacket.getEnableExpansionMask());

      if (parametersPacket.getRmsErrorThreshold() != noValue)
         setRMSErrorThreshold(parametersPacket.getRmsErrorThreshold());
      if (parametersPacket.getRmsErrorCost() != noValue)
         setRMSErrorCost(parametersPacket.getRmsErrorCost());
      if (parametersPacket.getRmsMinErrorToPenalize() != noValue)
         setRMSMinErrorToPenalize(parametersPacket.getRmsMinErrorToPenalize());
      if (parametersPacket.getHeightMapSnapThreshold() != noValue)
         setHeightMapSnapThreshold(parametersPacket.getHeightMapSnapThreshold());

      if (parametersPacket.getCliffHeightThreshold() != noValue)
         setCliffHeightThreshold(parametersPacket.getCliffHeightThreshold());
      if (parametersPacket.getScaledFootPolygonPercentage() != noValue)
         setScaledFootPolygonPercentage(parametersPacket.getScaledFootPolygonPercentage());
   }
}