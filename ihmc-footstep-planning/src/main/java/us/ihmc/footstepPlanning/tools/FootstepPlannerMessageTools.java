package us.ihmc.footstepPlanning.tools;

import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;

public class FootstepPlannerMessageTools
{
   public static void copyParametersToPacket(FootstepPlannerParametersPacket packet, DefaultFootstepPlannerParametersReadOnly parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setReferencePlanAlpha(parameters.getReferencePlanAlpha());
      packet.setCheckForBodyBoxCollisions(parameters.getCheckForBodyBoxCollisions());
      packet.setCheckForPathCollisions(parameters.getCheckForPathCollisions());
      packet.setIdealFootstepWidth(parameters.getIdealFootstepWidth());
      packet.setIdealFootstepLength(parameters.getIdealFootstepLength());
      packet.setIdealSideStepWidth(parameters.getIdealSideStepWidth());
      packet.setIdealBackStepLength(parameters.getIdealBackStepLength());
      packet.setWiggleInsideDeltaTarget(parameters.getWiggleInsideDeltaTarget());
      packet.setWiggleInsideDeltaMinimum(parameters.getWiggleInsideDeltaMinimum());
      packet.setMaxStepReach(parameters.getMaxStepReach());
      packet.setMaxStepYaw(parameters.getMaxStepYaw());
      packet.setUseReachabilityMap(parameters.getUseReachabilityMap());
      packet.setSolutionQualityThreshold(parameters.getSolutionQualityThreshold());
      packet.setMinStepWidth(parameters.getMinStepWidth());
      packet.setMinStepLength(parameters.getMinStepLength());
      packet.setMinStepYaw(parameters.getMinStepYaw());
      packet.setMaxStepZ(parameters.getMaxStepZ());
      packet.setMaxSwingZ(parameters.getMaxSwingZ());
      packet.setMaxSwingReach(parameters.getMaxSwingReach());
      packet.setMinFootholdPercent(parameters.getMinFootholdPercent());
      packet.setMinSurfaceIncline(parameters.getMinSurfaceIncline());
      packet.setWiggleWhilePlanning(parameters.getWiggleWhilePlanning());
      packet.setEnableConcaveHullWiggler(parameters.getEnableConcaveHullWiggler());
      packet.setMaximumXyWiggleDistance(parameters.getMaxXYWiggleDistance());
      packet.setMaximumYawWiggle(parameters.getMaxYawWiggle());
      packet.setMaximumZPenetrationOnValleyRegions(parameters.getMaxZPenetrationOnValleyRegions());
      packet.setMaxStepWidth(parameters.getMaxStepWidth());
      packet.setMinDistanceFromCliffBottoms(parameters.getMinDistanceFromCliffBottoms());
      packet.setCliffBottomHeightToAvoid(parameters.getCliffBottomHeightToAvoid());
      packet.setMinDistanceFromCliffTops(parameters.getMinDistanceFromCliffTops());
      packet.setCliffTopHeightToAvoid(parameters.getCliffTopHeightToAvoid());
      packet.setBodyBoxHeight(parameters.getBodyBoxHeight());
      packet.setBodyBoxDepth(parameters.getBodyBoxDepth());
      packet.setBodyBoxWidth(parameters.getBodyBoxWidth());
      packet.setBodyBoxBaseX(parameters.getBodyBoxBaseX());
      packet.setBodyBoxBaseY(parameters.getBodyBoxBaseY());
      packet.setBodyBoxBaseZ(parameters.getBodyBoxBaseZ());
      packet.setMaximumSnapHeight(parameters.getMaximumSnapHeight());
      packet.setMinClearanceFromStance(parameters.getMinClearanceFromStance());
      packet.setFinalTurnProximity(parameters.getFinalTurnProximity());
      packet.setMaxBranchFactor(parameters.getMaxBranchFactor());
      packet.setEnableExpansionMask(parameters.getEnableExpansionMask());
      packet.setEnableShinCollisionCheck(parameters.getEnableShinCollisionCheck());
      packet.setShinToeClearance(parameters.getShinToeClearance());
      packet.setShinHeelClearance(parameters.getShinHeelClearance());
      packet.setShinLength(parameters.getShinLength());
      packet.setShinHeightOffset(parameters.getShinHeightOffset());
      packet.setRmsErrorThreshold(parameters.getRMSErrorThreshold());
      packet.setRmsErrorCost(parameters.getRMSErrorCost());
      packet.setRmsMinErrorToPenalize(parameters.getRMSMinErrorToPenalize());
      packet.setHeightMapSnapThreshold(parameters.getHeightMapSnapThreshold());
      packet.setCliffHeightThreshold(parameters.getCliffHeightThreshold());
      packet.setScaledFootPolygonPercentage(parameters.getScaledFootPolygonPercentage());

      packet.setAStarHeuristicsWeight(parameters.getAStarHeuristicsWeight());
      packet.setYawWeight(parameters.getYawWeight());
      packet.setPitchWeight(parameters.getPitchWeight());
      packet.setRollWeight(parameters.getRollWeight());
      packet.setStepUpWeight(parameters.getStepUpWeight());
      packet.setStepDownWeight(parameters.getStepDownWeight());
      packet.setForwardWeight(parameters.getForwardWeight());
      packet.setLateralWeight(parameters.getLateralWeight());
      packet.setCostPerStep(parameters.getCostPerStep());
      packet.setIntermediateBodyBoxChecks(parameters.getIntermediateBodyBoxChecks());
      packet.setFootholdAreaWeight(parameters.getFootholdAreaWeight());
      packet.setDistanceFromPathTolerance(parameters.getDistanceFromPathTolerance());
      packet.setDeltaYawFromReferenceTolerance(parameters.getDeltaYawFromReferenceTolerance());
   }
}
