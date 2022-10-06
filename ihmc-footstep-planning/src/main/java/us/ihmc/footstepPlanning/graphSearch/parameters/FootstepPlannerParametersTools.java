package us.ihmc.footstepPlanning.graphSearch.parameters;

import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;

public class FootstepPlannerParametersTools
{
   public static void toParameters(FootstepPlannerParametersPacket packet, FootstepPlannerParametersBasics parameters)
   {
      double noValue = FootstepPlannerParametersPacket.DEFAULT_NO_VALUE;
      parameters.setCheckForBodyBoxCollisions(packet.getCheckForBodyBoxCollisions());
      parameters.setCheckForPathCollisions(packet.getCheckForPathCollisions());
      parameters.setIntermediateBodyBoxChecks((int) packet.getIntermediateBodyBoxChecks());
      if (packet.getIdealFootstepWidth() != noValue)
         parameters.setIdealFootstepWidth(packet.getIdealFootstepWidth());
      if (packet.getIdealFootstepLength() != noValue)
         parameters.setIdealFootstepLength(packet.getIdealFootstepLength());
      if (packet.getIdealSideStepWidth() != noValue)
         parameters.setIdealSideStepWidth(packet.getIdealSideStepWidth());
      if (packet.getIdealBackStepLength() != noValue)
         parameters.setIdealBackStepLength(packet.getIdealBackStepLength());
      if (packet.getIdealStepLengthAtMaxStepZ() != noValue)
         parameters.setIdealStepLengthAtMaxStepZ(packet.getIdealStepLengthAtMaxStepZ());
      if (packet.getWiggleInsideDeltaTarget() != noValue)
         parameters.setWiggleInsideDeltaTarget(packet.getWiggleInsideDeltaTarget());
      if (packet.getWiggleInsideDeltaMinimum() != noValue)
         parameters.setWiggleInsideDeltaMinimum(packet.getWiggleInsideDeltaMinimum());
      if (packet.getMaximumStepReach() != noValue)
         parameters.setMaximumStepReach(packet.getMaximumStepReach());
      if (packet.getMaximumStepYaw() != noValue)
         parameters.setMaximumStepYaw(packet.getMaximumStepYaw());
      parameters.setUseReachabilityMap(packet.getUseReachabilityMap());
      if (packet.getSolutionQualityThreshold() != noValue)
         parameters.setSolutionQualityThreshold(packet.getSolutionQualityThreshold());
      if (packet.getMinimumStepWidth() != noValue)
         parameters.setMinimumStepWidth(packet.getMinimumStepWidth());
      if (packet.getMinimumStepLength() != noValue)
         parameters.setMinimumStepLength(packet.getMinimumStepLength());
      if (packet.getMinimumStepYaw() != noValue)
         parameters.setMinimumStepYaw(packet.getMinimumStepYaw());
      if (packet.getMaximumStepReachWhenSteppingUp() != noValue)
         parameters.setMaximumStepReachWhenSteppingUp(packet.getMaximumStepReachWhenSteppingUp());
      if (packet.getMaximumStepWidthWhenSteppingUp() != noValue)
         parameters.setMaximumStepWidthWhenSteppingUp(packet.getMaximumStepWidthWhenSteppingUp());
      if (packet.getMaximumStepZWhenSteppingUp() != noValue)
         parameters.setMaximumStepZWhenSteppingUp(packet.getMaximumStepZWhenSteppingUp());
      if (packet.getMaximumStepXWhenForwardAndDown() != noValue)
         parameters.setMaximumStepXWhenForwardAndDown(packet.getMaximumStepXWhenForwardAndDown());
      if (packet.getMaximumStepYWhenForwardAndDown() != noValue)
         parameters.setMaximumStepYWhenForwardAndDown(packet.getMaximumStepYWhenForwardAndDown());
      if (packet.getMaximumStepZWhenForwardAndDown() != noValue)
         parameters.setMaximumStepZWhenForwardAndDown(packet.getMaximumStepZWhenForwardAndDown());
      if (packet.getMaximumStepZ() != noValue)
         parameters.setMaximumStepZ(packet.getMaximumStepZ());
      if (packet.getMaximumSwingZ() != noValue)
         parameters.setMaximumSwingZ(packet.getMaximumSwingZ());
      if (packet.getMaximumSwingReach() != noValue)
         parameters.setMaximumSwingReach(packet.getMaximumSwingReach());
      if (packet.getMinimumStepZWhenFullyPitched() != noValue)
         parameters.setMinimumStepZWhenFullyPitched(packet.getMinimumStepZWhenFullyPitched());
      if (packet.getMaximumStepXWhenFullyPitched() != noValue)
         parameters.setMaximumStepXWhenFullyPitched(packet.getMaximumStepXWhenFullyPitched());
      if (packet.getStepYawReductionFactorAtMaxReach() != noValue)
         parameters.setStepYawReductionFactorAtMaxReach(packet.getStepYawReductionFactorAtMaxReach());
      if (packet.getMinimumFootholdPercent() != noValue)
         parameters.setMinimumFootholdPercent(packet.getMinimumFootholdPercent());
      if (packet.getMinimumSurfaceInclineRadians() != noValue)
         parameters.setMinimumSurfaceInclineRadians(packet.getMinimumSurfaceInclineRadians());
      parameters.setWiggleWhilePlanning(packet.getWiggleWhilePlanning());
      parameters.setEnableConcaveHullWiggler(packet.getEnableConcaveHullWiggler());
      if (packet.getMaximumXyWiggleDistance() != noValue)
         parameters.setMaximumXYWiggleDistance(packet.getMaximumXyWiggleDistance());
      if (packet.getMaximumYawWiggle() != noValue)
         parameters.setMaximumYawWiggle(packet.getMaximumYawWiggle());
      if (packet.getMaximumZPenetrationOnValleyRegions() != noValue)
         parameters.setMaximumZPenetrationOnValleyRegions(packet.getMaximumZPenetrationOnValleyRegions());
      if (packet.getMaximumStepWidth() != noValue)
         parameters.setMaximumStepWidth(packet.getMaximumStepWidth());
      if (packet.getCliffBaseHeightToAvoid() != noValue)
         parameters.setCliffBaseHeightToAvoid(packet.getCliffBaseHeightToAvoid());
      if (packet.getMinimumDistanceFromCliffBottoms() != noValue)
         parameters.setMinimumDistanceFromCliffBottoms(packet.getMinimumDistanceFromCliffBottoms());
      if (packet.getCliffTopHeightToAvoid() != noValue)
         parameters.setCliffTopHeightToAvoid(packet.getCliffTopHeightToAvoid());
      if (packet.getMinimumDistanceFromCliffTops() != noValue)
         parameters.setMinimumDistanceFromCliffTops(packet.getMinimumDistanceFromCliffTops());
      if (packet.getBodyBoxHeight() != noValue)
         parameters.setBodyBoxHeight(packet.getBodyBoxHeight());
      if (packet.getBodyBoxBaseZ() != noValue)
         parameters.setBodyBoxBaseZ(packet.getBodyBoxBaseZ());
      if (packet.getMaximumSnapHeight() != noValue)
         parameters.setMaximumSnapHeight(packet.getMaximumSnapHeight());
      if (packet.getMinClearanceFromStance() != noValue)
         parameters.setMinClearanceFromStance(packet.getMinClearanceFromStance());
      if (packet.getAStarHeuristicsWeight() != noValue)
         parameters.setAStarHeuristicsWeight(packet.getAStarHeuristicsWeight());

      if (packet.getYawWeight() != noValue)
         parameters.setYawWeight(packet.getYawWeight());
      if (packet.getPitchWeight() != noValue)
         parameters.setPitchWeight(packet.getPitchWeight());
      if (packet.getRollWeight() != noValue)
         parameters.setRollWeight(packet.getRollWeight());
      if (packet.getForwardWeight() != noValue)
         parameters.setForwardWeight(packet.getForwardWeight());
      if (packet.getLateralWeight() != noValue)
         parameters.setLateralWeight(packet.getLateralWeight());
      if (packet.getStepUpWeight() != noValue)
         parameters.setStepUpWeight(packet.getStepUpWeight());
      if (packet.getStepDownWeight() != noValue)
         parameters.setStepDownWeight(packet.getStepDownWeight());
      if (packet.getCostPerStep() != noValue)
         parameters.setCostPerStep(packet.getCostPerStep());

      if (packet.getFootholdAreaWeight() != noValue)
         parameters.setFootholdAreaWeight(packet.getFootholdAreaWeight());
      if (packet.getDistanceFromPathTolerance() != noValue)
         parameters.setDistanceFromPathTolerance(packet.getDistanceFromPathTolerance());
      if (packet.getDeltaYawFromReferenceTolerance() != noValue)
         parameters.setDeltaYawFromReferenceTolerance(packet.getDeltaYawFromReferenceTolerance());
      parameters.setEnableShinCollisionCheck(packet.getEnableShinCollisionCheck());
      if (packet.getShinLength() != noValue)
         parameters.setShinLength(packet.getShinLength());
      if (packet.getShinToeClearance() != noValue)
         parameters.setShinToeClearance(packet.getShinToeClearance());
      if (packet.getShinHeelClearance() != noValue)
         parameters.setShinHeelClearance(packet.getShinHeelClearance());
      if (packet.getShinHeightOffet() != noValue)
         parameters.setShinHeightOffset(packet.getShinHeightOffet());

      if (packet.getBodyBoxDepth() != noValue)
      {
         parameters.setBodyBoxDepth(packet.getBodyBoxDepth());
      }
      if (packet.getBodyBoxWidth() != noValue)
      {
         parameters.setBodyBoxWidth(packet.getBodyBoxWidth());
      }
      if (packet.getBodyBoxBaseX() != noValue)
      {
         parameters.setBodyBoxBaseX(packet.getBodyBoxBaseX());
      }
      if (packet.getBodyBoxBaseY() != noValue)
      {
         parameters.setBodyBoxBaseY(packet.getBodyBoxBaseY());
      }
      if (packet.getFinalTurnProximity() != noValue)
      {
         parameters.setFinalTurnProximity(packet.getFinalTurnProximity());
      }
      parameters.setMaximumBranchFactor(packet.getMaximumBranchFactor());
      parameters.setEnableExpansionMask(packet.getEnableExpansionMask());

      if (packet.getRmsErrorThreshold() != noValue)
         parameters.setRMSErrorThreshold(packet.getRmsErrorThreshold());
      if (packet.getRmsErrorCost() != noValue)
         parameters.setRMSErrorCost(packet.getRmsErrorCost());
      if (packet.getRmsMinErrorToPenalize() != noValue)
         parameters.setRMSMinErrorToPenalize(packet.getRmsMinErrorToPenalize());
      if (packet.getHeightMapSnapThreshold() != noValue)
         parameters.setHeightMapSnapThreshold(packet.getHeightMapSnapThreshold());
   }
}
