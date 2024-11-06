package us.ihmc.footstepPlanning.tools;

import toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.SwingPlannerParametersPacket;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlannerMessageTools
{
   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(RobotSide initialStanceSide,
                                                                                   Pose3DReadOnly startLeftFootPose,
                                                                                   Pose3DReadOnly startRightFootPose,
                                                                                   Pose3DReadOnly goalMidFootPose,
                                                                                   double idealStanceWidth,
                                                                                   boolean planBodyPath)
   {
      Pose3D goalLeftFootPose = new Pose3D(goalMidFootPose);
      Pose3D goalRightFootPose = new Pose3D(goalMidFootPose);
      goalLeftFootPose.appendTranslation(0.0, 0.5 * idealStanceWidth, 0.0);
      goalRightFootPose.appendTranslation(0.0, -0.5 * idealStanceWidth, 0.0);
      return createFootstepPlanningRequestPacket(initialStanceSide, startLeftFootPose, startRightFootPose, goalLeftFootPose, goalRightFootPose, planBodyPath);
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(RobotSide initialStanceSide,
                                                                                   Pose3DReadOnly startLeftFootPose,
                                                                                   Pose3DReadOnly startRightFootPose,
                                                                                   Pose3DReadOnly goalLeftFootPose,
                                                                                   Pose3DReadOnly goalRightFootPose,
                                                                                   boolean planBodyPath)
   {
      FootstepPlanningRequestPacket message = new FootstepPlanningRequestPacket();
      message.setRequestedInitialStanceSide(initialStanceSide.toByte());
      message.getStartLeftFootPose().set(startLeftFootPose);
      message.getStartRightFootPose().set(startRightFootPose);
      message.getGoalLeftFootPose().set(goalLeftFootPose);
      message.getGoalRightFootPose().set(goalRightFootPose);
      message.setPlanBodyPath(planBodyPath);

      return message;
   }

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

   public static void copyParametersToPacket(AStarBodyPathPlannerParametersPacket packet, AStarBodyPathPlannerParametersReadOnly parameters)
   {
      if (parameters == null)
      {
         return;
      }
      packet.setCheckForCollisions(parameters.getCheckForCollisions());
      packet.setComputeSurfaceNormalCost(parameters.getComputeSurfaceNormalCost());
      packet.setComputeTraversibility(parameters.getComputeTraversibility());
      packet.setPerformSmoothing(parameters.getPerformSmoothing());

      packet.setRollCostWeight(parameters.getRollCostWeight());
      packet.setRollCostDeadband(parameters.getRollCostDeadband());
      packet.setMaxPenalizedRollAngle(parameters.getMaxPenalizedRollAngle());
      packet.setSnapRadius(parameters.getSnapRadius());
      packet.setMinSnapHeightThreshold(parameters.getMinSnapHeightThreshold());
      packet.setInclineCostWeight(parameters.getInclineCostWeight());
      packet.setInclineCostDeadband(parameters.getInclineCostDeadband());
      packet.setMaxIncline(parameters.getMaxIncline());
      packet.setCollisionBoxSizeY(parameters.getCollisionBoxSizeY());
      packet.setCollisionBoxSizeX(parameters.getCollisionBoxSizeX());
      packet.setCollisionBoxGroundClearance(parameters.getCollisionBoxGroundClearance());
      packet.setTraversibilityWeight(parameters.getTraversibilityWeight());
      packet.setTraversibilityStanceWeight(parameters.getTraversibilityStanceWeight());
      packet.setTraversibilityStepWeight(parameters.getTraversibilityStepWeight());
      packet.setMinTraversibilityScore(parameters.getMinTraversibilityScore());
      packet.setMinNormalAngleToPenalizeForTraversibility(parameters.getMinNormalAngleToPenalizeForTraversibility());
      packet.setMaxNormalAngleToPenalizeForTraversibility(parameters.getMaxNormalAngleToPenalizeForTraversibility());
      packet.setTraversibilityInclineWeight(parameters.getTraversibilityInclineWeight());
      packet.setTraversibilitySearchWidth(parameters.getTraversibilitySearchWidth());
      packet.setMinOccupiedNeighborsForTraversibility(parameters.getMinOccupiedNeighborsForTraversibility());
      packet.setHalfStanceWidth(parameters.getHalfStanceWidth());
      packet.setTraversibilityHeightWindowWidth(parameters.getTraversibilityHeightWindowWidth());
      packet.setTraversibilityHeightWindowDeadband(parameters.getTraversibilityHeightWindowDeadband());
      packet.setHeightProximityForSayingWalkingOnGround(parameters.getHeightProximityForSayingWalkingOnGround());
      packet.setTraversibilityNonGroundDiscountWhenWalkingOnGround(parameters.getTraversibilityNonGroundDiscountWhenWalkingOnGround());
      packet.setSmootherCollisionWeight(parameters.getSmootherCollisionWeight());
      packet.setSmootherSmoothnessWeight(parameters.getSmootherSmoothnessWeight());
      packet.setSmootherTurnPointSmoothnessDiscount(parameters.getSmootherTurnPointSmoothnessDiscount());
      packet.setSmootherMinCurvatureToPenalize(parameters.getSmootherMinCurvatureToPenalize());
      packet.setSmootherEqualSpacingWeight(parameters.getSmootherEqualSpacingWeight());
      packet.setSmootherRollWeight(parameters.getSmootherRollWeight());
      packet.setSmootherDisplacementWeight(parameters.getSmootherDisplacementWeight());
      packet.setSmootherTraversibilityWeight(parameters.getSmootherTraversibilityWeight());
      packet.setSmootherGroundPlaneWeight(parameters.getSmootherGroundPlaneWeight());
      packet.setSmootherMinimumTraversibilityToSearchFor(parameters.getSmootherMinimumTraversibilityToSearchFor());
      packet.setSmootherTraversibilityThresholdForNoDiscount(parameters.getSmootherTraversibilityThresholdForNoDiscount());
      packet.setSmootherHillClimbGain(parameters.getSmootherHillClimbGain());
      packet.setSmootherGradientThresholdToTerminate(parameters.getSmootherGradientThresholdToTerminate());
   }

   public static void copyParametersToPacket(SwingPlannerParametersPacket packet, SwingPlannerParametersReadOnly parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setSwingHeightIfCollisionDetected(parameters.getSwingHeightIfCollisionDetected());
      packet.setMinimumSwingTime(parameters.getMinimumSwingTime());
      packet.setMaximumSwingTime(parameters.getMaximumSwingTime());
      packet.setFootStubClearance(parameters.getFootStubClearance());
      packet.setWaypointProportionShiftForStubAvoidance(parameters.getWaypointProportionShiftForStubAvoidance());
      packet.setDoInitialFastApproximation(parameters.getDoInitialFastApproximation());
      packet.setFastApproximationLessClearance(parameters.getFastApproximationLessClearance());
      packet.setMinimumSwingFootClearance(parameters.getMinimumSwingFootClearance());
      packet.setNumberOfChecksPerSwing(parameters.getNumberOfChecksPerSwing());
      packet.setMaximumNumberOfAdjustmentAttempts(parameters.getMaximumNumberOfAdjustmentAttempts());
      packet.setMaximumWaypointAdjustmentDistance(parameters.getMaximumWaypointAdjustmentDistance());
      packet.setMinimumAdjustmentIncrementDistance(parameters.getMinimumAdjustmentIncrementDistance());
      packet.setMaximumAdjustmentIncrementDistance(parameters.getMaximumAdjustmentIncrementDistance());
      packet.setAdjustmentIncrementDistanceGain(parameters.getAdjustmentIncrementDistanceGain());
      packet.setMinimumHeightAboveFloorForCollision(parameters.getMinimumHeightAboveFloorForCollision());
      packet.setAdditionalSwingTimeIfExpanded(parameters.getAdditionalSwingTimeIfExpanded());
      packet.setPercentageExtraSizeXLow(parameters.getExtraSizePercentageLow(Axis3D.X));
      packet.setPercentageExtraSizeXHigh(parameters.getExtraSizePercentageHigh(Axis3D.X));
      packet.setExtraSizeXLow(parameters.getExtraSizeLow(Axis3D.X));
      packet.setExtraSizeXHigh(parameters.getExtraSizeHigh(Axis3D.X));
      packet.setPercentageExtraSizeYLow(parameters.getExtraSizePercentageLow(Axis3D.Y));
      packet.setPercentageExtraSizeYHigh(parameters.getExtraSizePercentageHigh(Axis3D.Y));
      packet.setExtraSizeYLow(parameters.getExtraSizeLow(Axis3D.Y));
      packet.setExtraSizeYHigh(parameters.getExtraSizeHigh(Axis3D.Y));
      packet.setPercentageExtraSizeZLow(parameters.getExtraSizePercentageLow(Axis3D.Z));
      packet.setPercentageExtraSizeZHigh(parameters.getExtraSizePercentageHigh(Axis3D.Z));
      packet.setExtraSizeZLow(parameters.getExtraSizeLow(Axis3D.Z));
      packet.setExtraSizeZHigh(parameters.getExtraSizeHigh(Axis3D.Z));
      packet.setPercentageMaxDisplacementLow(parameters.getPercentageLowMaxDisplacement());
      packet.setPercentageMaxDisplacementHigh(parameters.getPercentageHighMaxDisplacement());
      packet.setMaxDisplacementLow(parameters.getMaxDisplacementLow());
      packet.setMaxDisplacementHigh(parameters.getMaxDisplacementHigh());
      packet.setMotionCorrelationAlpha(parameters.getMotionCorrelationAlpha());
      packet.setAllowLateralMotion(parameters.getAllowLateralMotion());
      packet.setMinXyTranslationToPlanSwing(parameters.getMinXYTranslationToPlanSwing());
   }

   public static void copyParametersToPacket(VisibilityGraphsParametersPacket packet, VisibilityGraphsParametersReadOnly parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setMaxInterRegionConnectionLength(parameters.getMaxInterRegionConnectionLength());
      packet.setNormalZThresholdForAccessibleRegions(parameters.getNormalZThresholdForAccessibleRegions());
      packet.setNavigableExtrusionDistance(parameters.getNavigableExtrusionDistance());
      packet.setObstacleExtrusionDistance(parameters.getObstacleExtrusionDistance());
      packet.setObstacleExtrusionDistanceIfNotTooHighToStep(parameters.getObstacleExtrusionDistanceIfNotTooHighToStep());
      packet.setTooHighToStepDistance(parameters.getTooHighToStepDistance());
      packet.setHeightForMaxAvoidance(parameters.getHeightForMaxAvoidance());
      packet.setClusterResolution(parameters.getClusterResolution());
      packet.setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
      packet.setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
      packet.setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
      packet.setRegionOrthogonalAngle(parameters.getRegionOrthogonalAngle());
      packet.setSearchHostRegionEpsilon(parameters.getSearchHostRegionEpsilon());
      packet.setCanEasilyStepOverHeight(parameters.getCanEasilyStepOverHeight());
      packet.setCanDuckUnderHeight(parameters.getCanDuckUnderHeight());
      packet.setLengthForLongInterRegionEdge(parameters.getLengthForLongInterRegionEdge());
      packet.setPerformPostProcessingNodeShifting(parameters.getPerformPostProcessingNodeShifting());
      packet.setIntroduceMidpointsInPostProcessing(parameters.getIntroduceMidpointsInPostProcessing());
      packet.setComputeOrientationsToAvoidObstacles(parameters.getComputeOrientationsToAvoidObstacles());
      packet.setHeuristicWeight(parameters.getHeuristicWeight());
      packet.setDistanceWeight(parameters.getDistanceWeight());
      packet.setElevationWeight(parameters.getElevationWeight());
      packet.setReturnBestEffortSolution(parameters.returnBestEffortSolution());
      packet.setOccludedGoalEdgeWeight(parameters.getOccludedGoalEdgeWeight());
      packet.setWeightForInterRegionEdge(parameters.getWeightForInterRegionEdge());
      packet.setOptimizeForNarrowPassage(parameters.getOptimizeForNarrowPassage());
   }
}
