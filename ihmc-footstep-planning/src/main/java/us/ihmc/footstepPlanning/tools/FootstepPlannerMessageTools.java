package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
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

   public static void copyParametersToPacket(FootstepPlannerParametersPacket packet, FootstepPlannerParametersReadOnly parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setCheckForBodyBoxCollisions(parameters.checkForBodyBoxCollisions());
      packet.setCheckForPathCollisions(parameters.checkForPathCollisions());
      packet.setIdealFootstepWidth(parameters.getIdealFootstepWidth());
      packet.setIdealFootstepLength(parameters.getIdealFootstepLength());
      packet.setWiggleInsideDelta(parameters.getWiggleInsideDelta());
      packet.setMaximumStepReach(parameters.getMaximumStepReach());
      packet.setMaximumStepYaw(parameters.getMaximumStepYaw());
      packet.setMinimumStepWidth(parameters.getMinimumStepWidth());
      packet.setMinimumStepLength(parameters.getMinimumStepLength());
      packet.setMinimumStepYaw(parameters.getMinimumStepYaw());
      packet.setMaximumStepReachWhenSteppingUp(parameters.getMaximumStepReachWhenSteppingUp());
      packet.setMaximumStepWidthWhenSteppingUp(parameters.getMaximumStepWidthWhenSteppingUp());
      packet.setMaximumStepZWhenSteppingUp(parameters.getMaximumStepZWhenSteppingUp());
      packet.setMaximumStepXWhenForwardAndDown(parameters.getMaximumStepXWhenForwardAndDown());
      packet.setMaximumStepYWhenForwardAndDown(parameters.getMaximumStepYWhenForwardAndDown());
      packet.setMaximumStepZWhenForwardAndDown(parameters.getMaximumStepZWhenForwardAndDown());
      packet.setMaximumStepZ(parameters.getMaximumStepZ());
      packet.setMinimumStepZWhenFullyPitched(parameters.getMinimumStepZWhenFullyPitched());
      packet.setMaximumStepXWhenFullyPitched(parameters.getMaximumStepXWhenFullyPitched());
      packet.setStepYawReductionFactorAtMaxReach(parameters.getStepYawReductionFactorAtMaxReach());
      packet.setTranslationScaleFromGrandparentNode(parameters.getTranslationScaleFromGrandparentNode());
      packet.setMinimumFootholdPercent(parameters.getMinimumFootholdPercent());
      packet.setMinimumSurfaceInclineRadians(parameters.getMinimumSurfaceInclineRadians());
      packet.setWiggleIntoConvexHullOfPlanarRegions(parameters.getWiggleIntoConvexHullOfPlanarRegions());
      packet.setMaximumXyWiggleDistance(parameters.getMaximumXYWiggleDistance());
      packet.setMaximumYawWiggle(parameters.getMaximumYawWiggle());
      packet.setMaximumZPenetrationOnValleyRegions(parameters.getMaximumZPenetrationOnValleyRegions());
      packet.setMaximumStepWidth(parameters.getMaximumStepWidth());
      packet.setMinimumDistanceFromCliffBottoms(parameters.getMinimumDistanceFromCliffBottoms());
      packet.setCliffHeightToAvoid(parameters.getCliffHeightToAvoid());
      packet.setReturnBestEffortPlan(parameters.getReturnBestEffortPlan());
      packet.setMinimumStepsForBestEffortPlan(parameters.getMinimumStepsForBestEffortPlan());
      packet.setBodyBoxHeight(parameters.getBodyBoxHeight());
      packet.setBodyBoxDepth(parameters.getBodyBoxDepth());
      packet.setBodyBoxWidth(parameters.getBodyBoxWidth());
      packet.setBodyBoxBaseX(parameters.getBodyBoxBaseX());
      packet.setBodyBoxBaseY(parameters.getBodyBoxBaseY());
      packet.setBodyBoxBaseZ(parameters.getBodyBoxBaseZ());
      packet.setMinXClearanceFromStance(parameters.getMinXClearanceFromStance());
      packet.setMinYClearanceFromStance(parameters.getMinYClearanceFromStance());
      packet.setFinalTurnProximity(parameters.getFinalTurnProximity());
      packet.setFinalTurnBodyPathProximity(parameters.getFinalTurnBodyPathProximity());
      packet.setFinalTurnProximityBlendFactor(parameters.getFinalTurnProximityBlendFactor());

      packet.setUseQuadraticDistanceCost(parameters.useQuadraticDistanceCost());
      packet.setUseQuadraticHeightCost(parameters.useQuadraticHeightCost());

      packet.setAStarHeuristicsWeight(parameters.getAStarHeuristicsWeight().getValue());
      packet.setVisGraphWithAStarHeuristicsWeight(parameters.getVisGraphWithAStarHeuristicsWeight().getValue());
      packet.setDepthFirstHeuristicsWeight(parameters.getDepthFirstHeuristicsWeight().getValue());
      packet.setBodyPathBasedHeuristicsWeight(parameters.getBodyPathBasedHeuristicsWeight().getValue());

      packet.setYawWeight(parameters.getYawWeight());
      packet.setPitchWeight(parameters.getPitchWeight());
      packet.setRollWeight(parameters.getRollWeight());
      packet.setStepUpWeight(parameters.getStepUpWeight());
      packet.setStepDownWeight(parameters.getStepDownWeight());
      packet.setForwardWeight(parameters.getForwardWeight());
      packet.setLateralWeight(parameters.getLateralWeight());
      packet.setCostPerStep(parameters.getCostPerStep());
      packet.setBoundingBoxCost(parameters.getBoundingBoxCost());
      packet.setNumberOfBoundingBoxChecks(parameters.getNumberOfBoundingBoxChecks());
      packet.setMaximum2dDistanceFromBoundingBoxToPenalize(parameters.getMaximum2dDistanceFromBoundingBoxToPenalize());
      packet.setLongStepWeight(parameters.getLongStepWeight());
      packet.setFootholdAreaWeight(parameters.getFootholdAreaWeight());
      packet.setBodyPathViolationWeight(parameters.getBodyPathViolationWeight());
      packet.setDistanceFromPathTolerance(parameters.getDistanceFromPathTolerance());
      packet.setDeltaYawFromReferenceTolerance(parameters.getDeltaYawFromReferenceTolerance());
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
      packet.setPreferredNavigableExtrusionDistance(parameters.getPreferredNavigableExtrusionDistance());
      packet.setPreferredObstacleExtrusionDistance(parameters.getPreferredObstacleExtrusionDistance());
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
      packet.setWeightForNonPreferredEdge(parameters.getWeightForNonPreferredEdge());
      packet.setIncludePreferredExtrusions(parameters.includePreferredExtrusions());
   }
}
