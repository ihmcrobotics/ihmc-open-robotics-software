package us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools;

import controller_msgs.msg.dds.PawStepPlanningRequestPacket;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawStepPlannerMessageTools
{
   public static PawStepPlanningRequestPacket createPawPlanningRequestPacket(FramePose3D initialBodyPose, RobotQuadrant initialStanceQuadrant,
                                                                         FramePose3D goalPose)
   {
      return createPawPlanningRequestPacket(initialBodyPose, initialStanceQuadrant, goalPose, PawStepPlannerType.A_STAR);
   }

   public static PawStepPlanningRequestPacket createPawPlanningRequestPacket(FramePose3D initialBodyPose, RobotQuadrant initialStanceQuadrant,
                                                                         FramePose3D goalPose, PawStepPlannerType requestedPlannerType)
   {
      PawStepPlanningRequestPacket message = new PawStepPlanningRequestPacket();
      message.setInitialStepRobotQuadrant(initialStanceQuadrant.toByte());

      FramePoint3D initialFramePoint = new FramePoint3D(initialBodyPose.getPosition());
      initialFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.getBodyPositionInWorld().set(new Point3D32(initialFramePoint));

      FrameQuaternion initialFrameOrientation = new FrameQuaternion(initialBodyPose.getOrientation());
      initialFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.getBodyOrientationInWorld().set(new Quaternion32(initialFrameOrientation));

      FramePoint3D goalFramePoint = new FramePoint3D(goalPose.getPosition());
      goalFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.getGoalPositionInWorld().set(new Point3D32(goalFramePoint));

      FrameQuaternion goalFrameOrientation = new FrameQuaternion(goalPose.getOrientation());
      goalFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.getGoalOrientationInWorld().set(new Quaternion32(goalFrameOrientation));

      message.setRequestedPawPlannerType(requestedPlannerType.toByte());
      return message;
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
      packet.setPreferredObstacleExtrusionDistance(parameters.getPreferredObstacleExtrusionDistance());
      packet.setObstacleExtrusionDistanceIfNotTooHighToStep(parameters.getObstacleExtrusionDistanceIfNotTooHighToStep());
      packet.setTooHighToStepDistance(parameters.getTooHighToStepDistance());
      packet.setClusterResolution(parameters.getClusterResolution());
      packet.setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
      packet.setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
      packet.setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
      packet.setRegionOrthogonalAngle(parameters.getRegionOrthogonalAngle());
      packet.setSearchHostRegionEpsilon(parameters.getSearchHostRegionEpsilon());
      packet.setCanEasilyStepOverHeight(parameters.getCanEasilyStepOverHeight());
      packet.setCanDuckUnderHeight(parameters.getCanDuckUnderHeight());
      packet.setLengthForLongInterRegionEdge(parameters.getLengthForLongInterRegionEdge());
      packet.setHeuristicWeight(parameters.getHeuristicWeight());
      packet.setDistanceWeight(parameters.getDistanceWeight());
      packet.setElevationWeight(parameters.getElevationWeight());
      packet.setReturnBestEffortSolution(parameters.returnBestEffortSolution());
      packet.setOccludedGoalEdgeWeight(parameters.getOccludedGoalEdgeWeight());
   }
}
