package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools;

import controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket;
import controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepPlannerMessageTools
{
   public static QuadrupedFootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialBodyPose, RobotQuadrant initialStanceQuadrant,
                                                                                              FramePose3D goalPose)
   {
      return createFootstepPlanningRequestPacket(initialBodyPose, initialStanceQuadrant, goalPose, FootstepPlannerType.A_STAR);
   }

   public static QuadrupedFootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialBodyPose, RobotQuadrant initialStanceQuadrant,
                                                                                            FramePose3D goalPose, FootstepPlannerType requestedPlannerType)
   {
      QuadrupedFootstepPlanningRequestPacket message = new QuadrupedFootstepPlanningRequestPacket();
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

      message.setRequestedFootstepPlannerType(requestedPlannerType.toByte());
      return message;
   }


   public static void copyParametersToPacket(VisibilityGraphsParametersPacket packet, VisibilityGraphsParameters parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setMaxInterRegionConnectionLength(parameters.getMaxInterRegionConnectionLength());
      packet.setNormalZThresholdForAccessibleRegions(parameters.getNormalZThresholdForAccessibleRegions());
      packet.setExtrusionDistance(parameters.getObstacleExtrusionDistance());
      packet.setExtrusionDistanceIfNotTooHighToStep(parameters.getObstacleExtrusionDistanceIfNotTooHighToStep());
      packet.setTooHighToStepDistance(parameters.getTooHighToStepDistance());
      packet.setClusterResolution(parameters.getClusterResolution());
      packet.setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
      packet.setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
      packet.setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
      packet.setRegionOrthogonalAngle(parameters.getRegionOrthogonalAngle());
      packet.setSearchHostRegionEpsilon(parameters.getSearchHostRegionEpsilon());
   }
}
