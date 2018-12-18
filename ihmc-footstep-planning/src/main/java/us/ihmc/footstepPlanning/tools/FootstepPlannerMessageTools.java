package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlannerMessageTools
{
   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose)
   {
      return createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose, FootstepPlannerType.PLANAR_REGION_BIPEDAL);
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose, FootstepPlannerType requestedPlannerType)
   {
      FootstepPlanningRequestPacket message = new FootstepPlanningRequestPacket();
      message.setInitialStanceRobotSide(initialStanceSide.toByte());

      FramePoint3D initialFramePoint = new FramePoint3D(initialStanceFootPose.getPosition());
      initialFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.getStanceFootPositionInWorld().set(new Point3D32(initialFramePoint));

      FrameQuaternion initialFrameOrientation = new FrameQuaternion(initialStanceFootPose.getOrientation());
      initialFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.getStanceFootOrientationInWorld().set(new Quaternion32(initialFrameOrientation));

      FramePoint3D goalFramePoint = new FramePoint3D(goalPose.getPosition());
      goalFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.getGoalPositionInWorld().set(new Point3D32(goalFramePoint));

      FrameQuaternion goalFrameOrientation = new FrameQuaternion(goalPose.getOrientation());
      goalFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.getGoalOrientationInWorld().set(new Quaternion32(goalFrameOrientation));

      message.setRequestedFootstepPlannerType(requestedPlannerType.toByte());
      return message;
   }

}
