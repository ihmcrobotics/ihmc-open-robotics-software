package us.ihmc.behaviors.tools.affordanceTemplates;

import ihmc_common_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class AffordanceTemplateHandMovement implements AffordanceTemplateAction
{
   private final ReferenceFrame affordanceFrame;
   private final FramePose3D handPose = new FramePose3D();
   private final RigidBodyTransform endPointTransform = new RigidBodyTransform();

   public AffordanceTemplateHandMovement(String name, ReferenceFrame affordanceFrame)
   {
      this.affordanceFrame = affordanceFrame;
   }

   private void moveHand(double x, double y, double z, double yaw, double pitch, double roll, RobotSide side, double trajectoryTime, BehaviorHelper helper)
   {
      handPose.setToZero(affordanceFrame);
      handPose.getPosition().set(x, y, z);
      handPose.getOrientation().setYawPitchRoll(yaw, pitch, roll);
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      handPose.get(endPointTransform);

      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(side.toByte());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(FrameInformation.WORLD_FRAME);
      SE3TrajectoryPointMessage trajectoryPoint = handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(trajectoryTime);
      trajectoryPoint.getPosition().set(handPose.getPosition());
      trajectoryPoint.getOrientation().set(handPose.getOrientation());
      trajectoryPoint.getLinearVelocity().set(EuclidCoreTools.zeroVector3D);
      trajectoryPoint.getAngularVelocity().set(EuclidCoreTools.zeroVector3D);
      helper.publishToController(handTrajectoryMessage);
   }

   public RigidBodyTransform getEndPointTransform()
   {
      return endPointTransform;
   }
}
