package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import ihmc_common_msgs.msg.dds.FrameInformation;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class HandPoseAction extends HandPoseActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private ReferenceFrame parentReferenceFrame;
   private final FramePose3D pose = new FramePose3D();

   public HandPoseAction(ROS2ControllerHelper ros2ControllerHelper, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void update()
   {
      parentReferenceFrame = referenceFrameLibrary.findFrameByName(getParentFrameName());

      pose.setIncludingFrame(parentReferenceFrame, getTransformToParent());
      pose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   @Override
   public void executeAction()
   {
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(getSide().toByte());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(FrameInformation.WORLD_FRAME);
      SE3TrajectoryPointMessage trajectoryPoint = handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(getTrajectoryDuration());
      trajectoryPoint.getPosition().set(pose.getPosition());
      trajectoryPoint.getOrientation().set(pose.getOrientation());
      trajectoryPoint.getLinearVelocity().set(EuclidCoreTools.zeroVector3D);
      trajectoryPoint.getAngularVelocity().set(EuclidCoreTools.zeroVector3D);
      ros2ControllerHelper.publishToController(handTrajectoryMessage);
   }
}
