package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;

import java.util.UUID;

public class FootstepAction extends FootstepActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private ReferenceFrame parentReferenceFrame;
   private final FramePose3D pose = new FramePose3D();

   public FootstepAction(ROS2ControllerHelper ros2ControllerHelper, ROS2SyncedRobotModel syncedRobot, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
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
      double swingDuration = 1.2;
      double transferDuration = 0.8;
      FootstepPlan footstepPlan = new FootstepPlan();
      footstepPlan.addFootstep(getSide(), pose);
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
                                                                                                                    swingDuration,
                                                                                                                    transferDuration);
      footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      ros2ControllerHelper.publishToController(footstepDataListMessage);
   }
}
