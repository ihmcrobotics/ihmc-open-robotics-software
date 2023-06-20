package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.UUID;

public class FootstepAction extends FootstepActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final FramePose3D pose = new FramePose3D();

   public FootstepAction(ROS2ControllerHelper ros2ControllerHelper, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      setReferenceFrameLibrary(referenceFrameLibrary);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex)
   {
      pose.setToZero(getReferenceFrame());
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
