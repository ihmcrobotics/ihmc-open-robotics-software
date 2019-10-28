package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ros2.Ros2Node;

import java.util.function.Function;

public class RemoteSyncedHumanoidRobotState extends RemoteSyncedRobotModel
{
   private final HumanoidRobotState humanoidRobotState;
   private final FramePose3D temporaryPoseForQuickReading = new FramePose3D();

   public RemoteSyncedHumanoidRobotState(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      super(robotModel, ros2Node);

      humanoidRobotState = new HumanoidRobotState(fullRobotModel, robotConfigurationData);
   }

   public HumanoidRobotState pollHumanoidRobotState()
   {
      pollFullRobotModel();

      humanoidRobotState.updateFrames();

      return humanoidRobotState;
   }

   public HumanoidRobotState getHumanoidRobotState()
   {
      return humanoidRobotState;
   }

   public FramePose3DReadOnly quickPollPoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      temporaryPoseForQuickReading.setFromReferenceFrame(frameSelector.apply(pollHumanoidRobotState()));
      return temporaryPoseForQuickReading;
   }
}
