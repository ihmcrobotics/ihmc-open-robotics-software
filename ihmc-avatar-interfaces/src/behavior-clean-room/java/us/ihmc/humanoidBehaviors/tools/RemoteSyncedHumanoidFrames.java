package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ros2.Ros2Node;

public class RemoteSyncedHumanoidFrames extends RemoteSyncedRobotModel
{
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   public RemoteSyncedHumanoidFrames(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      super(robotModel, ros2Node);

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
   }

   public HumanoidReferenceFrames pollHumanoidReferenceFrames()
   {
      pollFullRobotModel();
      humanoidReferenceFrames.updateFrames();
      return humanoidReferenceFrames;
   }
}
