package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;

public class HumanoidRobotState extends HumanoidReferenceFrames
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final RobotConfigurationData robotConfigurationData;

   public HumanoidRobotState(FullHumanoidRobotModel fullRobotModel, RobotConfigurationData robotConfigurationData)
   {
      super(fullRobotModel);
      this.fullRobotModel = fullRobotModel;
      this.robotConfigurationData = robotConfigurationData;
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public RobotConfigurationData getRobotConfigurationData()
   {
      return robotConfigurationData;
   }

   public long getTimestamp()
   {
      return robotConfigurationData.getMonotonicTime();
   }
}
