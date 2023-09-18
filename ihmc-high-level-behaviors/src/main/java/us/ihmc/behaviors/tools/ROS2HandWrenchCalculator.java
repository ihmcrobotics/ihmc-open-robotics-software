package us.ihmc.behaviors.tools;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commonWalkingControlModules.contact.HandWrenchCalculator;
import us.ihmc.commons.thread.Notification;

public class ROS2HandWrenchCalculator extends HandWrenchCalculator
{
   // RobotConfigurationData is published at 120Hz -> break freq.: 5Hz - 20Hz
   private final Notification robotConfigurationSyncNotification = new Notification();

   public ROS2HandWrenchCalculator(ROS2SyncedRobotModel syncedRobot)
   {
      super(syncedRobot.getFullRobotModel(), null);
      syncedRobot.addRobotConfigurationDataReceivedCallback(robotConfigurationSyncNotification::set);
   }

   @Override
   public void compute()
   {
      if (robotConfigurationSyncNotification.poll())
      {
         super.compute();
      }
   }
}
