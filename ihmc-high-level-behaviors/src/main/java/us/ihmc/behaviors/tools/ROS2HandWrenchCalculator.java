package us.ihmc.behaviors.tools;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commonWalkingControlModules.contact.HandWrenchCalculator;
import us.ihmc.commons.thread.Notification;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ROS2HandWrenchCalculator extends HandWrenchCalculator
{
   // RobotConfigurationData is published at 120Hz -> break freq.: 5Hz - 20Hz
   private final Notification robotConfigurationSyncNotification = new Notification();

   public ROS2HandWrenchCalculator(RobotSide side, ROS2SyncedRobotModel syncedRobot)
   {
      super(side,
            syncedRobot.getFullRobotModel(),
            new YoRegistry(ROS2HandWrenchCalculator.class.getSimpleName() + side.getPascalCaseName()),
            StateEstimatorParameters.ROBOT_CONFIGURATION_DATA_PUBLISH_DT);
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
