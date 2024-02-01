package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class MessagerSyncedRobotModel extends CommunicationsSyncedRobotModel
{
   private volatile RobotConfigurationData latestRobotConfigurationData;
   private boolean hasReceivedFirstMessage = false;

   private final List<Consumer<RobotConfigurationData>> userCallbacks = new ArrayList<>();

   public MessagerSyncedRobotModel(Messager messager,
                                   MessagerAPIFactory.Topic<RobotConfigurationData> topic,
                                   DRCRobotModel robotModel,
                                   FullHumanoidRobotModel fullRobotModel,
                                   HumanoidRobotSensorInformation sensorInformation)
   {
      super(robotModel, fullRobotModel, null, sensorInformation);

      messager.addTopicListener(topic, robotConfigurationData ->
      {
         FullRobotModelUtils.checkJointNameHash(jointNameHash, robotConfigurationData.getJointNameHash());
         latestRobotConfigurationData = robotConfigurationData;
         resetDataReceptionTimer();
         hasReceivedFirstMessage = true;
         for (Consumer<RobotConfigurationData> userCallback : userCallbacks)
         {
            userCallback.accept(robotConfigurationData);
         }
      });
   }

   @Override
   public RobotConfigurationData getLatestRobotConfigurationData()
   {
      return latestRobotConfigurationData;
   }

   @Override
   public HandJointAnglePacket getLatestHandJointAnglePacket(RobotSide robotSide)
   {
      return null; // TODO: Implement
   }

   public boolean hasReceivedFirstMessage()
   {
      return hasReceivedFirstMessage;
   }

   public void addRobotConfigurationDataReceivedCallback(Runnable callback)
   {
      addRobotConfigurationDataReceivedCallback(message -> callback.run());
   }

   public void addRobotConfigurationDataReceivedCallback(Consumer<RobotConfigurationData> callback)
   {
      userCallbacks.add(callback);
   }
}
