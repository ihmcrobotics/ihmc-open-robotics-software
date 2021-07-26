package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class MessagerSyncedRobotModel extends CommunicationsSyncedRobotModel
{
   private volatile RobotConfigurationData latestMessage;
   private boolean hasReceivedFirstMessage = false;

   private List<Consumer<RobotConfigurationData>> userCallbacks = new ArrayList<>();

   public MessagerSyncedRobotModel(Messager messager,
                                   MessagerAPIFactory.Topic<RobotConfigurationData> topic,
                                   FullHumanoidRobotModel fullRobotModel,
                                   HumanoidRobotSensorInformation sensorInformation)
   {
      super(fullRobotModel, sensorInformation);

      messager.registerTopicListener(topic, robotConfigurationData ->
      {
         FullRobotModelUtils.checkJointNameHash(jointNameHash, robotConfigurationData.getJointNameHash());
         latestMessage = robotConfigurationData;
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
      return latestMessage;
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
