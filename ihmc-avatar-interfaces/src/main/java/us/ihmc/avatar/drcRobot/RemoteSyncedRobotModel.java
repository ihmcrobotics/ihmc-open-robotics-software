package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.function.Consumer;

public class RemoteSyncedRobotModel extends MessageSyncedRobotModel
{
   private final ROS2Input<RobotConfigurationData> robotConfigurationDataInput;

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      this(robotModel, ros2Node, robotModel.createFullRobotModel());
   }

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);
      robotConfigurationDataInput = new ROS2Input<>(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                                    robotConfigurationData,
                                                    message ->
                                                    {
                                                       FullRobotModelUtils.checkJointNameHash(jointNameHash, message.getJointNameHash());
                                                       return true;
                                                    });
      robotConfigurationDataInput.addCallback(message -> resetDataReceptionTimer());
   }

   @Override
   public RobotConfigurationData getLatestRobotConfigurationData()
   {
      return robotConfigurationDataInput.getLatest();
   }

   public boolean hasReceivedFirstMessage()
   {
      return robotConfigurationDataInput.hasReceivedFirstMessage();
   }

   public void addRobotConfigurationDataReceivedCallback(Runnable callback)
   {
      robotConfigurationDataInput.addCallback(message -> callback.run());
   }

   public void addRobotConfigurationDataReceivedCallback(Consumer<RobotConfigurationData> callback)
   {
      robotConfigurationDataInput.addCallback(callback);
   }
}
