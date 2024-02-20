package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.function.Consumer;

public class ROS2SyncedRobotModel extends CommunicationsSyncedRobotModel
{
   private final ROS2Input<RobotConfigurationData> robotConfigurationDataInput;
   private final SideDependentList<ROS2Input<HandJointAnglePacket>> handJointAnglePacketInputs = new SideDependentList<>();

   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      this(robotModel, ros2Node, robotModel.createFullRobotModel());
   }

   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      super(robotModel, fullRobotModel, robotModel.getHandModels(), robotModel.getSensorInformation());

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

      for (RobotSide robotSide : RobotSide.values)
      {
         handJointAnglePacketInputs.set(robotSide, new ROS2Input<>(ros2Node,
                                                                   ROS2Tools.getHandJointAnglesTopic(robotModel.getSimpleRobotName()),
                                                                   null,
                                                                   message -> robotSide.toByte() == message.getRobotSide()));
      }
   }

   @Override
   public RobotConfigurationData getLatestRobotConfigurationData()
   {
      return robotConfigurationDataInput.getLatest();
   }

   @Override
   public HandJointAnglePacket getLatestHandJointAnglePacket(RobotSide robotSide)
   {
      return handJointAnglePacketInputs.get(robotSide).getLatest();
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

   public void destroy()
   {
      robotConfigurationDataInput.destroy();
   }
}
