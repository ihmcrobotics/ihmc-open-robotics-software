package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.sakeGripper.ROS2SakeHandStatus;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Consumer;

public class ROS2SyncedRobotModel extends CommunicationsSyncedRobotModel
{
   private final ROS2Input<RobotConfigurationData> robotConfigurationDataInput;
   private final ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatusInput;
   private final SideDependentList<ROS2Input<HandJointAnglePacket>> handJointAnglePacketInputs = new SideDependentList<>();
   private final SideDependentList<ROS2SakeHandStatus> sakeHandStatus = new SideDependentList<>();

   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      this(robotModel, ros2Node, robotModel.createFullRobotModel());
   }

   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2NodeInterface ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      super(robotModel, fullRobotModel, robotModel.getHandModels(), robotModel.getSensorInformation());

      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotModel.getSimpleRobotName());
      robotConfigurationDataInput = new ROS2Input<>(ros2Node,
                                                    ControllerAPI.getTopic(controllerOutputTopic, RobotConfigurationData.class),
                                                    robotConfigurationData,
                                                    message ->
                                                    {
                                                       FullRobotModelUtils.checkJointNameHash(jointNameHash, message.getJointNameHash());
                                                       return true;
                                                    });
      robotConfigurationDataInput.addCallback(message -> resetDataReceptionTimer());
      capturabilityBasedStatusInput = new ROS2Input<>(ros2Node,
                                                      CapturabilityBasedStatus.class,
                                                      ControllerAPI.getTopic(controllerOutputTopic, CapturabilityBasedStatus.class));

      for (RobotSide robotSide : RobotSide.values)
      {
         handJointAnglePacketInputs.set(robotSide, new ROS2Input<>(ros2Node,
                                                                   ControllerAPI.getTopic(controllerOutputTopic, HandJointAnglePacket.class),
                                                                   null,
                                                                   message -> robotSide.toByte() == message.getRobotSide()));
         sakeHandStatus.put(robotSide, new ROS2SakeHandStatus(ros2Node, robotModel.getSimpleRobotName(), robotSide));
      }
   }

   @Override
   public RobotConfigurationData getLatestRobotConfigurationData()
   {
      return robotConfigurationDataInput.getLatest();
   }

   public CapturabilityBasedStatus getLatestCapturabilityBasedStatus()
   {
      return capturabilityBasedStatusInput.getLatest();
   }

   @Override
   public HandJointAnglePacket getLatestHandJointAnglePacket(RobotSide robotSide)
   {
      return handJointAnglePacketInputs.get(robotSide).getLatest();
   }

   public SideDependentList<ROS2SakeHandStatus> getSakeHandStatus()
   {
      return sakeHandStatus;
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
