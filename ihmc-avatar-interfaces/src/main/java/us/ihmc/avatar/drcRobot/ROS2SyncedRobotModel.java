package us.ihmc.avatar.drcRobot;

import controller_msgs.CapturabilityBasedStatus;
import controller_msgs.HandJointAnglePacket;
import controller_msgs.RobotConfigurationData;
import us.ihmc.avatar.sakeGripper.ROS2SakeHandStatus;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class ROS2SyncedRobotModel extends CommunicationsSyncedRobotModel
{
   private final ROS2Input<RobotConfigurationData> robotConfigurationDataInput;
   private final ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatusInput;
   private final SideDependentList<ROS2Input<HandJointAnglePacket>> handJointAnglePacketInputs = new SideDependentList<>();
   private final SideDependentList<ROS2SakeHandStatus> sakeHandStatus = new SideDependentList<>();
   private final AtomicBoolean loggedJointHashMismatch = new AtomicBoolean(false);

   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      this(robotModel, ros2Node, robotModel.createFullRobotModel());
   }
   
   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2Node ros2Node, boolean enforceUniqueReferenceFrames)
   {
      this(robotModel, ros2Node, robotModel.createFullRobotModel(enforceUniqueReferenceFrames));
   }

   public ROS2SyncedRobotModel(DRCRobotModel robotModel, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      super(robotModel, fullRobotModel, robotModel.getHandModels(), robotModel.getSensorInformation());

      robotConfigurationDataInput = new ROS2Input<>(ros2Node,
                                                    StateEstimatorAPI.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                                    robotConfigurationData,
                                                    message ->
                                                    {
                                                       if (jointNameHash == message.getJointNameHash())
                                                          return true;
                                                       if (loggedJointHashMismatch.compareAndSet(false, true))
                                                          LogTools.warn(
                                                                "Ignoring RobotConfigurationData with joint name hash {} (expected {}). Another robot model is publishing on this ROS domain.",
                                                                message.getJointNameHash(),
                                                                jointNameHash);
                                                       return false;
                                                    });
      robotConfigurationDataInput.addCallback(message -> resetDataReceptionTimer());
      capturabilityBasedStatusInput = new ROS2Input<>(ros2Node,
                                                      CapturabilityBasedStatus.class,
                                                      HumanoidControllerAPI.getLowFrequencyTopic(CapturabilityBasedStatus.class, robotModel.getSimpleRobotName()));

      for (RobotSide robotSide : RobotSide.values)
      {
         handJointAnglePacketInputs.set(robotSide, new ROS2Input<>(ros2Node,
                                                                   StateEstimatorAPI.getHandJointAnglesTopic(robotModel.getSimpleRobotName()),
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
   }
}
