package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Input;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

import static us.ihmc.communication.ROS2Tools.HUMANOID_CONTROLLER;

public class RemoteSyncedRobotModel
{
   protected final FullHumanoidRobotModel fullRobotModel;
   protected RobotConfigurationData robotConfigurationData;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;
   private final ROS2Input<RobotConfigurationData> robotConfigurationDataInput;

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      robotConfigurationData = new RobotConfigurationData();
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());

      robotConfigurationDataInput = new ROS2Input<RobotConfigurationData>(ros2Node,
                                                                          RobotConfigurationData.class,
                                                                          robotModel.getSimpleRobotName(), HUMANOID_CONTROLLER,
                                                                     message ->
                                                                     {
                                                                        FullRobotModelUtils.checkJointNameHash(jointNameHash,
                                                                                                               message.getJointNameHash());
                                                                        return true;
                                                                     });
   }

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      robotConfigurationData = robotConfigurationDataInput.getLatest();

      fullRobotModel.getRootJoint().setJointOrientation(robotConfigurationData.getRootOrientation());
      fullRobotModel.getRootJoint().setJointPosition(robotConfigurationData.getRootTranslation());

      for (int i = 0; i < robotConfigurationData.getJointAngles().size(); i++)
      {
         allJoints[i].setQ(robotConfigurationData.getJointAngles().get(i));
      }

      fullRobotModel.getElevator().updateFramesRecursively();

      return fullRobotModel;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public boolean hasReceivedFirstMessage()
   {
      return robotConfigurationDataInput.hasReceivedFirstMessage();
   }
}
