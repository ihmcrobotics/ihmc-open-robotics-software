package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.Ros2QueuedSubscription;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class RemoteSyncedRobotModel
{
   protected final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;
   private final Ros2QueuedSubscription<RobotConfigurationData> robotConfigurationDataQueue;
   private final RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());

      robotConfigurationDataQueue = ROS2Tools.createQueuedSubscription(ros2Node, RobotConfigurationData.class,
                                                                       ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()));
   }

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      if (robotConfigurationDataQueue.flushAndGetLatest(robotConfigurationData))
      {
         FullRobotModelUtils.checkJointNameHash(jointNameHash, robotConfigurationData.getJointNameHash());

         fullRobotModel.getRootJoint().setJointOrientation(robotConfigurationData.getRootOrientation());
         fullRobotModel.getRootJoint().setJointPosition(robotConfigurationData.getRootTranslation());

         for (int i = 0; i < robotConfigurationData.getJointAngles().size(); i++)
         {
            allJoints[i].setQ(robotConfigurationData.getJointAngles().get(i));
         }

         fullRobotModel.getElevator().updateFramesRecursively();
      }

      return fullRobotModel;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
