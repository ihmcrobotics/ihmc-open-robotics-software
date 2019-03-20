package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class RemoteSyncedRobotModel
{
   protected final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;
   private RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::ros2UpdateCallback);
   }

   private void ros2UpdateCallback(Subscriber<RobotConfigurationData> subscriber)
   {
      synchronized (this)  // flush and get latest was not working, so sync over a callback
      {
         RobotConfigurationData incomingData = subscriber.takeNextData(); // may be 1 or 2 ticks behind, is this okay?
         if (incomingData != null)
         {
            FullRobotModelUtils.checkJointNameHash(jointNameHash, incomingData.getJointNameHash());
            robotConfigurationData = incomingData;
         }
      }
   }

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      synchronized (this)  // all accessors to robotConfigurationData must copy it's data, it's reference will be changing
      {
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
