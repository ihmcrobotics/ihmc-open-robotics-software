package us.ihmc.quadrupedPlanning.input;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;

import java.util.zip.CRC32;

public class QuadrupedRobotModelProviderNode
{
   private final FullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedReferenceFrames referenceFrames;

   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;

   public QuadrupedRobotModelProviderNode(String robotName, Ros2Node ros2Node, FullQuadrupedRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      allJoints = fullRobotModel.getOneDoFJoints();
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
      jointNameHash = calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());


      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> processRobotConfigurationData(s.takeNextData()));
   }

   public QuadrupedRobotModelProviderNode(String robotName, RealtimeRos2Node ros2Node, FullQuadrupedRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      allJoints = fullRobotModel.getOneDoFJoints();
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
      jointNameHash = calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());


      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> processRobotConfigurationData(s.takeNextData()));
   }

   public FullQuadrupedRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   private void processRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (robotConfigurationData.getJointNameHash() != jointNameHash)
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      RigidBodyTransform newRootJointPose = new RigidBodyTransform(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
      fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);

      float[] newJointConfiguration = robotConfigurationData.getJointAngles().toArray();
      for (int i = 0; i < allJoints.length; i++)
         allJoints[i].setQ(newJointConfiguration[i]);

      fullRobotModel.getElevator().updateFramesRecursively();
      referenceFrames.updateFrames();
   }

   private static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointBasics joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }

}
