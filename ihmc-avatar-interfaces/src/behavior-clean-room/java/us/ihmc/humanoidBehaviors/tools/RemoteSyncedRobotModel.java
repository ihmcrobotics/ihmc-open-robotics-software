package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Input;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

import static us.ihmc.communication.ROS2Tools.*;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

public class RemoteSyncedRobotModel
{
   protected final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;
   private final ROS2Input<RobotConfigurationData> robotConfigurationData;
   private long monotonicTime = 0L;

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());

      robotConfigurationData = new ROS2Input<RobotConfigurationData>(ros2Node,
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
      RobotConfigurationData latestRobotConfigurationData = robotConfigurationData.getLatest();
      monotonicTime = latestRobotConfigurationData.getMonotonicTime();

      fullRobotModel.getRootJoint().setJointOrientation(latestRobotConfigurationData.getRootOrientation());
      fullRobotModel.getRootJoint().setJointPosition(latestRobotConfigurationData.getRootTranslation());

      for (int i = 0; i < latestRobotConfigurationData.getJointAngles().size(); i++)
      {
         allJoints[i].setQ(latestRobotConfigurationData.getJointAngles().get(i));
      }

      fullRobotModel.getElevator().updateFramesRecursively();

      return fullRobotModel;
   }

   public Pair<FullHumanoidRobotModel, Long> pollFullRobotModelAndTimestamp()
   {
      FullHumanoidRobotModel fullRobotModel = pollFullRobotModel();
      ImmutablePair<FullHumanoidRobotModel, Long> modelAndTimestamp = new ImmutablePair<FullHumanoidRobotModel, Long>(fullRobotModel, monotonicTime);
      return modelAndTimestamp;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Pair<FullHumanoidRobotModel, Long> getFullRobotModelAndTimestamp()
   {
      ImmutablePair<FullHumanoidRobotModel, Long> modelAndTimestamp = new ImmutablePair<FullHumanoidRobotModel, Long>(fullRobotModel, monotonicTime);
      return modelAndTimestamp;
   }
}
