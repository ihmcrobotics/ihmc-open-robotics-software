package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.Timer;
import us.ihmc.communication.util.TimerSnapshot;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

import java.util.function.Function;

public class RemoteSyncedRobotModel
{
   protected final FullHumanoidRobotModel fullRobotModel;
   private final Timer dataReceptionTimer;
   protected RobotConfigurationData robotConfigurationData;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;
   private final ROS2Input<RobotConfigurationData> robotConfigurationDataInput;
   private final HumanoidReferenceFrames referenceFrames;

   private final FramePose3D temporaryPoseForQuickReading = new FramePose3D();

   public RemoteSyncedRobotModel(DRCRobotModel robotModel, Ros2NodeInterface ros2Node)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      robotConfigurationData = new RobotConfigurationData();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());
      robotConfigurationDataInput = new ROS2Input<>(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                                    ROS2TopicNameTools.newMessageInstance(RobotConfigurationData.class),
                                                    message ->
                                                    {
                                                       FullRobotModelUtils.checkJointNameHash(jointNameHash, message.getJointNameHash());
                                                       return true;
                                                    });
      dataReceptionTimer = new Timer();
      robotConfigurationDataInput.addCallback(this::resetDataReceptionTimer);
   }

   private synchronized void resetDataReceptionTimer(RobotConfigurationData message)
   {
      dataReceptionTimer.reset();
   }

   public void update()
   {
      robotConfigurationData = robotConfigurationDataInput.getLatest();

      fullRobotModel.getRootJoint().setJointOrientation(robotConfigurationData.getRootOrientation());
      fullRobotModel.getRootJoint().setJointPosition(robotConfigurationData.getRootTranslation());

      for (int i = 0; i < robotConfigurationData.getJointAngles().size(); i++)
      {
         allJoints[i].setQ(robotConfigurationData.getJointAngles().get(i));
      }

      fullRobotModel.getElevator().updateFramesRecursively();

      referenceFrames.updateFrames();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public RobotConfigurationData getRobotConfigurationData()
   {
      return robotConfigurationData;
   }

   public long getTimestamp()
   {
      return robotConfigurationData.getMonotonicTime();
   }

   public FramePose3DReadOnly getFramePoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      temporaryPoseForQuickReading.setFromReferenceFrame(frameSelector.apply(referenceFrames));
      return temporaryPoseForQuickReading;
   }

   public boolean hasReceivedFirstMessage()
   {
      return robotConfigurationDataInput.hasReceivedFirstMessage();
   }

   public synchronized TimerSnapshot getDataReceptionTimerSnapshot()
   {
      return dataReceptionTimer.createSnapshot();
   }
}
