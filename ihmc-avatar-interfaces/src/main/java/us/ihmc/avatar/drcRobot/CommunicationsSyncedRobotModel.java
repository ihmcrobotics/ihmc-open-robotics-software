package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshot;

import java.util.HashMap;
import java.util.function.Function;

public abstract class CommunicationsSyncedRobotModel
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final SideDependentList<HandModel> handModels;
   private final Timer dataReceptionTimer;
   protected RobotConfigurationData robotConfigurationData;
   protected SideDependentList<HandJointAnglePacket> handJointAnglePackets = new SideDependentList<>();
   private final OneDoFJointBasics[] allJoints;
   private final SideDependentList<HashMap<HandJointName, OneDoFJointBasics>> handJoints = new SideDependentList<>();
   protected final int jointNameHash;
   private final HumanoidReferenceFrames referenceFrames;
   private final FramePose3D temporaryPoseForQuickReading = new FramePose3D();

   public CommunicationsSyncedRobotModel(FullHumanoidRobotModel fullRobotModel, SideDependentList<HandModel> handModels, HumanoidRobotSensorInformation sensorInformation)
   {
      this.fullRobotModel = fullRobotModel;
      this.handModels = handModels;
      robotConfigurationData = new RobotConfigurationData();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel, sensorInformation);
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);

      if (handModels != null)
         HandModelUtils.getHandJoints(handModels, fullRobotModel, handJoints);

      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());
      dataReceptionTimer = new Timer();
   }

   public abstract RobotConfigurationData getLatestRobotConfigurationData();

   public abstract HandJointAnglePacket getLatestHandJointAnglePacket(RobotSide robotSide);

   protected synchronized void resetDataReceptionTimer()
   {
      dataReceptionTimer.reset();
   }

   public void update()
   {
      robotConfigurationData = getLatestRobotConfigurationData();
      for (RobotSide robotSide : RobotSide.values)
      {
         handJointAnglePackets.set(robotSide, getLatestHandJointAnglePacket(robotSide));
      }

      if (robotConfigurationData != null)
      {
         updateInternal();
      }
   }

   protected void updateInternal()
   {
      fullRobotModel.getRootJoint().setJointOrientation(robotConfigurationData.getRootOrientation());
      fullRobotModel.getRootJoint().setJointPosition(robotConfigurationData.getRootTranslation());

      for (int i = 0; i < robotConfigurationData.getJointAngles().size(); i++)
      {
         allJoints[i].setQ(robotConfigurationData.getJointAngles().get(i));
      }

      if (handModels != null)
      {
         HandModelUtils.copyHandJointAnglesFromMessagesToOneDoFJoints(handModels, handJoints, handJointAnglePackets);
      }

      fullRobotModel.getElevator().updateFramesRecursively();

      try
      {
         referenceFrames.updateFrames();
      }
      catch (NotARotationMatrixException e)
      {
         LogTools.error(e.getMessage());
      }
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

   public synchronized TimerSnapshot getDataReceptionTimerSnapshot()
   {
      return dataReceptionTimer.createSnapshot();
   }
}
