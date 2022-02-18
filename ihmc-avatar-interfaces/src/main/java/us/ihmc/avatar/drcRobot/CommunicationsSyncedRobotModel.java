package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshot;

import java.util.function.Function;

public abstract class CommunicationsSyncedRobotModel
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final Timer dataReceptionTimer;
   protected RobotConfigurationData robotConfigurationData;
   private final OneDoFJointBasics[] allJoints;
   protected final int jointNameHash;
   private final HumanoidReferenceFrames referenceFrames;
   private final FramePose3D temporaryPoseForQuickReading = new FramePose3D();

   public CommunicationsSyncedRobotModel(FullHumanoidRobotModel fullRobotModel, HumanoidRobotSensorInformation sensorInformation)
   {
      this.fullRobotModel = fullRobotModel;
      robotConfigurationData = new RobotConfigurationData();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel, sensorInformation);
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());
      dataReceptionTimer = new Timer();
   }

   public abstract RobotConfigurationData getLatestRobotConfigurationData();

   protected synchronized void resetDataReceptionTimer()
   {
      dataReceptionTimer.reset();
   }

   public void update()
   {
      robotConfigurationData = getLatestRobotConfigurationData();

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
