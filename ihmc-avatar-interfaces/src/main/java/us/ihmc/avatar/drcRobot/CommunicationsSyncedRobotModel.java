package us.ihmc.avatar.drcRobot;

import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.commonWalkingControlModules.contact.HandWrenchCalculator;
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
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

public abstract class CommunicationsSyncedRobotModel
{
   private final DRCRobotModel robotModel;
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
   private final ArrayList<SpatialVectorMessage> forceSensorData = new ArrayList<>();
   private final SideDependentList<HandWrenchCalculator> handWrenchCalculators = new SideDependentList<>();

   public CommunicationsSyncedRobotModel(DRCRobotModel robotModel,
                                         FullHumanoidRobotModel fullRobotModel,
                                         SideDependentList<HandModel> handModels,
                                         HumanoidRobotSensorInformation sensorInformation)
   {
      this.robotModel = robotModel;
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

      if (handModels != null)
      {
         for (RobotSide side : handModels.sides())
         {
            if (robotModel.getRobotVersion().hasSakeGripperJoints(side))
               handWrenchCalculators.put(side, new HandWrenchCalculator(side, fullRobotModel, null, StateEstimatorParameters.ROBOT_CONFIGURATION_DATA_PUBLISH_DT));
         }
      }
   }

   public void initializeToDefaultRobotInitialSetup(double groundHeight, double initialYaw, double x, double y)
   {
      robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw, x, y).initializeFullRobotModel(fullRobotModel);
      updateFramesForFullRobotModel();
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
      fullRobotModel.getRootJoint().setJointPosition(robotConfigurationData.getRootPosition());

      for (int i = 0; i < robotConfigurationData.getJointAngles().size(); i++)
      {
         allJoints[i].setQ(robotConfigurationData.getJointAngles().get(i));
         allJoints[i].setQd(robotConfigurationData.getJointVelocities().get(i));
         allJoints[i].setTau(robotConfigurationData.getJointTorques().get(i));
      }

      forceSensorData.clear();
      for (int i = 0; i < robotConfigurationData.getForceSensorData().size(); i++)
      {
         SpatialVectorMessage spatialVectorMessage = robotConfigurationData.getForceSensorData().get(i);
         forceSensorData.add(spatialVectorMessage);
      }

      if (handModels != null)
      {
         HandModelUtils.copyHandJointAnglesFromMessagesToOneDoFJoints(handModels, handJoints, handJointAnglePackets);
      }

      updateFramesForFullRobotModel();

      for (RobotSide side : handWrenchCalculators.sides())
      {
         handWrenchCalculators.get(side).compute();
      }
   }

   private void updateFramesForFullRobotModel()
   {
      try
      {
         fullRobotModel.getElevator().updateFramesRecursively();
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

   public ArrayList<SpatialVectorMessage> getForceSensorData()
   {
      return forceSensorData;
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

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public SideDependentList<HandWrenchCalculator> getHandWrenchCalculators()
   {
      return handWrenchCalculators;
   }
}
