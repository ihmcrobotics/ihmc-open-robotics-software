package us.ihmc.humanoidRobotics.communication.packets;

import java.util.ArrayList;
import java.util.Random;
import java.util.stream.IntStream;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.bdi.BDIRobotBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public final class RandomHumanoidMessages
{
   private RandomHumanoidMessages()
   {
   }

   public static QueueableMessage nextQueueableMessage(Random random)
   {
      QueueableMessage next = new QueueableMessage();
      next.setExecutionMode(RandomNumbers.nextEnum(random, ExecutionMode.class).toByte());
      next.setPreviousMessageId(random.nextLong());
      next.setExecutionDelayTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      return next;
   }

   public static FrameInformation nextFrameInformation(Random random)
   {
      FrameInformation next = new FrameInformation();
      next.setTrajectoryReferenceFrameId(random.nextLong());
      next.setDataReferenceFrameId(random.nextLong());
      return next;
   }

   public static SelectionMatrix3DMessage nextSelectionMatrix3DMessage(Random random)
   {
      SelectionMatrix3DMessage next = new SelectionMatrix3DMessage();
      next.setSelectionFrameId(random.nextLong());
      next.setXSelected(random.nextBoolean());
      next.setYSelected(random.nextBoolean());
      next.setZSelected(random.nextBoolean());
      return next;
   }

   public static WeightMatrix3DMessage nextWeightMatrix3DMessage(Random random)
   {
      WeightMatrix3DMessage next = new WeightMatrix3DMessage();
      next.setWeightFrameId(random.nextLong());
      next.setXWeight(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setYWeight(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setZWeight(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      return next;
   }

   public static TrajectoryPoint1DMessage nextTrajectoryPoint1DMessage(Random random)
   {
      TrajectoryPoint1DMessage next = new TrajectoryPoint1DMessage();
      next.setTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      next.setPosition(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      next.setVelocity(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      return next;
   }

   public static TrajectoryPoint1DMessage[] nextTrajectoryPoint1DMessages(Random random)
   {
      return nextTrajectoryPoint1DMessages(random, random.nextInt(16) + 1);
   }

   public static TrajectoryPoint1DMessage[] nextTrajectoryPoint1DMessages(Random random, int length)
   {
      TrajectoryPoint1DMessage[] next = new TrajectoryPoint1DMessage[length];
      for (int i = 0; i < length; i++)
         next[i] = nextTrajectoryPoint1DMessage(random);
      return next;
   }

   public static OneDoFJointTrajectoryMessage nextOneDoFJointTrajectoryMessage(Random random)
   {
      OneDoFJointTrajectoryMessage next = new OneDoFJointTrajectoryMessage();
      MessageTools.copyData(nextTrajectoryPoint1DMessages(random), next.getTrajectoryPoints());
      next.setWeight(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      return next;
   }

   public static OneDoFJointTrajectoryMessage[] nextOneDoFJointTrajectoryMessages(Random random)
   {
      return nextOneDoFJointTrajectoryMessages(random, random.nextInt(10) + 1);
   }

   public static OneDoFJointTrajectoryMessage[] nextOneDoFJointTrajectoryMessages(Random random, int length)
   {
      OneDoFJointTrajectoryMessage[] next = new OneDoFJointTrajectoryMessage[length];
      for (int i = 0; i < length; i++)
         next[i] = nextOneDoFJointTrajectoryMessage(random);
      return next;
   }

   public static JointspaceTrajectoryMessage nextJointspaceTrajectoryMessage(Random random)
   {
      JointspaceTrajectoryMessage next = new JointspaceTrajectoryMessage();
      next.getQueueingProperties().set(nextQueueableMessage(random));
      MessageTools.copyData(nextOneDoFJointTrajectoryMessages(random), next.getJointTrajectoryMessages());
      return next;
   }

   public static ArmTrajectoryMessage nextArmTrajectoryMessage(Random random)
   {
      ArmTrajectoryMessage next = new ArmTrajectoryMessage();
      next.getJointspaceTrajectory().set(RandomHumanoidMessages.nextJointspaceTrajectoryMessage(random));
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      return next;
   }

   public static SE3TrajectoryPointMessage nextSE3TrajectoryPointMessage(Random random)
   {
      SE3TrajectoryPointMessage next = new SE3TrajectoryPointMessage();
      next.setTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      next.getPosition().set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0));
      next.getOrientation().set(RandomGeometry.nextQuaternion(random));
      next.getLinearVelocity().set(RandomGeometry.nextVector3D(random));
      next.getAngularVelocity().set(RandomGeometry.nextVector3D(random));
      return next;
   }

   public static SE3TrajectoryPointMessage[] nextSE3TrajectoryPointMessages(Random random)
   {
      return nextSE3TrajectoryPointMessages(random, random.nextInt(16) + 1);
   }

   public static SE3TrajectoryPointMessage[] nextSE3TrajectoryPointMessages(Random random, int length)
   {
      SE3TrajectoryPointMessage[] next = new SE3TrajectoryPointMessage[length];
      for (int i = 0; i < length; i++)
      {
         next[i] = nextSE3TrajectoryPointMessage(random);
      }
      return next;
   }

   public static SE3TrajectoryMessage nextSE3TrajectoryMessage(Random random)
   {
      SE3TrajectoryMessage next = new SE3TrajectoryMessage();
      MessageTools.copyData(nextSE3TrajectoryPointMessages(random), next.getTaskspaceTrajectoryPoints());
      next.getAngularSelectionMatrix().set(nextSelectionMatrix3DMessage(random));
      next.getLinearSelectionMatrix().set(nextSelectionMatrix3DMessage(random));
      next.getFrameInformation().set(nextFrameInformation(random));
      next.getAngularWeightMatrix().set(nextWeightMatrix3DMessage(random));
      next.getLinearWeightMatrix().set(nextWeightMatrix3DMessage(random));
      next.setUseCustomControlFrame(random.nextBoolean());
      next.getControlFramePose().set(EuclidGeometryRandomTools.nextPose3D(random));
      next.getQueueingProperties().set(nextQueueableMessage(random));
      return next;
   }

   public static PelvisTrajectoryMessage nextPelvisTrajectoryMessage(Random random)
   {
      PelvisTrajectoryMessage next = new PelvisTrajectoryMessage();
      next.setEnableUserPelvisControl(random.nextBoolean());
      next.setEnableUserPelvisControlDuringWalking(random.nextBoolean());
      next.getSe3Trajectory().set(nextSE3TrajectoryMessage(random));
      return next;
   }

   public static HandTrajectoryMessage nextHandTrajectoryMessage(Random random)
   {
      HandTrajectoryMessage next = new HandTrajectoryMessage();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getSe3Trajectory().set(nextSE3TrajectoryMessage(random));
      return next;
   }

   public static FootTrajectoryMessage nextFootTrajectoryMessage(Random random)
   {
      FootTrajectoryMessage next = new FootTrajectoryMessage();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getSe3Trajectory().set(nextSE3TrajectoryMessage(random));
      return next;
   }

   public static HandHybridJointspaceTaskspaceTrajectoryMessage nextHandHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      HandHybridJointspaceTaskspaceTrajectoryMessage next = new HandHybridJointspaceTaskspaceTrajectoryMessage();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getTaskspaceTrajectoryMessage().set(RandomHumanoidMessages.nextSE3TrajectoryMessage(random));
      next.getJointspaceTrajectoryMessage().set(RandomHumanoidMessages.nextJointspaceTrajectoryMessage(random));
      next.getJointspaceTrajectoryMessage().getQueueingProperties().set(next.getTaskspaceTrajectoryMessage().getQueueingProperties());
      return next;
   }

   public static HeadHybridJointspaceTaskspaceTrajectoryMessage nextHeadHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      HeadHybridJointspaceTaskspaceTrajectoryMessage next = new HeadHybridJointspaceTaskspaceTrajectoryMessage();
      next.getTaskspaceTrajectoryMessage().set(RandomHumanoidMessages.nextSO3TrajectoryMessage(random));
      next.getJointspaceTrajectoryMessage().set(RandomHumanoidMessages.nextJointspaceTrajectoryMessage(random));
      next.getJointspaceTrajectoryMessage().getQueueingProperties().set(next.getTaskspaceTrajectoryMessage().getQueueingProperties());
      return next;
   }

   public static ChestHybridJointspaceTaskspaceTrajectoryMessage nextChestHybridJointspaceTaskspaceTrajectoryMessage(Random random)
   {
      ChestHybridJointspaceTaskspaceTrajectoryMessage next = new ChestHybridJointspaceTaskspaceTrajectoryMessage();
      next.getTaskspaceTrajectoryMessage().set(RandomHumanoidMessages.nextSO3TrajectoryMessage(random));
      next.getJointspaceTrajectoryMessage().set(RandomHumanoidMessages.nextJointspaceTrajectoryMessage(random));
      next.getJointspaceTrajectoryMessage().getQueueingProperties().set(next.getTaskspaceTrajectoryMessage().getQueueingProperties());
      return next;
   }

   public static SO3TrajectoryPointMessage nextSO3TrajectoryPointMessage(Random random)
   {
      SO3TrajectoryPointMessage next = new SO3TrajectoryPointMessage();
      next.setTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      next.getOrientation().set(RandomGeometry.nextQuaternion(random));
      next.getAngularVelocity().set(RandomGeometry.nextVector3D(random));
      return next;
   }

   public static SO3TrajectoryPointMessage[] nextSO3TrajectoryPointMessages(Random random)
   {
      return nextSO3TrajectoryPointMessages(random, random.nextInt(16) + 1);
   }

   public static SO3TrajectoryPointMessage[] nextSO3TrajectoryPointMessages(Random random, int length)
   {
      SO3TrajectoryPointMessage[] next = new SO3TrajectoryPointMessage[length];
      for (int i = 0; i < length; i++)
      {
         next[i] = nextSO3TrajectoryPointMessage(random);
      }
      return next;
   }

   public static SO3TrajectoryMessage nextSO3TrajectoryMessage(Random random)
   {
      SO3TrajectoryMessage next = new SO3TrajectoryMessage();
      MessageTools.copyData(nextSO3TrajectoryPointMessages(random), next.getTaskspaceTrajectoryPoints());
      next.getFrameInformation().set(nextFrameInformation(random));
      next.getSelectionMatrix().set(nextSelectionMatrix3DMessage(random));
      next.getWeightMatrix().set(nextWeightMatrix3DMessage(random));
      next.setUseCustomControlFrame(random.nextBoolean());
      next.getControlFramePose().set(EuclidGeometryRandomTools.nextPose3D(random));
      next.getQueueingProperties().set(nextQueueableMessage(random));
      return next;
   }

   public static HeadTrajectoryMessage nextHeadTrajectoryMessage(Random random)
   {
      HeadTrajectoryMessage next = new HeadTrajectoryMessage();
      next.getSo3Trajectory().set(nextSO3TrajectoryMessage(random));
      return next;
   }

   public static PelvisOrientationTrajectoryMessage nextPelvisOrientationTrajectoryMessage(Random random)
   {
      PelvisOrientationTrajectoryMessage next = new PelvisOrientationTrajectoryMessage();
      next.getSo3Trajectory().set(nextSO3TrajectoryMessage(random));
      next.setEnableUserPelvisControlDuringWalking(random.nextBoolean());
      return next;
   }

   public static ChestTrajectoryMessage nextChestTrajectoryMessage(Random random)
   {
      ChestTrajectoryMessage next = new ChestTrajectoryMessage();
      next.getSo3Trajectory().set(nextSO3TrajectoryMessage(random));
      return next;
   }

   public static WholeBodyTrajectoryMessage nextWholeBodyTrajectoryMessage(Random random)
   {
      WholeBodyTrajectoryMessage next = new WholeBodyTrajectoryMessage();
      next.getLeftHandTrajectoryMessage().set(RandomHumanoidMessages.nextHandTrajectoryMessage(random));
      next.getLeftHandTrajectoryMessage().setRobotSide(RobotSide.LEFT.toByte());
      next.getRightHandTrajectoryMessage().set(RandomHumanoidMessages.nextHandTrajectoryMessage(random));
      next.getRightHandTrajectoryMessage().setRobotSide(RobotSide.RIGHT.toByte());
      next.getLeftArmTrajectoryMessage().set(RandomHumanoidMessages.nextArmTrajectoryMessage(random));
      next.getLeftArmTrajectoryMessage().setRobotSide(RobotSide.LEFT.toByte());
      next.getRightArmTrajectoryMessage().set(RandomHumanoidMessages.nextArmTrajectoryMessage(random));
      next.getRightArmTrajectoryMessage().setRobotSide(RobotSide.RIGHT.toByte());
      next.getLeftFootTrajectoryMessage().set(RandomHumanoidMessages.nextFootTrajectoryMessage(random));
      next.getLeftFootTrajectoryMessage().setRobotSide(RobotSide.LEFT.toByte());
      next.getRightFootTrajectoryMessage().set(RandomHumanoidMessages.nextFootTrajectoryMessage(random));
      next.getRightFootTrajectoryMessage().setRobotSide(RobotSide.RIGHT.toByte());
      next.getChestTrajectoryMessage().set(RandomHumanoidMessages.nextChestTrajectoryMessage(random));
      next.getPelvisTrajectoryMessage().set(RandomHumanoidMessages.nextPelvisTrajectoryMessage(random));
      next.getHeadTrajectoryMessage().set(RandomHumanoidMessages.nextHeadTrajectoryMessage(random));
      return next;
   }

   public static NeckTrajectoryMessage nextNeckTrajectoryMessage(Random random)
   {
      NeckTrajectoryMessage next = new NeckTrajectoryMessage();
      next.getJointspaceTrajectory().set(nextJointspaceTrajectoryMessage(random));
      return next;
   }

   public static SpineTrajectoryMessage nextSpineTrajectoryMessage(Random random)
   {
      SpineTrajectoryMessage next = new SpineTrajectoryMessage();
      next.getJointspaceTrajectory().set(nextJointspaceTrajectoryMessage(random));
      return next;
   }

   public static LoadBearingMessage nextLoadBearingMessage(Random random)
   {
      LoadBearingMessage next = new LoadBearingMessage();
      next.setLoad(random.nextBoolean());
      next.setCoefficientOfFriction(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.getBodyFrameToContactFrame().set(EuclidGeometryRandomTools.nextPose3D(random));
      next.getContactNormalInWorldFrame().set(EuclidCoreRandomTools.nextVector3D(random));
      return next;
   }

   public static HandLoadBearingMessage nextHandLoadBearingMessage(Random random)
   {
      HandLoadBearingMessage next = new HandLoadBearingMessage();
      next.setRobotSide(RobotSide.generateRandomRobotSide(random).toByte());
      next.setUseJointspaceCommand(random.nextBoolean());
      next.getJointspaceTrajectory().set(nextJointspaceTrajectoryMessage(random));
      next.setExecutionDelayTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.getLoadBearingMessage().set(nextLoadBearingMessage(random));
      return next;
   }

   public static FootLoadBearingMessage nextFootLoadBearingMessage(Random random)
   {
      FootLoadBearingMessage next = new FootLoadBearingMessage();
      next.setRobotSide(RobotSide.generateRandomRobotSide(random).toByte());
      next.setExecutionDelayTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setLoadBearingRequest(RandomNumbers.nextEnum(random, LoadBearingRequest.class).toByte());
      return next;
   }

   public static FootstepDataMessage nextFootstepDataMessage(Random random)
   {
      FootstepDataMessage next = new FootstepDataMessage();
      next.setRobotSide(RobotSide.generateRandomRobotSide(random).toByte());
      next.getLocation().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      IntStream.range(0, random.nextInt(10)).forEach(i -> next.getPredictedContactPoints2d().add().set(EuclidCoreRandomTools.nextPoint2D(random)));
      next.setTrajectoryType(RandomNumbers.nextEnum(random, TrajectoryType.class).toByte());
      next.setSwingHeight(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      if (next.getTrajectoryType() == TrajectoryType.CUSTOM.toByte())
      {
         next.getCustomPositionWaypoints().add().set(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         next.getCustomPositionWaypoints().add().set(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
      }
      else if (next.getTrajectoryType() == TrajectoryType.WAYPOINTS.toByte())
      {
         MessageTools.copyData(nextSE3TrajectoryPointMessages(random), next.getSwingTrajectory());
      }
      next.setSwingTrajectoryBlendDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setSwingDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setTransferDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setTouchdownDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setExecutionDelayTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      return next;
   }

   public static ArrayList<FootstepDataMessage> nextFootstepDataMessages(Random random)
   {
      return nextFootstepDataMessages(random, random.nextInt(16) + 1);
   }

   public static ArrayList<FootstepDataMessage> nextFootstepDataMessages(Random random, int length)
   {
      ArrayList<FootstepDataMessage> next = new ArrayList<>();
      for (int i = 0; i < length; i++)
         next.add(nextFootstepDataMessage(random));
      return next;
   }

   public static FootstepDataListMessage nextFootstepDataListMessage(Random random)
   {
      FootstepDataListMessage next = new FootstepDataListMessage();
      MessageTools.copyData(nextFootstepDataMessages(random), next.getFootstepDataList());
      next.setExecutionTiming(RandomNumbers.nextEnum(random, ExecutionTiming.class).toByte());
      next.setDefaultSwingDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setDefaultTransferDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setFinalTransferDuration(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      next.setTrustHeightOfFootsteps(random.nextBoolean());
      next.setAreFootstepsAdjustable(random.nextBoolean());
      next.setOffsetFootstepsWithExecutionError(random.nextBoolean());
      next.getQueueingProperties().set(nextQueueableMessage(random));
      return next;
   }

   public static HumanoidBehaviorTypePacket nextHumanoidBehaviorTypePacket(Random random)
   {
      HumanoidBehaviorTypePacket next = new HumanoidBehaviorTypePacket();
      next.setHumanoidBehaviorType(RandomNumbers.nextEnum(random, HumanoidBehaviorType.class).toByte());
      return next;
   }

   public static IMUPacket nextIMUPacket(Random random)
   {
      IMUPacket next = new IMUPacket();
      next.getLinearAcceleration().set(EuclidCoreRandomTools.nextVector3D32(random));
      next.getOrientation().set(EuclidCoreRandomTools.nextQuaternion32(random));
      next.getAngularVelocity().set(EuclidCoreRandomTools.nextVector3D32(random));
      return next;
   }

   public static IMUPacket[] nextIMUPackets(Random random)
   {
      return nextIMUPackets(random, random.nextInt(16) + 1);
   }

   public static IMUPacket[] nextIMUPackets(Random random, int length)
   {
      IMUPacket[] next = new IMUPacket[length];
      for (int i = 0; i < length; i++)
      {
         next[i] = nextIMUPacket(random);
      }
      return next;
   }

   public static RobotConfigurationData nextRobotConfigurationData(Random random)
   {
      RobotConfigurationData next = new RobotConfigurationData();
      int size = random.nextInt(10000);
      next.setWallTime(random.nextLong());
      next.setMonotonicTime(random.nextLong());
      next.setSyncTimestamp(random.nextLong());
      next.setJointNameHash(random.nextInt(10000));
      next.getJointAngles().add(RandomNumbers.nextFloatArray(random, size, 1.0f));
      next.getJointVelocities().add(RandomNumbers.nextFloatArray(random, size, 1.0f));
      next.getJointTorques().add(RandomNumbers.nextFloatArray(random, size, 1.0f));
      next.getRootTranslation().set(EuclidCoreRandomTools.nextVector3D32(random));
      next.getPelvisLinearVelocity().set(EuclidCoreRandomTools.nextVector3D32(random));
      next.getPelvisAngularVelocity().set(EuclidCoreRandomTools.nextVector3D32(random));
      next.getRootOrientation().set(EuclidCoreRandomTools.nextQuaternion32(random));
      next.getPelvisLinearAcceleration().set(EuclidCoreRandomTools.nextVector3D32(random));

      size = Math.abs(random.nextInt(1000));
      for (int i = 0; i < next.getForceSensorData().size(); i++)
         next.getForceSensorData().add().set(nextSpatialVectorMessage(random));
      for (IMUPacket imuPacket : nextIMUPackets(random))
         next.getImuSensorData().add().set(imuPacket);
      next.setRobotMotionStatus(RandomNumbers.nextEnum(random, RobotMotionStatus.class).toByte());
      next.setLastReceivedPacketTypeId(random.nextInt(1000));
      next.setLastReceivedPacketUniqueId(random.nextLong());
      next.setLastReceivedPacketRobotTimestamp(random.nextLong());
      return next;
   }

   public static SpatialVectorMessage nextSpatialVectorMessage(Random random)
   {
      SpatialVectorMessage next = new SpatialVectorMessage();
      next.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
      return next;
   }

   public static HighLevelStateChangeStatusMessage nextHighLevelStateChangeStatusMessage(Random random)
   {
      HighLevelStateChangeStatusMessage next = new HighLevelStateChangeStatusMessage();
      next.setInitialHighLevelControllerName(RandomNumbers.nextEnum(random, HighLevelControllerName.class).toByte());
      next.setEndHighLevelControllerName(RandomNumbers.nextEnum(random, HighLevelControllerName.class).toByte());
      return next;
   }

   public static FootstepPlanRequestPacket nextFootstepPlanRequestPacket(Random random)
   {
      FootstepPlanRequestPacket next = new FootstepPlanRequestPacket();
      next.getStartFootstep().set(nextFootstepDataMessage(random));
      next.setThetaStart(random.nextDouble());
      next.setMaxSubOptimality(random.nextDouble());
      MessageTools.copyData(nextFootstepDataMessages(random), next.getGoals());
      next.setFootstepPlanRequestType(RandomNumbers.nextEnum(random, FootstepPlanRequestType.class).toByte());
      return next;
   }

   public static HeatMapPacket nextHeatMapPacket(Random random)
   {
      HeatMapPacket next = new HeatMapPacket();
      next.setHeight(RandomNumbers.nextInt(random, -100, 100));
      next.setWidth(RandomNumbers.nextInt(random, -100, 100));
      next.getData().add(RandomNumbers.nextFloatArray(random, next.getHeight() * next.getWidth(), 1.0f));
      next.setName(Integer.toHexString(random.nextInt()));
      return next;
   }

   public static BoundingBoxesPacket nextBoundingBoxesPacket(Random random)
   {
      BoundingBoxesPacket next = new BoundingBoxesPacket();
      int boxesToGenerate = random.nextInt(20);

      for (int i = 0; i < boxesToGenerate; i++)
      {
         next.getLabels().add().append(Integer.toHexString(random.nextInt()));
         next.getBoundingBoxesXCoordinates().add(RandomNumbers.nextInt(random, -1000, 1000));
         next.getBoundingBoxesYCoordinates().add(RandomNumbers.nextInt(random, -1000, 1000));
         next.getBoundingBoxesWidths().add(RandomNumbers.nextInt(random, 0, 1000));
         next.getBoundingBoxesHeights().add(RandomNumbers.nextInt(random, 0, 1000));
      }
      return next;
   }

   public static ObjectDetectorResultPacket nextObjectDetectorResultPacket(Random random)
   {
      ObjectDetectorResultPacket next = new ObjectDetectorResultPacket();
      next.getHeatMap().set(nextHeatMapPacket(random));
      next.getBoundingBoxes().set(nextBoundingBoxesPacket(random));
      return next;
   }

   public static PauseWalkingMessage nextPauseWalkingMessage(Random random)
   {
      PauseWalkingMessage next = new PauseWalkingMessage();
      next.setPause(random.nextBoolean());
      return next;
   }

   public static AtlasLowLevelControlModeMessage nextAtlasLowLevelControlModeMessage(Random random)
   {
      AtlasLowLevelControlModeMessage next = new AtlasLowLevelControlModeMessage();
      next.setRequestedAtlasLowLevelControlMode(RandomNumbers.nextEnum(random, AtlasLowLevelControlMode.class).toByte());
      return next;
   }

   public static BehaviorControlModeResponsePacket nextBehaviorControlModeResponsePacket(Random random)
   {
      BehaviorControlModeResponsePacket next = new BehaviorControlModeResponsePacket();
      next.setBehaviorControlModeEnumRequest(RandomNumbers.nextEnum(random, BehaviorControlModeEnum.class).toByte());
      return next;
   }

   public static EuclideanTrajectoryPointMessage nextEuclideanTrajectoryPointMessage(Random random)
   {
      EuclideanTrajectoryPointMessage next = new EuclideanTrajectoryPointMessage();
      next.setTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      next.getPosition().set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0));
      next.getLinearVelocity().set(RandomGeometry.nextVector3D(random));
      return next;
   }

   public static EuclideanTrajectoryPointMessage[] nextEuclideanTrajectoryPointMessages(Random random)
   {
      return nextEuclideanTrajectoryPointMessages(random, random.nextInt(16) + 1);
   }

   public static EuclideanTrajectoryPointMessage[] nextEuclideanTrajectoryPointMessages(Random random, int length)
   {
      EuclideanTrajectoryPointMessage[] next = new EuclideanTrajectoryPointMessage[length];
      for (int i = 0; i < length; i++)
      {
         next[i] = nextEuclideanTrajectoryPointMessage(random);
      }
      return next;
   }

   public static EuclideanTrajectoryMessage nextEuclideanTrajectoryMessage(Random random)
   {
      EuclideanTrajectoryMessage next = new EuclideanTrajectoryMessage();
      MessageTools.copyData(nextEuclideanTrajectoryPointMessages(random), next.getTaskspaceTrajectoryPoints());
      next.getSelectionMatrix().set(nextSelectionMatrix3DMessage(random));
      next.getFrameInformation().set(nextFrameInformation(random));
      next.getWeightMatrix().set(nextWeightMatrix3DMessage(random));
      next.setUseCustomControlFrame(random.nextBoolean());
      next.getControlFramePose().set(EuclidGeometryRandomTools.nextPose3D(random));
      next.getQueueingProperties().set(nextQueueableMessage(random));
      return next;
   }

   public static PelvisHeightTrajectoryMessage nextPelvisHeightTrajectoryMessage(Random random)
   {
      PelvisHeightTrajectoryMessage next = new PelvisHeightTrajectoryMessage();
      next.getEuclideanTrajectory().set(nextEuclideanTrajectoryMessage(random));
      next.setEnableUserPelvisControl(random.nextBoolean());
      next.setEnableUserPelvisControlDuringWalking(random.nextBoolean());
      next.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      next.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      next.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return next;
   }

   public static QuadrupedBodyHeightMessage nextQuadrupedBodyHeightMessage(Random random)
   {
      QuadrupedBodyHeightMessage next = new QuadrupedBodyHeightMessage();
      next.getEuclideanTrajectory().set(nextEuclideanTrajectoryMessage(random));
      next.setControlBodyHeight(random.nextBoolean());
      next.setIsExpressedInAbsoluteTime(random.nextBoolean());
      next.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      next.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      next.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
      return next;
   }

   public static MomentumTrajectoryMessage nextMomentumTrajectoryMessage(Random random)
   {
      MomentumTrajectoryMessage next = new MomentumTrajectoryMessage();
      next.getAngularMomentumTrajectory().set(nextEuclideanTrajectoryMessage(random));
      return next;
   }

   public static CenterOfMassTrajectoryMessage nextCenterOfMassTrajectoryMessage(Random random)
   {
      CenterOfMassTrajectoryMessage next = new CenterOfMassTrajectoryMessage();
      next.getEuclideanTrajectory().set(nextEuclideanTrajectoryMessage(random));
      return next;
   }

   public static LocalizationStatusPacket nextLocalizationStatusPacket(Random random)
   {
      LocalizationStatusPacket next = new LocalizationStatusPacket();
      next.setOverlap(random.nextDouble());
      next.setStatus(Integer.toHexString(random.nextInt()));
      return next;
   }

   public static PelvisPoseErrorPacket nextPelvisPoseErrorPacket(Random random)
   {
      PelvisPoseErrorPacket next = new PelvisPoseErrorPacket();
      next.setResidualError(random.nextFloat());
      next.setTotalError(random.nextFloat());
      next.setHasMapBeenReset(random.nextBoolean());
      return next;
   }

   public static PointCloudWorldPacket nextPointCloudWorldPacket(Random random)
   {
      PointCloudWorldPacket next = new PointCloudWorldPacket();
      next.setTimestamp(random.nextLong());
      next.getGroundQuadTreeSupport().add(RandomNumbers.nextFloatArray(random, random.nextInt(), 100.0f));
      next.getDecayingWorldScan().add(RandomNumbers.nextFloatArray(random, random.nextInt(), 100.0f));
      next.setDefaultGroundHeight(random.nextFloat());
      return next;
   }

   public static FootstepPathPlanPacket nextFootstepPathPlanPacket(Random random)
   {
      FootstepPathPlanPacket next = new FootstepPathPlanPacket();
      next.setGoalsValid(random.nextBoolean());
      next.getStart().set(nextFootstepDataMessage(random));
      MessageTools.copyData(nextFootstepDataMessages(random), next.getOriginalGoals());
      MessageTools.copyData(nextFootstepDataMessages(random), next.getPathPlan());
      int size = Math.abs(random.nextInt(1000));
      for (int i = 0; i < size; i++)
      {
         next.getFootstepUnknown().add((byte) random.nextInt(2));
      }
      next.setSubOptimality(random.nextDouble());
      next.setPathCost(random.nextDouble());

      return next;
   }

   public static AtlasWristSensorCalibrationRequestPacket nextAtlasWristSensorCalibrationRequestPacket(Random random)
   {
      AtlasWristSensorCalibrationRequestPacket next = new AtlasWristSensorCalibrationRequestPacket();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      return next;
   }

   public static VehiclePosePacket nextVehiclePosePacket(Random random)
   {
      VehiclePosePacket next = new VehiclePosePacket();
      next.getPosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      return next;
   }

   public static HandPowerCyclePacket nextHandPowerCyclePacket(Random random)
   {
      HandPowerCyclePacket next = new HandPowerCyclePacket();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      return next;
   }

   public static CapturabilityBasedStatus nextCapturabilityBasedStatus(Random random)
   {
      CapturabilityBasedStatus next = new CapturabilityBasedStatus();
      double max = Double.MAX_VALUE / 2;
      next.getCapturePoint2d().set(RandomGeometry.nextPoint3D(random, max, max, 0.0));
      next.getDesiredCapturePoint2d().set(RandomGeometry.nextPoint3D(random, max, max, 0.0));
      next.getCenterOfMass3d().set(RandomGeometry.nextPoint3D(random, max, max, max));

      IntStream.range(0, HumanoidMessageTools.CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES).mapToObj(i -> EuclidCoreRandomTools.nextPoint2D(random)).forEach(next.getLeftFootSupportPolygon2d().add()::set);
      IntStream.range(0, HumanoidMessageTools.CAPTURABILITY_BASED_STATUS_MAXIMUM_NUMBER_OF_VERTICES).mapToObj(i -> EuclidCoreRandomTools.nextPoint2D(random)).forEach(next.getRightFootSupportPolygon2d().add()::set);

      return next;
   }

   public static HandDesiredConfigurationMessage nextHandDesiredConfigurationMessage(Random random)
   {
      HandDesiredConfigurationMessage next = new HandDesiredConfigurationMessage();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.setDesiredHandConfiguration(RandomNumbers.nextEnum(random, HandConfiguration.class).toByte());
      return next;
   }

   public static WalkingStatusMessage nextWalkingStatusMessage(Random random)
   {
      WalkingStatusMessage next = new WalkingStatusMessage();
      next.setWalkingStatus(RandomNumbers.nextEnum(random, WalkingStatus.class).toByte());
      return next;
   }

   public static SnapFootstepPacket nextSnapFootstepPacket(Random random)
   {
      SnapFootstepPacket next = new SnapFootstepPacket();
      // Number of footsteps
      int numberOfFootsteps = random.nextInt(255);

      // create random footsteps
      int[] footstepOrder = new int[numberOfFootsteps];
      byte[] flag = new byte[numberOfFootsteps];
      ArrayList<FootstepDataMessage> footsteps = new ArrayList<FootstepDataMessage>();
      RigidBodyTransform previousFootstep = new RigidBodyTransform();

      double[] XYZ_MAX = {2.0, 2.0, 2.0};
      double[] XYZ_MIN = {-2.0, -2.0, -3.0};

      double xMax = 0.90 * Math.min(Math.abs(XYZ_MAX[0]), Math.abs(XYZ_MIN[0]));
      double yMax = 0.90 * Math.min(Math.abs(XYZ_MAX[1]), Math.abs(XYZ_MIN[1]));
      double zMax = 0.90 * Math.min(Math.abs(XYZ_MAX[2]), Math.abs(XYZ_MIN[2]));

      for (int footstepNumber = 0; footstepNumber < numberOfFootsteps; footstepNumber++)
      {
         footstepOrder[footstepNumber] = footstepNumber;
         flag[footstepNumber] = (byte) random.nextInt(3);
         RobotSide robotSide = (footstepNumber % 2 == 0) ? RobotSide.RIGHT : RobotSide.LEFT;

         Point3D position = RandomGeometry.nextPoint3D(random, xMax, yMax, zMax);

         Quaternion orientation = new Quaternion();
         orientation.set(RandomGeometry.nextAxisAngle(random));

         previousFootstep.transform(position);

         previousFootstep.setTranslation(new Vector3D32(position));
         previousFootstep.setRotation(orientation);

         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, new Point3D(position), orientation);

         footsteps.add(footstepData);
      }

      MessageTools.copyData(footsteps, next.getFootstepData());
      next.getFootstepOrder().add(footstepOrder);
      next.getFlag().add(flag);
      return next;
   }

   public static DetectedObjectPacket nextDetectedObjectPacket(Random random)
   {
      DetectedObjectPacket next = new DetectedObjectPacket();
      next.getPose().set(EuclidGeometryRandomTools.nextPose3D(random));
      next.setId(random.nextInt(255));
      return next;
   }

   public static DesiredAccelerationsMessage nextDesiredAccelerationsMessage(Random random)
   {
      DesiredAccelerationsMessage next = new DesiredAccelerationsMessage();
      next.getDesiredJointAccelerations().add(RandomNumbers.nextDoubleArray(random, random.nextInt(16) + 1, 1.0));
      next.getQueueingProperties().set(nextQueueableMessage(random));
      return next;
   }

   public static NeckDesiredAccelerationsMessage nextNeckDesiredAccelerationsMessage(Random random)
   {
      NeckDesiredAccelerationsMessage next = new NeckDesiredAccelerationsMessage();
      next.getDesiredAccelerations().set(nextDesiredAccelerationsMessage(random));
      return next;
   }

   public static LocalizationPacket nextLocalizationPacket(Random random)
   {
      LocalizationPacket next = new LocalizationPacket();
      next.setReset(random.nextBoolean());
      next.setToggle(random.nextBoolean());
      return next;
   }

   public static ArmDesiredAccelerationsMessage nextArmDesiredAccelerationsMessage(Random random)
   {
      ArmDesiredAccelerationsMessage next = new ArmDesiredAccelerationsMessage();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getDesiredAccelerations().set(nextDesiredAccelerationsMessage(random));
      return next;
   }

   public static SpineDesiredAccelerationsMessage nextSpineDesiredAccelerationsMessage(Random random)
   {
      SpineDesiredAccelerationsMessage next = new SpineDesiredAccelerationsMessage();
      next.getDesiredAccelerations().set(nextDesiredAccelerationsMessage(random));
      return next;
   }

   public static MultisenseParameterPacket nextMultisenseParameterPacket(Random random)
   {
      MultisenseParameterPacket next = new MultisenseParameterPacket();
      next.setInitialize(random.nextBoolean());
      next.setGain(random.nextDouble());
      next.setMotorSpeed(random.nextDouble());
      next.setLedEnable(random.nextBoolean());
      next.setFlashEnable(random.nextBoolean());
      next.setDutyCycle(random.nextInt());
      next.setAutoExposure(random.nextBoolean());
      next.setAutoWhiteBalance(random.nextBoolean());
      return next;
   }

   public static KinematicsToolboxOutputStatus nextKinematicsToolboxOutputStatus(Random random)
   {
      KinematicsToolboxOutputStatus next = new KinematicsToolboxOutputStatus();
      next.setJointNameHash(random.nextInt());
      next.getDesiredJointAngles().add(RandomNumbers.nextFloatArray(random, random.nextInt(100), 1.0f));
      next.getDesiredRootTranslation().set(EuclidCoreRandomTools.nextVector3D32(random));
      next.getDesiredRootOrientation().set(EuclidCoreRandomTools.nextQuaternion32(random));
      next.setSolutionQuality(random.nextDouble());
      return next;
   }

   public static BlackFlyParameterPacket nextBlackFlyParameterPacket(Random random)
   {
      BlackFlyParameterPacket next = new BlackFlyParameterPacket();
      next.setAutoExposure(random.nextBoolean());
      next.setAutoGain(random.nextBoolean());
      next.setAutoShutter(random.nextBoolean());
      next.setExposure(random.nextDouble());
      next.setFrameRate(random.nextDouble());
      next.setFromUi(random.nextBoolean());
      next.setGain(random.nextDouble());
      next.setShutter(random.nextDouble());
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      return next;
   }

   public static VideoPacket nextVideoPacket(Random random)
   {
      VideoPacket next = new VideoPacket();
      next.setVideoSource(RandomNumbers.nextEnum(random, VideoSource.class).toByte());
      next.setTimestamp(random.nextLong());
      byte[] data = new byte[random.nextInt((int) (Math.pow(2, 20) - 19))];
      random.nextBytes(data);
      next.getData().add(data);
      next.getPosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      next.getIntrinsicParameters().set(nextIntrinsicParametersMessage(random));
      return next;
   }

   public static IntrinsicParametersMessage nextIntrinsicParametersMessage(Random random)
   {
      IntrinsicParametersMessage next = new IntrinsicParametersMessage();
      next.setWidth(random.nextInt());
      next.setHeight(random.nextInt());
      next.setFx(random.nextDouble());
      next.setFy(random.nextDouble());
      next.setSkew(random.nextDouble());
      next.setCx(random.nextDouble());
      next.setCy(random.nextDouble());
      next.getRadial().add(RandomNumbers.nextDoubleArray(random, random.nextInt(1000), 1.0));
      next.setT1(random.nextDouble());
      next.setT2(random.nextDouble());
      return next;
   }

   public static LocalizationPointMapPacket nextLocalizationPointMapPacket(Random random)
   {
      LocalizationPointMapPacket next = new LocalizationPointMapPacket();
      next.setTimestamp(random.nextLong());
      next.getLocalizationPointMap().add(RandomNumbers.nextFloatArray(random, random.nextInt(10000), 1.0f));
      return next;
   }

   public static GoHomeMessage nextGoHomeMessage(Random random)
   {
      GoHomeMessage next = new GoHomeMessage();
      next.setHumanoidBodyPart(RandomNumbers.nextEnum(random, HumanoidBodyPart.class).toByte());
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.setTrajectoryTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01));
      return next;
   }

   public static PrepareForLocomotionMessage nextPrepareForLocomotionMessage(Random random)
   {
      PrepareForLocomotionMessage next = new PrepareForLocomotionMessage();
      next.setPrepareManipulation(random.nextBoolean());
      next.setPreparePelvis(random.nextBoolean());
      return next;
   }

   public static StampedPosePacket nextStampedPosePacket(Random random)
   {
      StampedPosePacket next = new StampedPosePacket();
      next.getPose().set(EuclidGeometryRandomTools.nextPose3D(random));
      next.setTimestamp(random.nextLong());
      next.setConfidenceFactor(random.nextDouble());
      next.getFrameId().append(Integer.toHexString(random.nextInt()));
      return next;
   }

   public static AtlasDesiredPumpPSIPacket nextAtlasDesiredPumpPSIPacket(Random random)
   {
      AtlasDesiredPumpPSIPacket next = new AtlasDesiredPumpPSIPacket();
      next.setDesiredPumpPsi(random.nextInt());
      return next;
   }

   public static BDIBehaviorStatusPacket nextBDIBehaviorStatusPacket(Random random)
   {
      BDIBehaviorStatusPacket next = new BDIBehaviorStatusPacket();
      next.setCurrentBdiRobotBehavior(RandomNumbers.nextEnum(random, BDIRobotBehavior.class).toByte());
      return next;
   }

   public static StopAllTrajectoryMessage nextStopAllTrajectoryMessage(Random random)
   {
      StopAllTrajectoryMessage next = new StopAllTrajectoryMessage();
      return next;
   }

   public static AdjustFootstepMessage nextAdjustFootstepMessage(Random random)
   {
      AdjustFootstepMessage next = new AdjustFootstepMessage();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getLocation().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));
      IntStream.range(0, random.nextInt(10)).mapToObj(i -> EuclidCoreRandomTools.nextPoint2D(random)).forEach(next.getPredictedContactPoints2d().add()::set);
      next.setExecutionDelayTime(RandomNumbers.nextDoubleWithEdgeCases(random, 0.1));
      return next;
   }

   public static FootstepStatusMessage nextFootstepStatusMessage(Random random)
   {
      FootstepStatusMessage next = new FootstepStatusMessage();
      next.setFootstepStatus(RandomNumbers.nextEnum(random, FootstepStatus.class).toByte());
      next.setFootstepIndex(RandomNumbers.nextIntWithEdgeCases(random, 0.1));
      next.setRobotSide(RobotSide.generateRandomRobotSide(random).toByte());
      next.getDesiredFootPositionInWorld().set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0));
      next.getDesiredFootOrientationInWorld().set(RandomGeometry.nextQuaternion(random));
      next.getActualFootPositionInWorld().set(RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0));
      next.getActualFootOrientationInWorld().set(RandomGeometry.nextQuaternion(random));
      return next;
   }

   public static AtlasElectricMotorAutoEnableFlagPacket nextAtlasElectricMotorAutoEnableFlagPacket(Random random)
   {
      AtlasElectricMotorAutoEnableFlagPacket next = new AtlasElectricMotorAutoEnableFlagPacket();
      next.setShouldAutoEnable(random.nextBoolean());
      return next;
   }

   public static BDIBehaviorCommandPacket nextBDIBehaviorCommandPacket(Random random)
   {
      BDIBehaviorCommandPacket next = new BDIBehaviorCommandPacket();
      next.setAtlasBdiRobotBehavior(RandomNumbers.nextEnum(random, BDIRobotBehavior.class).toByte());
      next.setStop(random.nextBoolean());
      return next;
   }

   public static ObjectWeightPacket nextObjectWeightPacket(Random random)
   {
      ObjectWeightPacket next = new ObjectWeightPacket();
      next.setWeight(random.nextDouble());
      next.setRobotSide(random.nextBoolean() ? RobotSide.LEFT.toByte() : RobotSide.RIGHT.toByte());
      return next;
   }

   public static LegCompliancePacket nextLegCompliancePacket(Random random)
   {
      LegCompliancePacket next = new LegCompliancePacket();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getMaxVelocityDeltas().add(RandomNumbers.nextFloatArray(random, random.nextInt(1000), 1.0f));
      return next;
   }

   public static HandJointAnglePacket nextHandJointAnglePacket(Random random)
   {
      HandJointAnglePacket next = new HandJointAnglePacket();
      next.setRobotSide(RandomNumbers.nextEnum(random, RobotSide.class).toByte());
      next.getJointAngles().add(RandomNumbers.nextDoubleArray(random, random.nextInt(1000), 1.0));
      next.setConnected(random.nextBoolean());
      next.setCalibrated(random.nextBoolean());
      return next;
   }

   public static BehaviorControlModePacket nextBehaviorControlModePacket(Random random)
   {
      BehaviorControlModePacket next = new BehaviorControlModePacket();
      next.setBehaviorControlModeEnumRequest(RandomNumbers.nextEnum(random, BehaviorControlModeEnum.class).toByte());
      return next;
   }

   public static AtlasElectricMotorEnablePacket nextAtlasElectricMotorEnablePacket(Random random)
   {
      AtlasElectricMotorEnablePacket next = new AtlasElectricMotorEnablePacket();
      next.setAtlasElectricMotorPacketEnumEnable(RandomNumbers.nextEnum(random, AtlasElectricMotorPacketEnum.class).toByte());
      next.setEnable(random.nextBoolean());
      return next;
   }

   public static HighLevelStateMessage nextHighLevelStateMessage(Random random)
   {
      HighLevelStateMessage next = new HighLevelStateMessage();
      next.setHighLevelControllerName(RandomNumbers.nextEnum(random, HighLevelControllerName.class).toByte());
      return next;
   }

   public static SCSListenerPacket nextSCSListenerPacket(Random random)
   {
      SCSListenerPacket next = new SCSListenerPacket();
      next.setIsStopped(random.nextBoolean());
      return next;
   }

   public static SimulatedLidarScanPacket nextSimulatedLidarScanPacket(Random random)
   {
      SimulatedLidarScanPacket next = new SimulatedLidarScanPacket();
      int size = Math.abs(random.nextInt(1000000));
      for (int i = 0; i < size; i++)
      {
         next.getRanges().add(random.nextFloat());
      }

      next.setSensorId(random.nextInt());

      next.getLidarScanParameters().set(nextLidarScanParametersMessage(random));
      return next;
   }

   public static LidarScanParametersMessage nextLidarScanParametersMessage(Random random)
   {
      LidarScanParametersMessage next = new LidarScanParametersMessage();
      next.setTimestamp(random.nextLong());
      next.setSweepYawMax(random.nextFloat());
      next.setSweepYawMin(random.nextFloat());
      next.setHeightPitchMax(random.nextFloat());
      next.setHeightPitchMin(random.nextFloat());
      next.setTimeIncrement(random.nextFloat());
      next.setScanTime(random.nextFloat());
      next.setMinRange(random.nextFloat());
      next.setMaxRange(random.nextFloat());
      next.setPointsPerSweep(random.nextInt());
      next.setScanHeight(random.nextInt());
      return null;
   }

   public static ManualHandControlPacket nextManualHandControlPacket(Random random)
   {
      ManualHandControlPacket next = new ManualHandControlPacket();
      next.setRobotSide(RobotSide.generateRandomRobotSide(random).toByte());
      double[] angles = RandomNumbers.nextDoubleArray(random, 4, 0, 1);

      next.setIndex(angles[0]);
      next.setMiddle(angles[1]);
      next.setThumb(angles[2]);
      next.setSpread(angles[3]);
      next.setControlType(0);
      return next;
   }

   public static AbortWalkingMessage nextAbortWalkingMessage(Random random)
   {
      AbortWalkingMessage next = new AbortWalkingMessage();
      return next;
   }
}
