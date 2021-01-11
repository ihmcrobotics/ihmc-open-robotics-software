package us.ihmc.humanoidBehaviors.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.tools.ros2.ROS2ControllerPublisherMap;
import us.ihmc.humanoidBehaviors.tools.ros2.ROS2PublisherMap;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

// TODO: Clean this up by using DRCUserInterfaceNetworkingManager (After cleaning that up first...)
public class RemoteHumanoidRobotInterface
{
   private final ROS2NodeInterface ros2Node;
   private final DRCRobotModel robotModel;
   private final String robotName;
   private final HumanoidJointNameMap jointMap;

   private final ROS2ControllerPublisherMap controllerPublisherMap;
   private final ROS2PublisherMap publisherMap;

   private final ArrayList<TypedNotification<WalkingStatusMessage>> walkingCompletedNotifications = new ArrayList<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();
   private final ROS2Input<HighLevelStateChangeStatusMessage> controllerStateInput;
   private final ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatusInput;

   private final RemoteSyncedRobotModel syncedRobot;

   private final ROS2Topic<?> topicName;

   public RemoteHumanoidRobotInterface(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      robotName = robotModel.getSimpleRobotName();
      jointMap = robotModel.getJointMap();
      topicName = ROS2Tools.HUMANOID_CONTROLLER.withRobot(robotName);

      controllerPublisherMap = new ROS2ControllerPublisherMap(ros2Node, robotName);
      publisherMap = new ROS2PublisherMap(ros2Node);
      
      new ROS2Callback<>(ros2Node, WalkingStatusMessage.class, topicName.withOutput(), this::acceptWalkingStatus);
      new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, topicName.withOutput(), footstepStatusMessage::set);

      HighLevelStateChangeStatusMessage initialState = new HighLevelStateChangeStatusMessage();
      initialState.setInitialHighLevelControllerName(HighLevelControllerName.DO_NOTHING_BEHAVIOR.toByte());
      initialState.setEndHighLevelControllerName(HighLevelControllerName.WALKING.toByte());
      controllerStateInput = new ROS2Input<>(ros2Node, HighLevelStateChangeStatusMessage.class, topicName.withOutput(), initialState, this::acceptStatusChange);
      capturabilityBasedStatusInput = new ROS2Input<>(ros2Node, CapturabilityBasedStatus.class, topicName.withOutput());

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
   }

   // TODO: Somehow wrap HumanoidMessageTools

   private boolean acceptStatusChange(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName fromState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      HighLevelControllerName toState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      LogTools.debug("Controller state: {} to {}", fromState == null ? fromState : fromState.name(), toState == null ? toState : toState.name());
      return toState != null;
   }

   private void acceptWalkingStatus(WalkingStatusMessage message)
   {
      LogTools.debug("Walking status: {}", WalkingStatus.fromByte(message.getWalkingStatus()).name());
      if (message.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         while (!walkingCompletedNotifications.isEmpty())
         {
            walkingCompletedNotifications.remove(0).set(message);
         }
      }
   }

   public RemoteSyncedRobotModel newSyncedRobot()
   {
      return new RemoteSyncedRobotModel(robotModel, ros2Node); // TODO: Is using the existing robotModel okay?
   }

   public ROS2Callback<FootstepStatusMessage> createFootstepStatusCallback(Consumer<FootstepStatusMessage> consumer)
   {
      return new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, topicName.withOutput(), consumer);
   }

   public FootstepStatusMessage getLatestFootstepStatusMessage()
   {
      return footstepStatusMessage.get();
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan)
   {
      LogTools.debug("Tasking {} footstep(s) to the robot", footstepPlan.getFootstepDataList().size());

      controllerPublisherMap.publish(footstepPlan);

      TypedNotification<WalkingStatusMessage> walkingCompletedNotification = new TypedNotification<>();
      walkingCompletedNotifications.add(walkingCompletedNotification);
      return walkingCompletedNotification;
   }

   public void requestFootTrajectory(RobotSide robotSide, double trajectoryTime, FramePose3D footPose)
   {
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      footPose.get(position, orientation);
      requestFootTrajectory(robotSide, trajectoryTime, position, orientation);
   }

   public void requestFootTrajectory(RobotSide robotSide, double trajectoryTime, Point3D position, Quaternion orientation)
   {
      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
      footTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestFootTrajectory(footTrajectoryMessage);
   }

   public void requestFootLoadBearing(RobotSide robotSide, LoadBearingRequest loadBearingRequest)
   {
      FootLoadBearingMessage message = HumanoidMessageTools.createFootLoadBearingMessage(robotSide, loadBearingRequest);
      message.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestFootLoadBearing(message);
   }

   public void requestArmTrajectory(RobotSide robotSide, double trajectoryTime, double[] jointAngles)
   {
      ArmTrajectoryMessage armTrajectoryMessage = createArmTrajectoryMessage(robotSide, trajectoryTime, jointAngles);
      armTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestArmTrajectory(armTrajectoryMessage);
   }

   private final ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide side, double trajectoryTime, double[] jointAngles)
   {
      int numberOfArmJoints = jointMap.getArmJointNames().length;
      double[] jointAnglesAdjusted = Arrays.copyOfRange(jointAngles, 0, numberOfArmJoints);
      return HumanoidMessageTools.createArmTrajectoryMessage(side, trajectoryTime, jointAnglesAdjusted);
   }

   public void requestChestGoHome(double trajectoryTime)
   {
      GoHomeMessage chestGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, trajectoryTime);
      chestGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestGoHome(chestGoHomeMessage);
   }

   public void requestPelvisGoHome(double trajectoryTime)
   {
      GoHomeMessage pelvisGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime);
      pelvisGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestGoHome(pelvisGoHomeMessage);
   }

   public void requestChestOrientationTrajectory(double trajectoryTime, FrameQuaternion chestOrientation, ReferenceFrame dataFrame,
                                                 ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage chestOrientationMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                         chestOrientation,
                                                                                                         dataFrame,
                                                                                                         trajectoryFrame);
      chestOrientationMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestChestOrientationTrajectory(chestOrientationMessage);
   }

   public void pitchHeadDown()
   {
      pitchHeadWithRespectToChest(20.0);
   }

   public void pitchHeadWithRespectToChest(double headPitch)
   {
      syncedRobot.update();

      ReferenceFrame chestFrame = syncedRobot.getReferenceFrames().getChestFrame();
      FrameQuaternion headOrientation = new FrameQuaternion(chestFrame, 0.0, headPitch, 0.0);
      headOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      requestHeadOrientationTrajectory(1.0, headOrientation, ReferenceFrame.getWorldFrame(), syncedRobot.getReferenceFrames().getPelvisZUpFrame());
   }

   public void pitchHeadWithRespectToChest(double headPitch, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      ReferenceFrame chestFrame = humanoidReferenceFrames.getChestFrame();
      FrameQuaternion headOrientation = new FrameQuaternion(chestFrame, 0.0, headPitch, 0.0);
      headOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      requestHeadOrientationTrajectory(1.0, headOrientation, ReferenceFrame.getWorldFrame(), humanoidReferenceFrames.getPelvisZUpFrame());
   }

   public void requestHeadOrientationTrajectory(double trajectoryTime,
                                                Orientation3DReadOnly headOrientation,
                                                ReferenceFrame dataFrame,
                                                ReferenceFrame trajectoryFrame)
   {
      requestHeadOrientationTrajectory(trajectoryTime, headOrientation, MessageTools.toFrameId(dataFrame), MessageTools.toFrameId(trajectoryFrame));
   }

   /**
    * Obtain the frame id with MessageTools.toFrameId
    */
   public void requestHeadOrientationTrajectory(double trajectoryTime,
                                                Orientation3DReadOnly headOrientation,
                                                long dataFrameId,
                                                long trajectoryFrameId)
   {
      HeadTrajectoryMessage headOrientationMessage = new HeadTrajectoryMessage();
      Vector3DReadOnly desiredAngularVelocity = EuclidCoreTools.zeroVector3D;
      SO3TrajectoryPointMessage pointMessage = new SO3TrajectoryPointMessage();
      pointMessage.setTime(trajectoryTime);
      pointMessage.getOrientation().set(new Quaternion(headOrientation));
      pointMessage.getAngularVelocity().set(new Vector3D(desiredAngularVelocity));
      SO3TrajectoryMessage so3TrajectoryMessage = new SO3TrajectoryMessage();
      so3TrajectoryMessage.getTaskspaceTrajectoryPoints().add().set(pointMessage);
      so3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrameId);
      headOrientationMessage.getSo3Trajectory().set(so3TrajectoryMessage);
      headOrientationMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(dataFrameId);
      headOrientationMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestHeadOrientationTrajectory(headOrientationMessage);
   }

   public void requestPelvisOrientationTrajectory(double trajectoryTime, FrameQuaternion pelvisOrientation)
   {

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime,
                                                                                                                                            pelvisOrientation);
      pelvisOrientationTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestPelvisOrientationTrajectory(pelvisOrientationTrajectoryMessage);
   }

   public void requestPelvisTrajectory(double trajectoryTime, FramePoint3DReadOnly pelvisPosition, FrameQuaternionReadOnly pelvisOrientation)
   {
      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, pelvisPosition, pelvisOrientation);
      pelvisTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      requestPelvisTrajectory(pelvisTrajectoryMessage);
   }

   public void requestFootTrajectory(FootTrajectoryMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestFootLoadBearing(FootLoadBearingMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestArmTrajectory(ArmTrajectoryMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestChestOrientationTrajectory(ChestTrajectoryMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestHeadOrientationTrajectory(HeadTrajectoryMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestPelvisOrientationTrajectory(PelvisOrientationTrajectoryMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestPelvisTrajectory(PelvisTrajectoryMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void requestGoHome(GoHomeMessage message)
   {
      controllerPublisherMap.publish(message);
   }

   public void publishPose(Pose3D pose, double confidenceFactor, long timestamp)
   {
      StampedPosePacket stampedPosePacket = new StampedPosePacket();
      stampedPosePacket.pose_.set(pose);
      stampedPosePacket.setTimestamp(timestamp);
      stampedPosePacket.setConfidenceFactor(confidenceFactor);

      LogTools.debug("Publishing Pose " + pose + " with timestamp " + timestamp);
      publisherMap.publish(ROS2Tools.getPoseCorrectionTopic(robotName), stampedPosePacket);
   }

   public void pauseWalking()
   {
      LogTools.info("Sending pause walking to robot");
      PauseWalkingMessage pause = new PauseWalkingMessage();
      pause.setPause(true);
      controllerPublisherMap.publish(pause);
   }

   public boolean isRobotWalking()
   {
      HighLevelControllerName controllerState = getLatestControllerState();
      return (controllerState == HighLevelControllerName.WALKING);
   }

   public HighLevelControllerName getLatestControllerState()
   {
      return HighLevelControllerName.fromByte(controllerStateInput.getLatest().getEndHighLevelControllerName());
   }
}
