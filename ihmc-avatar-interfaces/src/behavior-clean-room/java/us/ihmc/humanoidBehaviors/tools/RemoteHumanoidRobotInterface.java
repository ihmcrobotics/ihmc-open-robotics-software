package us.ihmc.humanoidBehaviors.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Function;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModule;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModuleLauncher;
import us.ihmc.communication.*;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

// TODO: Clean this up by using DRCUserInterfaceNetworkingManager (After cleaning that up first...)
public class RemoteHumanoidRobotInterface
{
   private final Ros2NodeInterface ros2Node;
   private final String robotName;
   private final DRCRobotJointMap jointMap;

   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher;
   private final IHMCROS2Publisher<FootTrajectoryMessage> footTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<FootLoadBearingMessage> footLoadBearingMessagePublisher;
   private final IHMCROS2Publisher<ArmTrajectoryMessage> armTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<ChestTrajectoryMessage> chestOrientationTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<HeadTrajectoryMessage> headOrientationTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<PelvisOrientationTrajectoryMessage> pelvisOrientationTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<PelvisTrajectoryMessage> pelvisTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<GoHomeMessage> goHomeMessagePublisher;
   private final IHMCROS2Publisher<StampedPosePacket> stampedPosePublisher;

   private final ArrayList<TypedNotification<WalkingStatusMessage>> walkingCompletedNotifications = new ArrayList<>();
   private final ROS2Input<HighLevelStateChangeStatusMessage> controllerStateInput;
   private final ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatusInput;

   private final RemoteSyncedHumanoidRobotState remoteSyncedHumanoidRobotState;

   private final FootstepPlanPostProcessingModule footstepPlanPostProcessingModule;

   public RemoteHumanoidRobotInterface(Ros2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      this.ros2Node = ros2Node;
      robotName = robotModel.getSimpleRobotName();
      jointMap = robotModel.getJointMap();

      ROS2ModuleIdentifier controllerId = ROS2Tools.HUMANOID_CONTROLLER;
      footTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, FootTrajectoryMessage.class, robotName, controllerId);
      footLoadBearingMessagePublisher = new IHMCROS2Publisher<>(ros2Node, FootLoadBearingMessage.class, robotName, controllerId);
      armTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, ArmTrajectoryMessage.class, robotName, controllerId);
      chestOrientationTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, ChestTrajectoryMessage.class, robotName, controllerId);
      headOrientationTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, HeadTrajectoryMessage.class, robotName, controllerId);
      pelvisOrientationTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, PelvisOrientationTrajectoryMessage.class, robotName, controllerId);
      pelvisTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, PelvisTrajectoryMessage.class, robotName, controllerId);
      goHomeMessagePublisher = new IHMCROS2Publisher<>(ros2Node, GoHomeMessage.class, robotName, controllerId);
      footstepDataListPublisher = new IHMCROS2Publisher<>(ros2Node, FootstepDataListMessage.class, robotName, controllerId);
      pausePublisher = new IHMCROS2Publisher<>(ros2Node, PauseWalkingMessage.class, robotName, controllerId);
      stampedPosePublisher = new IHMCROS2Publisher<>(ros2Node, StampedPosePacket.class, robotName, controllerId);

      new ROS2Callback<>(ros2Node, WalkingStatusMessage.class, robotName, controllerId, this::acceptWalkingStatus);

      HighLevelStateChangeStatusMessage initialState = new HighLevelStateChangeStatusMessage();
      initialState.setInitialHighLevelControllerName(HighLevelControllerName.DO_NOTHING_BEHAVIOR.toByte());
      initialState.setEndHighLevelControllerName(HighLevelControllerName.WALKING.toByte());
      controllerStateInput = new ROS2Input<>(ros2Node, HighLevelStateChangeStatusMessage.class, robotName, controllerId, initialState, this::acceptStatusChange);
      capturabilityBasedStatusInput = new ROS2Input<>(ros2Node, CapturabilityBasedStatus.class, robotName, controllerId);

      footstepPlanPostProcessingModule = FootstepPlanPostProcessingModuleLauncher.createModule(robotModel);

      remoteSyncedHumanoidRobotState = new RemoteSyncedHumanoidRobotState(robotModel, ros2Node);
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

   public ROS2Callback createFootstepStatusCallback(Consumer<FootstepStatusMessage> consumer)
   {
      ROS2Callback<FootstepStatusMessage> ros2Callback
            = new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, robotName, ROS2Tools.HUMANOID_CONTROLLER, consumer);
//      ros2Callbacks.add(ros2Callback); // TODO: Use ManagedROS2Node
      return ros2Callback;
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan,
                                                              HumanoidReferenceFrames humanoidReferenceFrames,
                                                              PlanarRegionsList planarRegionsList)
   {
      FramePose3D leftFootPose = new FramePose3D();
      FramePose3D rightFootPose = new FramePose3D();
      leftFootPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(RobotSide.LEFT));
      rightFootPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(RobotSide.RIGHT));
      CapturabilityBasedStatus latestCapturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();

      FootstepPostProcessingPacket footstepPostProcessingPacket = new FootstepPostProcessingPacket();
      footstepPostProcessingPacket.getFootstepDataList().set(footstepPlan);
      footstepPostProcessingPacket.getLeftFootPositionInWorld().set(leftFootPose.getPosition());
      footstepPostProcessingPacket.getLeftFootOrientationInWorld().set(leftFootPose.getOrientation());
      footstepPostProcessingPacket.getRightFootPositionInWorld().set(rightFootPose.getPosition());
      footstepPostProcessingPacket.getRightFootOrientationInWorld().set(rightFootPose.getOrientation());
      footstepPostProcessingPacket.getLeftFootContactPoints2d().set(latestCapturabilityBasedStatus.getLeftFootSupportPolygon2d());
      footstepPostProcessingPacket.getRightFootContactPoints2d().set(latestCapturabilityBasedStatus.getRightFootSupportPolygon2d());
      footstepPostProcessingPacket.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      footstepPlanPostProcessingModule.handleRequestPacket(footstepPostProcessingPacket);

      return requestWalk(footstepPlan);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan)
   {
      LogTools.debug("Tasking {} footstep(s) to the robot", footstepPlan.getFootstepDataList().size());
      footstepDataListPublisher.publish(footstepPlan);

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
      HumanoidRobotState referenceFrames = pollHumanoidRobotState();

      ReferenceFrame chestFrame = referenceFrames.getChestFrame();
      FrameQuaternion headOrientation = new FrameQuaternion(chestFrame, 0.0, headPitch, 0.0);
      headOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      requestHeadOrientationTrajectory(1.0, headOrientation, ReferenceFrame.getWorldFrame(), referenceFrames.getPelvisZUpFrame());
   }

   public void requestHeadOrientationTrajectory(double trajectoryTime, FrameQuaternion headOrientation, ReferenceFrame dataFrame,
                                                ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage headOrientationMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime,
                                                                                                      headOrientation,
                                                                                                      dataFrame,
                                                                                                      trajectoryFrame);
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
      footTrajectoryMessagePublisher.publish(message);
   }

   public void requestFootLoadBearing(FootLoadBearingMessage message)
   {
      footLoadBearingMessagePublisher.publish(message);
   }

   public void requestArmTrajectory(ArmTrajectoryMessage message)
   {
      armTrajectoryMessagePublisher.publish(message);
   }

   public void requestChestOrientationTrajectory(ChestTrajectoryMessage message)
   {
      chestOrientationTrajectoryMessagePublisher.publish(message);
   }

   public void requestHeadOrientationTrajectory(HeadTrajectoryMessage message)
   {
      headOrientationTrajectoryMessagePublisher.publish(message);
   }

   public void requestPelvisOrientationTrajectory(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      pelvisOrientationTrajectoryMessagePublisher.publish(pelvisOrientationTrajectoryMessage);
   }

   public void requestPelvisTrajectory(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      pelvisTrajectoryMessagePublisher.publish(pelvisTrajectoryMessage);
   }

   public void requestGoHome(GoHomeMessage goHomeMessage)
   {
      goHomeMessagePublisher.publish(goHomeMessage);
   }

   public void publishPose(Pose3D pose, double confidenceFactor, long timestamp)
   {
      StampedPosePacket stampedPosePacket = new StampedPosePacket();
      stampedPosePacket.pose_.set(pose);
      stampedPosePacket.setTimestamp(timestamp);
      stampedPosePacket.setConfidenceFactor(confidenceFactor);

      LogTools.debug("Publishing Pose " + pose + " with timestamp " + timestamp);
      stampedPosePublisher.publish(stampedPosePacket);
   }

   public void pauseWalking()
   {
      LogTools.debug("Sending pause walking to robot");
      PauseWalkingMessage pause = new PauseWalkingMessage();
      pause.setPause(true);
      pausePublisher.publish(pause);
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

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      return getRobotState().pollFullRobotModel();
   }

   public HumanoidRobotState pollHumanoidRobotState()
   {
      return getRobotState().pollHumanoidRobotState();
   }

   public FramePose3DReadOnly quickPollPoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      return getRobotState().quickPollPoseReadOnly(frameSelector);
   }

   public RemoteSyncedHumanoidRobotState getRobotState()
   {
      return remoteSyncedHumanoidRobotState;
   }
}
