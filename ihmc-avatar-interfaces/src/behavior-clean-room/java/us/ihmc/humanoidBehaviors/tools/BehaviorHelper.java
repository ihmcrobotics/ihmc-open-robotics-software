package us.ihmc.humanoidBehaviors.tools;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;

import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
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
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * Class for entry methods for developing robot behaviors. The idea is to have this be the one-stop
 * shopping location for everything one might want to do when creating a robot behavior. It should
 * hide all of the network traffic. The methods should be a useful reminder to a behavior developer
 * all of the things that one can do with the robots. This class will likely get too large
 * eventually and need to be refactored into several classes. But until that time comes it should
 * contain everything for interacting with: the robot actions (taking steps and achieving poses),
 * robot sensing (reference frames, etc.), REA (getting planar regions), footstep planning, etc. At
 * first we'll make this so that all of the things are created, even if you don't need them. But
 * later, we'll make it so that they are created as needed. The main goal is to simplify and make
 * clean the Behaviors themselves. The public interface of this class should be a joy to work with.
 * The internals and all the things it relies on might be a nightmare, but the public API should not
 * be.
 */
public class BehaviorHelper
{
   private final Messager messager;
   private final DRCRobotModel robotModel;
   private final DRCRobotJointMap drcRobotJointMap;
   private final Ros2Node ros2Node;

   private final RemoteRobotControllerInterface remoteRobotControllerInterface;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final RemoteSyncedRobotModel remoteSyncedRobotModel;
   private final RemoteSyncedHumanoidRobotState remoteSyncedHumanoidRobotState;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final IHMCROS2Publisher<StampedPosePacket> stampedPosePublisher;
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;

   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   private PeriodicNonRealtimeThreadScheduler threadScheduler;

   public BehaviorHelper(Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      this.messager = messager;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;

      drcRobotJointMap = robotModel.getJointMap();

      remoteRobotControllerInterface = new RemoteRobotControllerInterface(ros2Node, robotModel);
      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, messager);

      remoteSyncedRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node);
      remoteSyncedHumanoidRobotState = new RemoteSyncedHumanoidRobotState(robotModel, ros2Node);

      MessageTopicNameGenerator robotSubscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
      MessageTopicNameGenerator robotPublisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName());
                     
      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            robotSubscriberTopicNameGenerator);

      stampedPosePublisher = ROS2Tools.createPublisher(ros2Node,
                                                       StampedPosePacket.class,
                                                       robotSubscriberTopicNameGenerator);

      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, ROS2Tools.REA);

      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA);
   }

   // Robot Sensing Methods:

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      return remoteSyncedRobotModel.pollFullRobotModel();
   }

   public HumanoidRobotState pollHumanoidRobotState()
   {
      return remoteSyncedHumanoidRobotState.pollHumanoidRobotState();
   }

   public FramePose3DReadOnly quickPollPoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      return remoteSyncedHumanoidRobotState.quickPollPoseReadOnly(frameSelector);
   }

   // Robot Action Methods:

   public void publishFootstepList(FootstepDataListMessage footstepList)
   {
      footstepDataListPublisher.publish(footstepList);
   }

   public void publishPose(Pose3D pose, double confidenceFactor, long timestamp)
   {
      StampedPosePacket stampedPosePacket = new StampedPosePacket();
      stampedPosePacket.pose_.set(pose);
      stampedPosePacket.setTimestamp(timestamp);
      stampedPosePacket.setConfidenceFactor(confidenceFactor);

      LogTools.info("Publishing Pose " + pose + " with timestamp " + timestamp);
      stampedPosePublisher.publish(stampedPosePacket);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames,
                                                              Boolean swingOverPlanarRegions, PlanarRegionsList planarRegionsList)
   {
      return remoteRobotControllerInterface.requestWalk(footstepPlan, humanoidReferenceFrames, swingOverPlanarRegions, planarRegionsList);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames, PlanarRegionsList planarRegionsList)
   {
      return remoteRobotControllerInterface.requestWalk(footstepPlan, humanoidReferenceFrames, false, planarRegionsList);
   }

   public void pauseWalking()
   {
      remoteRobotControllerInterface.pauseWalking();
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
      remoteRobotControllerInterface.requestFootTrajectory(footTrajectoryMessage);
   }

   public void requestArmTrajectory(RobotSide robotSide, double trajectoryTime, double[] jointAngles)
   {
      ArmTrajectoryMessage armTrajectoryMessage = createArmTrajectoryMessage(robotSide, trajectoryTime, jointAngles);
      armTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestArmTrajectory(armTrajectoryMessage);
   }

   private final ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide side, double trajectoryTime, double[] jointAngles)
   {
      int numberOfArmJoints = drcRobotJointMap.getArmJointNames().length;
      double[] jointAnglesAdjusted = Arrays.copyOfRange(jointAngles, 0, numberOfArmJoints);
      return HumanoidMessageTools.createArmTrajectoryMessage(side, trajectoryTime, jointAnglesAdjusted);
   }

   public void requestChestGoHome(double trajectoryTime)
   {
      GoHomeMessage chestGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, trajectoryTime);
      chestGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestGoHome(chestGoHomeMessage);
   }

   public void requestPelvisGoHome(double trajectoryTime)
   {
      GoHomeMessage pelvisGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime);
      pelvisGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestGoHome(pelvisGoHomeMessage);
   }

   public void requestChestOrientationTrajectory(double trajectoryTime, FrameQuaternion chestOrientation, ReferenceFrame dataFrame,
                                                 ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage chestOrientationMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                         chestOrientation,
                                                                                                         dataFrame,
                                                                                                         trajectoryFrame);
      chestOrientationMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestChestOrientationTrajectory(chestOrientationMessage);
   }

   public void requestHeadOrientationTrajectory(double trajectoryTime, FrameQuaternion headOrientation, ReferenceFrame dataFrame,
                                                 ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage headOrientationMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime,
                                                                                                         headOrientation,
                                                                                                         dataFrame,
                                                                                                         trajectoryFrame);
      headOrientationMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestHeadOrientationTrajectory(headOrientationMessage);
   }

   public void requestPelvisOrientationTrajectory(double trajectoryTime, FrameQuaternion pelvisOrientation)
   {

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime,
                                                                                                                                            pelvisOrientation);
      pelvisOrientationTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestPelvisOrientationTrajectory(pelvisOrientationTrajectoryMessage);
   }

   public void requestPelvisTrajectory(double trajectoryTime, FramePoint3DReadOnly pelvisPosition, FrameQuaternionReadOnly pelvisOrientation)
   {
      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, pelvisPosition, pelvisOrientation);
      pelvisTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteRobotControllerInterface.requestPelvisTrajectory(pelvisTrajectoryMessage);
   }

   // Robot Action Callback and Polling Methods:

   public void createFootstepStatusCallback(Consumer<FootstepStatusMessage> consumer)
   {
      new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.HUMANOID_CONTROLLER, consumer);
   }

   public HighLevelControllerName getLatestControllerState()
   {
      return remoteRobotControllerInterface.latestControllerState();
   }

   // RobotEnvironmentAwareness Methods:

   public PlanarRegionsList getLatestPlanarRegionList()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest());
   }

   public PlanarRegionsListMessage getLatestPlanarRegionListMessage()
   {
      return planarRegionsList.getLatest();
   }

   public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }

   // Planning Methods:

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start, FramePose3DReadOnly goal,
                                                                     PlanarRegionsList planarRegionsList)
   {
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      return requestPlan(start, goal, planarRegionsListMessage);
   }

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start, FramePose3DReadOnly goal,
                                                                     PlanarRegionsListMessage planarRegionsListMessage)
   {
      return remoteFootstepPlannerInterface.requestPlan(start, goal, planarRegionsListMessage);
   }

   public void abortPlanning()
   {
      remoteFootstepPlannerInterface.abortPlanning();
   }

   // General Helper Methods:

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic, boolean initialValue, boolean activationValue)
   {
      return new ActivationReference<>(messager.createInput(topic, initialValue), activationValue);
   }

   // Thread and Schedule Methods:

   public void startScheduledThread(String simpleName, Runnable runnable, long period, TimeUnit timeUnit)
   {
      threadScheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      threadScheduler.schedule(runnable, period, timeUnit);
   }

   public void shutdownScheduledThread()
   {
      if (threadScheduler != null)
      {
         threadScheduler.shutdown();
      }
   }

   public boolean isRobotWalking()
   {
      HighLevelControllerName controllerState = getLatestControllerState();
      return (controllerState == HighLevelControllerName.WALKING);
   }


}
