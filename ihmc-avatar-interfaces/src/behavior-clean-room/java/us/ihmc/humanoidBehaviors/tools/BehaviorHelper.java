package us.ihmc.humanoidBehaviors.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Function;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.MutableTriple;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
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
import us.ihmc.humanoidBehaviors.RemoteREAInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;
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
 *
 * Open question: Trust vs. power vs. safety for/from behavior authors
 *
 * Robot:
 * - Command-only
 * - Status-only
 * - Interactive (footstep completion, hand trajectory completion, etc.)
 *
 * UI Communication.
 *
 * Toolbox comms:
 * - REA Input/Output
 * - Footstep planner
 *
 * Helper tools (threading, etc.)
 */
public class BehaviorHelper
{
   private final ManagedMessager messager;
   private final DRCRobotModel robotModel;
   private final Ros2Node ros2Node;

   private final String robotName;
   private final DRCRobotJointMap jointMap;

   private final RemoteHumanoidRobotInterface remoteHumanoidRobotInterface;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;
   private final RemoteREAInterface remoteREAInterface;

   private final List<ROS2Callback> ros2Callbacks = new ArrayList<>();

   public BehaviorHelper(DRCRobotModel robotModel, ManagedMessager messager, Ros2Node ros2Node)
   {
      this.messager = messager;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;

      robotName = robotModel.getSimpleRobotName();
      jointMap = robotModel.getJointMap();

      // TODO: Remove all this construction until needed

      // TODO: Create enable/disable support for these
      remoteHumanoidRobotInterface = new RemoteHumanoidRobotInterface(ros2Node, robotModel); // robot commands, status, interaction

      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, messager); // planner toolbox
      remoteREAInterface = new RemoteREAInterface(ros2Node); // REA toolbox

      // TODO: Extract UI comms class

      // TODO: Make accessors to classes; interface?
   }

   // Construction-only methods:

   // TODO

   // Robot Command Methods:

   public void publishPose(Pose3D pose, double confidenceFactor, long timestamp)
   {
      remoteHumanoidRobotInterface.publishPose(pose, confidenceFactor, timestamp);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepList)
   {
      return remoteHumanoidRobotInterface.requestWalk(footstepList);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan,
                                                              HumanoidReferenceFrames humanoidReferenceFrames,
                                                              boolean swingOverPlanarRegions,
                                                              PlanarRegionsList planarRegionsList)
   {
      return remoteHumanoidRobotInterface.requestWalk(footstepPlan, humanoidReferenceFrames, swingOverPlanarRegions, planarRegionsList);
   }

   public TypedNotification<WalkingStatusMessage> requestWalkWithSwingOvers(FootstepDataListMessage footstepPlan,
                                                                            HumanoidReferenceFrames humanoidReferenceFrames,
                                                                            PlanarRegionsList planarRegionsList)
   {
      return remoteHumanoidRobotInterface.requestWalk(footstepPlan, humanoidReferenceFrames, true, planarRegionsList);
   }

   public void pauseWalking()
   {
      remoteHumanoidRobotInterface.pauseWalking();
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
      remoteHumanoidRobotInterface.requestFootTrajectory(footTrajectoryMessage);
   }

   public void requestFootLoadBearing(RobotSide robotSide, LoadBearingRequest loadBearingRequest)
   {
      FootLoadBearingMessage message = HumanoidMessageTools.createFootLoadBearingMessage(robotSide, loadBearingRequest);
      message.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestFootLoadBearing(message);
   }

   public void requestArmTrajectory(RobotSide robotSide, double trajectoryTime, double[] jointAngles)
   {
      ArmTrajectoryMessage armTrajectoryMessage = createArmTrajectoryMessage(robotSide, trajectoryTime, jointAngles);
      armTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestArmTrajectory(armTrajectoryMessage);
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
      remoteHumanoidRobotInterface.requestGoHome(chestGoHomeMessage);
   }

   public void requestPelvisGoHome(double trajectoryTime)
   {
      GoHomeMessage pelvisGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime);
      pelvisGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestGoHome(pelvisGoHomeMessage);
   }

   public void requestChestOrientationTrajectory(double trajectoryTime, FrameQuaternion chestOrientation, ReferenceFrame dataFrame,
                                                 ReferenceFrame trajectoryFrame)
   {
      ChestTrajectoryMessage chestOrientationMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                         chestOrientation,
                                                                                                         dataFrame,
                                                                                                         trajectoryFrame);
      chestOrientationMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestChestOrientationTrajectory(chestOrientationMessage);
   }

   public void requestHeadOrientationTrajectory(double trajectoryTime, FrameQuaternion headOrientation, ReferenceFrame dataFrame,
                                                 ReferenceFrame trajectoryFrame)
   {
      HeadTrajectoryMessage headOrientationMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime,
                                                                                                         headOrientation,
                                                                                                         dataFrame,
                                                                                                         trajectoryFrame);
      headOrientationMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestHeadOrientationTrajectory(headOrientationMessage);
   }

   public void requestPelvisOrientationTrajectory(double trajectoryTime, FrameQuaternion pelvisOrientation)
   {

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime,
                                                                                                                                            pelvisOrientation);
      pelvisOrientationTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestPelvisOrientationTrajectory(pelvisOrientationTrajectoryMessage);
   }

   public void requestPelvisTrajectory(double trajectoryTime, FramePoint3DReadOnly pelvisPosition, FrameQuaternionReadOnly pelvisOrientation)
   {
      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, pelvisPosition, pelvisOrientation);
      pelvisTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      remoteHumanoidRobotInterface.requestPelvisTrajectory(pelvisTrajectoryMessage);
   }

   // Robot Action Callback and Polling Methods:

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      return remoteHumanoidRobotInterface.getRobotState().pollFullRobotModel();
   }

   public HumanoidRobotState pollHumanoidRobotState()
   {
      return remoteHumanoidRobotInterface.getRobotState().pollHumanoidRobotState();
   }

   public FramePose3DReadOnly quickPollPoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      return remoteHumanoidRobotInterface.getRobotState().quickPollPoseReadOnly(frameSelector);
   }

   public ROS2Callback createFootstepStatusCallback(Consumer<FootstepStatusMessage> consumer)
   {
      ROS2Callback<FootstepStatusMessage> ros2Callback
            = new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, robotModel.getSimpleRobotName(), ROS2Tools.HUMANOID_CONTROLLER, consumer);
      ros2Callbacks.add(ros2Callback);
      return ros2Callback;
   }

   public HighLevelControllerName getLatestControllerState()
   {
      return remoteHumanoidRobotInterface.latestControllerState();
   }

   public boolean isRobotWalking()
   {
      HighLevelControllerName controllerState = getLatestControllerState();
      return (controllerState == HighLevelControllerName.WALKING);
   }

   // RobotEnvironmentAwareness Methods:

   public PlanarRegionsList getLatestPlanarRegionList()
   {
      return remoteREAInterface.getLatestPlanarRegionList();
   }

   public PlanarRegionsListMessage getLatestPlanarRegionListMessage()
   {
      return remoteREAInterface.getLatestPlanarRegionListMessage();
   }

   public void clearREA()
   {
      remoteREAInterface.clearREA();
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

   // UI Communication Methods:

   public <T> void publishToUI(Topic<T> topic, T message)
   {
      messager.publish(topic, message);
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic)
   {
      return messager.createBooleanActivationReference(topic);
   }

   public <T> void createUICallback(Topic<T> topic, TopicListener<T> listener)
   {
      messager.registerCallback(topic, listener);
   }

   public <T> AtomicReference<T> createUIInput(Topic<T> topic, T initialValue)
   {
      return messager.createInput(topic, initialValue);
   }

   // Thread and Schedule Methods:
   // TODO: Track and auto start/stop threads?

   public PausablePeriodicThread createPausablePeriodicThread(Class<?> clazz, double period, Runnable runnable)
   {
      return createPausablePeriodicThread(clazz.getSimpleName(), period, 0, runnable);
   }

   public PausablePeriodicThread createPausablePeriodicThread(Class<?> clazz, double period, int crashesBeforeGivingUp, Runnable runnable)
   {
      return createPausablePeriodicThread(clazz.getSimpleName(), period, crashesBeforeGivingUp, runnable);
   }

   public PausablePeriodicThread createPausablePeriodicThread(String name, double period, int crashesBeforeGivingUp, Runnable runnable)
   {
      return new PausablePeriodicThread(name, period, crashesBeforeGivingUp, runnable);
   }

   // Behavior Helper Stuff:

   // TODO: Extract to behavior manager in general?
   public void setCommunicationCallbacksEnabled(boolean enabled)
   {
      for (ROS2Callback ros2Callback : ros2Callbacks)
      {
         ros2Callback.setEnabled(enabled);
      }
      messager.setEnabled(enabled);
   }

   public ManagedMessager getMessager()
   {
      return messager;
   }
}
