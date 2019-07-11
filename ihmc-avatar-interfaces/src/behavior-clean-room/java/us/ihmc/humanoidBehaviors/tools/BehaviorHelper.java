package us.ihmc.humanoidBehaviors.tools;

import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

/**
 * Class for entry methods for developing robot behaviors. The idea is to have this be the one-stop shopping location
 * for everything one might want to do when creating a robot behavior. It should hide all of the network traffic. 
 * The methods should be a useful reminder to a behavior developer all of the things that one can do with the robots.
 * 
 * This class will likely get too large eventually and need to be refactored into several classes. But until that time 
 * comes it should contain everything for interacting with: the robot actions (taking steps and achieving poses), robot sensing (reference frames, etc.),
 * REA (getting planar regions), footstep planning, etc.
 *
 * At first we'll make this so that all of the things are created, even if you don't need them. But later, we'll make it so that
 * they are created as needed.
 * 
 * The main goal is to simplify and make clean the Behaviors themselves. The public interface of this class should be a joy to work with.
 * The internals and all the things it relies on might be a nightmare, but the public API should not be.
 */
public class BehaviorHelper
{
   private final Messager messager;
   private final DRCRobotModel robotModel;
   private final Ros2Node ros2Node;

   private final RemoteRobotControllerInterface remoteRobotControllerInterface;
   private final RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   private final RemoteSyncedRobotModel remoteSyncedRobotModel;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;

   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   private PeriodicNonRealtimeThreadScheduler threadScheduler;

   public BehaviorHelper(Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      this.messager = messager;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;

      remoteRobotControllerInterface = new RemoteRobotControllerInterface(ros2Node, robotModel);
      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, messager);

      remoteSyncedRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node);
      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, LIDARBasedREAModule.ROS2_ID);

      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, LIDARBasedREAModule.ROS2_ID);
   }

   // Robot Sensing Methods:

   public FullHumanoidRobotModel pollFullRobotModel()
   {
      return remoteSyncedRobotModel.pollFullRobotModel();
   }

   public HumanoidReferenceFrames pollHumanoidReferenceFrames()
   {
      return remoteSyncedHumanoidFrames.pollHumanoidReferenceFrames();
   }

   public FramePose3DReadOnly quickPollPoseReadOnly(Function<HumanoidReferenceFrames, ReferenceFrame> frameSelector)
   {
      return remoteSyncedHumanoidFrames.quickPollPoseReadOnly(frameSelector);
   }

   // Robot Action Methods:

   public void publishFootstepList(FootstepDataListMessage footstepList)
   {
      footstepDataListPublisher.publish(footstepList);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames,
                                                              Boolean swingOverPlanarRegions, PlanarRegionsList planarRegionsList)
   {
      return remoteRobotControllerInterface.requestWalk(footstepPlan, humanoidReferenceFrames, swingOverPlanarRegions, planarRegionsList);
   }

   public void pauseWalking()
   {
      remoteRobotControllerInterface.pauseWalking();
   }

   // Robot Action Callback and Polling Methods:

   public void createFootstepStatusCallback(Consumer<FootstepStatusMessage> consumer)
   {
      new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, robotModel.getSimpleRobotName(), HighLevelHumanoidControllerFactory.ROS2_ID, consumer);
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

}
