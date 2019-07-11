package us.ihmc.humanoidBehaviors.tools;

import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

public class BehaviorHelper
{
   private final Messager messager;
   private final DRCRobotModel robotModel;
   private final Ros2Node ros2Node;

   private final RemoteSyncedRobotModel remoteSyncedRobotModel;
   private final RemoteSyncedHumanoidFrames remoteSyncedHumanoidFrames;

   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;

   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   private PeriodicNonRealtimeThreadScheduler threadScheduler;

   public BehaviorHelper(Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      this.messager = messager;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;

      remoteSyncedRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node);
      remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidFrames(robotModel, ros2Node);

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      
      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, LIDARBasedREAModule.ROS2_ID);
   }

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

   public void publishFootstepList(FootstepDataListMessage footstepList)
   {
      footstepDataListPublisher.publish(footstepList);
   }

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

   public void createFootstepStatusCallback(Consumer<FootstepStatusMessage> consumer)
   {
      new ROS2Callback<>(ros2Node, FootstepStatusMessage.class, robotModel.getSimpleRobotName(), HighLevelHumanoidControllerFactory.ROS2_ID, consumer);
   }

   public ActivationReference<Boolean> createBooleanActivationReference(Topic<Boolean> topic, boolean initialValue, boolean activationValue)
   {
      return new ActivationReference<>(messager.createInput(topic, initialValue), activationValue);
   }

   public PlanarRegionsList getLatestPlanarRegionList()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest());
   }

   public PlanarRegionsListMessage getLatestPlanarRegionListMessage()
   {
      return planarRegionsList.getLatest();
   }

}
