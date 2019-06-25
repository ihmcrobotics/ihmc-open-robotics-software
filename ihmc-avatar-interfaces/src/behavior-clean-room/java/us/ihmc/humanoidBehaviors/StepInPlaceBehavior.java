package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class StepInPlaceBehavior
{
   private final RemoteSyncedRobotModel remoteSyncedRobotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final ActivationReference<Boolean> stepping;
   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);
   private final SideDependentList<FootstepStatusMessage> footstepStatus = new SideDependentList<>();
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);

   public StepInPlaceBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      LogTools.debug("Initializing step in place behavior");

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      remoteSyncedRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           FootstepStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           s -> consumeFootstepStatus(s.takeNextData()));

      stepping = new ActivationReference<>(messager.createInput(API.Stepping, false), true);

      PeriodicNonRealtimeThreadScheduler threadScheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      threadScheduler.schedule(this::stepInPlace, 1, TimeUnit.SECONDS);

      messager.registerTopicListener(API.Abort, abort -> {
         if (abort)
         {
            threadScheduler.shutdown();
         }
      });
   }

   private void consumeFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         footstepsTaken.incrementAndGet();
      }
   }

   private void stepInPlace()
   {
      if (stepping.poll())
      {
         if (stepping.hasChanged())
         {
            LogTools.info("Starting to step");
         }

         if (footstepsTaken.compareAndSet(2, 0))
         {
            LogTools.info("Sending steps");

            FullHumanoidRobotModel fullRobotModel = remoteSyncedRobotModel.pollFullRobotModel();

            FootstepDataListMessage footstepList = new FootstepDataListMessage();

            for (RobotSide side : RobotSide.values)
            {
               MovingReferenceFrame stepFrame = fullRobotModel.getSoleFrame(side);
               FramePoint3D footLocation = new FramePoint3D(stepFrame);
               FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
               footLocation.changeFrame(ReferenceFrame.getWorldFrame());
               footOrientation.changeFrame(ReferenceFrame.getWorldFrame());
               footstepList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation));
               footstepList.setAreFootstepsAdjustable(true);
            }

            footstepDataListPublisher.publish(footstepList);
         }
      }
      else if (stepping.hasChanged())
      {
         LogTools.info("Stopped stepping");
      }
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("StepInPlaceBehavior");
      private static final CategoryTheme StepInPlace = apiFactory.createCategoryTheme("StepInPlace");

      public static final Topic<Boolean> Stepping = Root.child(StepInPlace).topic(apiFactory.createTypedTopicTheme("Stepping"));
      public static final Topic<Boolean> Abort = Root.child(StepInPlace).topic(apiFactory.createTypedTopicTheme("Abort"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
