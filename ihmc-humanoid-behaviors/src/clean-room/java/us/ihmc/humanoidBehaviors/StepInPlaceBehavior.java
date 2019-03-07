package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.tools.ActivationReference;
import us.ihmc.humanoidBehaviors.tools.FXUIMessagerAPIFactory;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class StepInPlaceBehavior
{
   private final FullHumanoidRobotModel fullRobotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final ActivationReference<Boolean> stepping;
   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);
   private final SideDependentList<FootstepStatusMessage> footstepStatus = new SideDependentList<>();
   private final MutableInt footstepsTaken = new MutableInt(2);

   public StepInPlaceBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      LogTools.debug("Initializing step in place behavior");

      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      fullRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node).getFullRobotModel();

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
         footstepsTaken.increment();
      }
   }

   private void stepInPlace()
   {
      if (stepping.pollActivated())
      {
         if (stepping.activationChanged())
         {
            LogTools.info("Starting to step");
         }

         if (footstepsTaken.getValue() >= 2)
         {
            footstepsTaken.setValue(0);

            LogTools.info("Sending steps");
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
      else if (stepping.activationChanged())
      {
         LogTools.info("Stopped stepping");
      }
   }

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(StepInPlaceBehavior.class);

      public static final Topic<Boolean> Stepping = apiFactory.createTopic("Stepping", Boolean.class);
      public static final Topic<Boolean> Abort = apiFactory.createTopic("Abort", Boolean.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
