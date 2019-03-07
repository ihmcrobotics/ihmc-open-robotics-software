package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class StepInPlaceBehavior
{
   private final FullHumanoidRobotModel fullRobotModel;

   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final AtomicReference<Boolean> stepping;

   public StepInPlaceBehavior(Messager messager, Ros2Node ros2Node, MessageTopicNameGenerator controllerTopicNameGenerator, DRCRobotModel robotModel)
   {
      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            controllerTopicNameGenerator);

      fullRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node).getFullRobotModel();

      stepping = messager.createInput(API.Stepping);

      PeriodicNonRealtimeThreadScheduler threadScheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
      threadScheduler.schedule(this::stepInPlace, 2, TimeUnit.SECONDS);
   }

   private void stepInPlace()
   {
      if (stepping.get())
      {
         FootstepDataListMessage footMessage = new FootstepDataListMessage();

         for (RobotSide side : RobotSide.values)
         {
            MovingReferenceFrame stepFrame = fullRobotModel.getSoleFrame(side);
            FramePoint3D footLocation = new FramePoint3D(stepFrame);
            FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrientation.changeFrame(ReferenceFrame.getWorldFrame());
            footMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation));
            footMessage.setAreFootstepsAdjustable(true);
         }

         footstepDataListPublisher.publish(footMessage);
      }
   }

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(StepInPlaceBehavior.class);

      public static final Topic<Boolean> Stepping = apiFactory.createTopic("Stepping", Boolean.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
