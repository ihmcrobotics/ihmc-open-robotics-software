package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateAdjustFootstepMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateArmDesiredAccelerationsMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateArmTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateChestTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateFootLoadBearingMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateFootTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateFootstepDataListMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateGoHomeMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateHandTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateHeadTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateNeckDesiredAccelerationsMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateNeckTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validatePelvisHeightTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validatePelvisOrientationTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validatePelvisTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateSpineDesiredAccelerationsMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateSpineTrajectoryMessage;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.AdjustFootstepMessage;
import controller_msgs.msg.dds.ArmDesiredAccelerationsMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.AutomaticManipulationAbortMessage;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.CenterOfMassTrajectoryMessage;
import controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.ClearDelayQueueMessage;
import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.FootLoadBearingMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.JointDesiredOutputMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.ManipulationAbortedStatus;
import controller_msgs.msg.dds.MomentumTrajectoryMessage;
import controller_msgs.msg.dds.MultiContactBalanceStatus;
import controller_msgs.msg.dds.NeckDesiredAccelerationsMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.PlanOffsetStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import controller_msgs.msg.dds.SpineDesiredAccelerationsMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageValidator;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AutomaticManipulationAbortCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ClearDelayQueueCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ContactStateChangeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandWrenchTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeadTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PrepareForLocomotionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyJointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyMultiContactTrajectoryCommand;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;

public class ControllerAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> controllerSupportedCommands;
   private static final List<Class<? extends Settable<?>>> controllerSupportedStatusMessages;
   private static final HashSet<Class<?>> inputMessageClasses = new HashSet<>();
   private static final HashSet<Class<?>> outputMessageClasses = new HashSet<>();

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();

      /** Commands supported by bipedal walking controller {@link WalkingControllerState} */
      commands.add(ArmTrajectoryCommand.class);
      commands.add(HandTrajectoryCommand.class);
      commands.add(FootTrajectoryCommand.class);
      commands.add(HeadTrajectoryCommand.class);
      commands.add(NeckTrajectoryCommand.class);
      commands.add(NeckDesiredAccelerationsCommand.class);
      commands.add(ChestTrajectoryCommand.class);
      commands.add(SpineTrajectoryCommand.class);
      commands.add(PelvisTrajectoryCommand.class);
      commands.add(PelvisOrientationTrajectoryCommand.class);
      commands.add(PelvisHeightTrajectoryCommand.class);
      commands.add(StopAllTrajectoryCommand.class);
      commands.add(FootstepDataListCommand.class);
      commands.add(AdjustFootstepCommand.class);
      commands.add(GoHomeCommand.class);
      commands.add(FootLoadBearingCommand.class);
      commands.add(ArmDesiredAccelerationsCommand.class);
      commands.add(AutomaticManipulationAbortCommand.class);
      commands.add(HighLevelControllerStateCommand.class);
      commands.add(AbortWalkingCommand.class);
      commands.add(PrepareForLocomotionCommand.class);
      commands.add(PauseWalkingCommand.class);
      commands.add(SpineDesiredAccelerationsCommand.class);
      commands.add(HandLoadBearingCommand.class);
      commands.add(HandHybridJointspaceTaskspaceTrajectoryCommand.class);
      commands.add(HeadHybridJointspaceTaskspaceTrajectoryCommand.class);
      commands.add(ChestHybridJointspaceTaskspaceTrajectoryCommand.class);
      commands.add(ClearDelayQueueCommand.class);
      commands.add(MomentumTrajectoryCommand.class);
      commands.add(CenterOfMassTrajectoryCommand.class);
      commands.add(PlanarRegionsListCommand.class);
      commands.add(StepConstraintRegionCommand.class);
      commands.add(HandWrenchTrajectoryCommand.class);

      /** Commands supported by multi-contact controller, not in this repo */
      commands.add(WholeBodyMultiContactTrajectoryCommand.class);
      commands.add(ContactStateChangeCommand.class);

      /** Command supported by the joint-space controller {@link JointspacePositionControllerState} */
      commands.add(WholeBodyJointspaceTrajectoryCommand.class);

      controllerSupportedCommands = Collections.unmodifiableList(commands);
      controllerSupportedCommands.forEach(command -> inputMessageClasses.add(ROS2TopicNameTools.newMessageInstance(command).getMessageClass()));

      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();

      /** Statuses supported by bipedal walking controller {@link WalkingControllerState} */
      statusMessages.add(CapturabilityBasedStatus.class);
      statusMessages.add(FootstepStatusMessage.class);
      statusMessages.add(PlanOffsetStatus.class);
      statusMessages.add(WalkingStatusMessage.class);
      statusMessages.add(WalkingControllerFailureStatusMessage.class);
      statusMessages.add(ManipulationAbortedStatus.class);
      statusMessages.add(HighLevelStateChangeStatusMessage.class);
      statusMessages.add(TextToSpeechPacket.class);
      statusMessages.add(ControllerCrashNotificationPacket.class);
      statusMessages.add(JointspaceTrajectoryStatusMessage.class);
      statusMessages.add(TaskspaceTrajectoryStatusMessage.class);
      statusMessages.add(JointDesiredOutputMessage.class);
      statusMessages.add(RobotDesiredConfigurationData.class);

      /** Statuses supported by multi-contact controller, not in this repo */
      statusMessages.add(MultiContactBalanceStatus.class);

      controllerSupportedStatusMessages = Collections.unmodifiableList(statusMessages);
      outputMessageClasses.addAll(controllerSupportedStatusMessages);
   }

   public static List<Class<? extends Command<?, ?>>> getControllerSupportedCommands()
   {
      return controllerSupportedCommands;
   }

   public static HashSet<Class<?>> getROS2CommandMessageTypes()
   {
      return inputMessageClasses;
   }

   public static HashSet<Class<?>> getROS2StatusMessageTypes()
   {
      return inputMessageClasses;
   }

   public static List<Class<? extends Settable<?>>> getControllerSupportedStatusMessages()
   {
      return controllerSupportedStatusMessages;
   }

   public static MessageValidator createDefaultMessageValidation()
   {
      Map<Class<? extends Settable<?>>, MessageValidator> validators = new HashMap<>();
      validators.put(ArmTrajectoryMessage.class, message -> validateArmTrajectoryMessage((ArmTrajectoryMessage) message));
      validators.put(HandTrajectoryMessage.class, message -> validateHandTrajectoryMessage((HandTrajectoryMessage) message));
      validators.put(FootTrajectoryMessage.class, message -> validateFootTrajectoryMessage((FootTrajectoryMessage) message));
      validators.put(HeadTrajectoryMessage.class, message -> validateHeadTrajectoryMessage((HeadTrajectoryMessage) message));
      validators.put(NeckTrajectoryMessage.class, message -> validateNeckTrajectoryMessage((NeckTrajectoryMessage) message));
      validators.put(NeckDesiredAccelerationsMessage.class, message -> validateNeckDesiredAccelerationsMessage((NeckDesiredAccelerationsMessage) message));
      validators.put(ChestTrajectoryMessage.class, message -> validateChestTrajectoryMessage((ChestTrajectoryMessage) message));
      validators.put(SpineTrajectoryMessage.class, message -> validateSpineTrajectoryMessage((SpineTrajectoryMessage) message));
      validators.put(PelvisTrajectoryMessage.class, message -> validatePelvisTrajectoryMessage((PelvisTrajectoryMessage) message));
      validators.put(PelvisOrientationTrajectoryMessage.class,
                     message -> validatePelvisOrientationTrajectoryMessage((PelvisOrientationTrajectoryMessage) message));
      validators.put(PelvisHeightTrajectoryMessage.class, message -> validatePelvisHeightTrajectoryMessage((PelvisHeightTrajectoryMessage) message));
      validators.put(FootstepDataListMessage.class, message -> validateFootstepDataListMessage((FootstepDataListMessage) message));
      validators.put(AdjustFootstepMessage.class, message -> validateAdjustFootstepMessage((AdjustFootstepMessage) message));
      validators.put(GoHomeMessage.class, message -> validateGoHomeMessage((GoHomeMessage) message));
      validators.put(FootLoadBearingMessage.class, message -> validateFootLoadBearingMessage((FootLoadBearingMessage) message));
      validators.put(ArmDesiredAccelerationsMessage.class, message -> validateArmDesiredAccelerationsMessage((ArmDesiredAccelerationsMessage) message));
      validators.put(SpineDesiredAccelerationsMessage.class, message -> validateSpineDesiredAccelerationsMessage((SpineDesiredAccelerationsMessage) message));

      return new MessageValidator()
      {
         @Override
         public String validate(Object message)
         {
            MessageValidator validator = validators.get(message.getClass());
            return validator == null ? null : validator.validate(message);
         }
      };
   }

   public static MessageIDExtractor createDefaultMessageIDExtractor()
   {
      Map<Class<? extends Settable<?>>, MessageIDExtractor> extractors = new HashMap<>();
      extractors.put(ArmTrajectoryMessage.class, m -> ((ArmTrajectoryMessage) m).getSequenceId());
      extractors.put(HandTrajectoryMessage.class, m -> ((HandTrajectoryMessage) m).getSequenceId());
      extractors.put(FootTrajectoryMessage.class, m -> ((FootTrajectoryMessage) m).getSequenceId());
      extractors.put(HeadTrajectoryMessage.class, m -> ((HeadTrajectoryMessage) m).getSequenceId());
      extractors.put(NeckTrajectoryMessage.class, m -> ((NeckTrajectoryMessage) m).getSequenceId());
      extractors.put(NeckDesiredAccelerationsMessage.class, m -> ((NeckDesiredAccelerationsMessage) m).getSequenceId());
      extractors.put(ChestTrajectoryMessage.class, m -> ((ChestTrajectoryMessage) m).getSequenceId());
      extractors.put(SpineTrajectoryMessage.class, m -> ((SpineTrajectoryMessage) m).getSequenceId());
      extractors.put(PelvisTrajectoryMessage.class, m -> ((PelvisTrajectoryMessage) m).getSequenceId());
      extractors.put(PelvisOrientationTrajectoryMessage.class, m -> ((PelvisOrientationTrajectoryMessage) m).getSequenceId());
      extractors.put(PelvisHeightTrajectoryMessage.class, m -> ((PelvisHeightTrajectoryMessage) m).getSequenceId());
      extractors.put(StopAllTrajectoryMessage.class, m -> ((StopAllTrajectoryMessage) m).getSequenceId());
      extractors.put(FootstepDataListMessage.class, m -> ((FootstepDataListMessage) m).getSequenceId());
      extractors.put(AdjustFootstepMessage.class, m -> ((AdjustFootstepMessage) m).getSequenceId());
      extractors.put(GoHomeMessage.class, m -> ((GoHomeMessage) m).getSequenceId());
      extractors.put(FootLoadBearingMessage.class, m -> ((FootLoadBearingMessage) m).getSequenceId());
      extractors.put(ArmDesiredAccelerationsMessage.class, m -> ((ArmDesiredAccelerationsMessage) m).getSequenceId());
      extractors.put(AutomaticManipulationAbortMessage.class, m -> ((AutomaticManipulationAbortMessage) m).getSequenceId());
      extractors.put(HighLevelStateMessage.class, m -> ((HighLevelStateMessage) m).getSequenceId());
      extractors.put(AbortWalkingMessage.class, m -> ((AbortWalkingMessage) m).getSequenceId());
      extractors.put(PrepareForLocomotionMessage.class, m -> ((PrepareForLocomotionMessage) m).getSequenceId());
      extractors.put(PauseWalkingMessage.class, m -> ((PauseWalkingMessage) m).getSequenceId());
      extractors.put(SpineDesiredAccelerationsMessage.class, m -> ((SpineDesiredAccelerationsMessage) m).getSequenceId());
      extractors.put(HandLoadBearingMessage.class, m -> ((HandLoadBearingMessage) m).getSequenceId());
      extractors.put(HandHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((HandHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
      extractors.put(HeadHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((HeadHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
      extractors.put(ChestHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((ChestHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
      extractors.put(ClearDelayQueueMessage.class, m -> ((ClearDelayQueueMessage) m).getSequenceId());
      extractors.put(MomentumTrajectoryMessage.class, m -> ((MomentumTrajectoryMessage) m).getSequenceId());
      extractors.put(CenterOfMassTrajectoryMessage.class, m -> ((CenterOfMassTrajectoryMessage) m).getSequenceId());
      extractors.put(PlanarRegionsListMessage.class, m -> ((PlanarRegionsListMessage) m).getSequenceId());

      return new MessageIDExtractor()
      {
         @Override
         public long getMessageID(Object message)
         {
            MessageIDExtractor extractor = extractors.get(message.getClass());
            return extractor == null ? NO_ID : extractor.getMessageID(message);
         }
      };
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ROS2Tools.getControllerInputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ROS2Tools.getControllerOutputTopic(robotName);
   }

   public static <T> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      if (inputMessageClasses.contains(messageClass))
      {
         return getInputTopic(robotName).withTypeName(messageClass);
      }
      if (outputMessageClasses.contains(messageClass))
      {
         return getOutputTopic(robotName).withTypeName(messageClass);
      }

      throw new RuntimeException("Topic does not exist: " + messageClass);
   }
}
