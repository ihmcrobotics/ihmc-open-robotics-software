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
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageFilter;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageValidator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AutomaticManipulationAbortCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ClearDelayQueueCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandHybridJointspaceTaskspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanOffsetStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;

public abstract class ControllerAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> supportedCommands;
   private static final List<Class<? extends Packet<?>>> supportedStatusMessages;

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
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

      supportedCommands = Collections.unmodifiableList(commands);

      List<Class<? extends Packet<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(CapturabilityBasedStatus.class);
      statusMessages.add(FootstepStatusMessage.class);
      statusMessages.add(PlanOffsetStatus.class);
      statusMessages.add(WalkingStatusMessage.class);
      statusMessages.add(WalkingControllerFailureStatusMessage.class);
      statusMessages.add(ManipulationAbortedStatus.class);
      statusMessages.add(HighLevelStateChangeStatusMessage.class);
      statusMessages.add(TextToSpeechPacket.class);
      statusMessages.add(RequestPlanarRegionsListMessage.class);

      supportedStatusMessages = Collections.unmodifiableList(statusMessages);
   }

   public static List<Class<? extends Command<?, ?>>> getControllerSupportedCommands()
   {
      return supportedCommands;
   }

   public static List<Class<? extends Packet<?>>> getControllerSupportedStatusMessages()
   {
      return supportedStatusMessages;
   }

   public static MessageValidator createDefaultMessageValidation()
   {
      Map<Class<? extends Packet<?>>, MessageValidator> validators = new HashMap<>();
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
         public String validate(Packet<?> message)
         {
            MessageValidator validator = validators.get(message.getClass());
            return validator == null ? null : validator.validate(message);
         }
      };
   }
}
