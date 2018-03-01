package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.SettablePacket;
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
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanOffsetStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;

public abstract class ControllerAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> supportedCommands;
   private static final List<Class<? extends SettablePacket<?>>> supportedStatusMessages;

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

      List<Class<? extends SettablePacket<?>>> statusMessages = new ArrayList<>();
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

   public static List<Class<? extends SettablePacket<?>>> getControllerSupportedStatusMessages()
   {
      return supportedStatusMessages;
   }

}
