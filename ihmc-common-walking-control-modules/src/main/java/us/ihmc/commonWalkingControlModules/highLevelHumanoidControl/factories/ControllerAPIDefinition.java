package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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
      commands.add(HandComplianceControlParametersCommand.class);
      commands.add(HighLevelControllerStateCommand.class);
      commands.add(AbortWalkingCommand.class);
      commands.add(PrepareForLocomotionCommand.class);
      commands.add(PauseWalkingCommand.class);
      commands.add(WholeBodyTrajectoryCommand.class);
      commands.add(SpineDesiredAccelerationCommand.class);
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
      statusMessages.add(FootstepStatus.class);
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
