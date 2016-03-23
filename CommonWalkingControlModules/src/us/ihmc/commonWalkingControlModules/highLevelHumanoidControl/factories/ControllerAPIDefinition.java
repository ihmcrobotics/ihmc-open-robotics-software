package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AbortWalkingCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmDesiredAccelerationsCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AutomaticManipulationAbortCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ChestTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.Command;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.EndEffectorLoadBearingCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootstepDataListCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.GoHomeCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandComplianceControlParametersCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HeadTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HighLevelStateCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PauseWalkingCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisHeightTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.StopAllTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.WholeBodyTrajectoryCommand;

public abstract class ControllerAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> supportedCommands;

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ArmTrajectoryCommand.class);
      commands.add(HandTrajectoryCommand.class);
      commands.add(FootTrajectoryCommand.class);
      commands.add(HeadTrajectoryCommand.class);
      commands.add(ChestTrajectoryCommand.class);
      commands.add(PelvisTrajectoryCommand.class);
      commands.add(PelvisOrientationTrajectoryCommand.class);
      commands.add(PelvisHeightTrajectoryCommand.class);
      commands.add(StopAllTrajectoryCommand.class);
      commands.add(FootstepDataListCommand.class);
      commands.add(GoHomeCommand.class);
      commands.add(EndEffectorLoadBearingCommand.class);
      commands.add(ArmDesiredAccelerationsCommand.class);
      commands.add(AutomaticManipulationAbortCommand.class);
      commands.add(HandComplianceControlParametersCommand.class);
      commands.add(HighLevelStateCommand.class);
      commands.add(AbortWalkingCommand.class);
      commands.add(PauseWalkingCommand.class);
      commands.add(WholeBodyTrajectoryCommand.class);

      supportedCommands = Collections.unmodifiableList(commands);
   }

   public static List<Class<? extends Command<?, ?>>> getControllerSupportedCommands()
   {
      return supportedCommands;
   }
}
