package us.ihmc.zulu.scriptCommandGenerator;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.ConcurrentLinkedQueue;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.scriptCommandGenerator.ExerciseAndJUnitScript;
import us.ihmc.avatar.scriptCommandGenerator.ScriptBasedControllerCommandGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.ZuluVersion;

public class ExerciseAndJUnitScriptCommandsTest
{
   @Test
   public void testAllScriptsSubmitControllerCommands()
   {
      FullHumanoidRobotModel fullRobotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS).createFullRobotModel();
      ConcurrentLinkedQueue<Command<?, ?>> controllerCommands = new ConcurrentLinkedQueue<>();
      ScriptBasedControllerCommandGenerator generator = new ScriptBasedControllerCommandGenerator(controllerCommands, fullRobotModel);
      ReferenceFrame referenceFrame = fullRobotModel.getSoleFrame(RobotSide.LEFT);

      for (ExerciseAndJUnitScript script : ExerciseAndJUnitScript.values())
      {
         controllerCommands.clear();
         generator.runExerciseScript(script, referenceFrame);

         if (script == ExerciseAndJUnitScript.SIMPLE_SINGLE_FOOT_TRAJECTORY_SCRIPT
             || script == ExerciseAndJUnitScript.SIMPLE_SINGLE_HAND_TRAJECTORY_SCRIPT
             || script == ExerciseAndJUnitScript.SIMPLE_SINGLE_PELVIS_HEIGHT_SCRIPT)
         {
            assertTrue(controllerCommands.isEmpty(), "Trajectory scripts are not yet hardcoded: " + script);
            continue;
         }

         assertFalse(controllerCommands.isEmpty(), "Expected commands for script " + script);
         assertTrue(controllerCommands.stream().anyMatch(command -> command instanceof FootstepDataListCommand
                                                                      || command instanceof PauseWalkingCommand),
                    "Expected footstep or pause commands for script " + script);
      }
   }
}
