package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.avatar.jumpingSimulation.JumpingSimulationFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoal;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

public class AtlasJumpingSimulation
{
   private static double jumpLength = 0.5;
   private static double flightDuration = 0.25;
   private static double supportDuration = 0.3;
   private static final String parameterResourceName = "/us/ihmc/atlas/parameters/jumping_controller.xml";

   public static void main(String[] args)
   {
      List<Class<? extends Command<?, ?>> >availableCommands = new ArrayList<>();
      availableCommands.add(JumpingGoal.class);
      CommandInputManager commandInputManager = new CommandInputManager(availableCommands);

      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      AtlasSimInitialSetup initialSetup = new AtlasSimInitialSetup();
      JumpingSimulationFactory simulationFactory = new JumpingSimulationFactory(robotModel, initialSetup, commandInputManager);
      SimulationConstructionSet scs = simulationFactory.createSimulation(parameterResourceName);

      YoRegistry registry = scs.getRootRegistry();
      YoBoolean triggerJump = new YoBoolean("triggerJump", registry);
      addButton("ShouldBeSquatting", 1.0, scs);
      addButton("triggerJump", 1.0, scs);

      triggerJump.addListener(v ->
                              {
                                 if (triggerJump.getBooleanValue())
                                 {
                                    triggerJump.set(false, false);

                                    JumpingGoal jumpingGoal = new JumpingGoal();
                                    jumpingGoal.setGoalLength(jumpLength);
                                    jumpingGoal.setSupportDuration(supportDuration);
                                    jumpingGoal.setFlightDuration(flightDuration);
                                    commandInputManager.submitCommand(jumpingGoal);
                                 }
                              });

      scs.startOnAThread();
      scs.simulate();
   }

   private static void addButton(String yoVariableName, double newValue, SimulationConstructionSet scs)
   {
      YoRegistry registry = scs.getRootRegistry();

      final YoVariable var = registry.findVariable(yoVariableName);
      final JButton button = new JButton(yoVariableName);
      scs.addButton(button);
      button.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            var.setValueFromDouble(newValue);
         }
      });
   }
}
