package us.ihmc.atlas;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasMomentumOptimizationSettings;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jumpingSimulation.JumpingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoal;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
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
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public MomentumOptimizationSettings getMomentumOptimizationSettings()
               {
                  return getTestMomentumOptimizationSettings(getJointMap(), getContactPointParameters().getNumberOfContactableBodies());
               }
            };
         }
      };
      AtlasSimInitialSetup initialSetup = new AtlasSimInitialSetup();
      JumpingSimulationFactory simulationFactory = new JumpingSimulationFactory(robotModel, initialSetup);
      SimulationConstructionSet scs = simulationFactory.createSimulation(parameterResourceName);
      CommandInputManager commandInputManager = simulationFactory.getCommandInputManager();

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

   private static MomentumOptimizationSettings getTestMomentumOptimizationSettings(HumanoidJointNameMap jointMap, int numberOfContactableBodies)
   {
      double jointAccelerationWeight = 1e-10;
      double jointJerkWeight = 0.0;
      double jointTorqueWeight = 0.0;
      double maximumJointAcceleration = 1000.0;

      return new AtlasMomentumOptimizationSettings(jointMap, numberOfContactableBodies)
      {
         @Override
         public double getJointAccelerationWeight()
         {
            return jointAccelerationWeight;
         }

         @Override
         public double getJointJerkWeight()
         {
            return jointJerkWeight;
         }

         @Override
         public double getJointTorqueWeight()
         {
            return jointTorqueWeight;
         }

         @Override
         public double getMaximumJointAcceleration()
         {
            return maximumJointAcceleration;
         }

         @Override
         public boolean areJointVelocityLimitsConsidered()
         {
            return false;
         }
      };
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
