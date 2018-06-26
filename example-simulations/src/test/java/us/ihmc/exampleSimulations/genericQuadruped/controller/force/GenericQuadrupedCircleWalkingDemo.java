package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestYoVariables;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

import java.io.IOException;

/**
 * Simulation of the generic quadruped walking in circles on flat ground.
 * Intended to be used to profile the quadruped controller, the test framework
 * is just used for convenience.
 */
public class GenericQuadrupedCircleWalkingDemo
{
   public static void main(String[] args)
   {
      try
      {
         GenericQuadrupedTestFactory quadrupedTestFactory = new GenericQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         GoalOrientedTestConductor conductor = quadrupedTestFactory.createTestConductor();
         QuadrupedTestYoVariables variables = new QuadrupedTestYoVariables(conductor.getScs());
         QuadrupedTeleopManager stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

         QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
         stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);
         double walkingSpeed = 0.25;
         double angularSpeed = 0.1;
         stepTeleopManager.requestXGait();
         stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, angularSpeed);
         conductor.addTerminalGoal(new YoVariableTestGoal()
         {
            @Override
            public boolean currentlyMeetsGoal()
            {
               return false;
            }

            @Override
            public String toString()
            {
               return "Unattainable terminal goal.";
            }
         });

         conductor.simulate();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
}
