package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitTurning720Test implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      try
      {
         ParameterRegistry.destroyAndRecreateInstance();
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 200000)
   public void rotate720InPlaceRight() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitStanceWidthInput().set(0.35);
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      variables.getYoPlanarVelocityInputZ().set(-0.5);

      int numSpins = 2;
      for (int i = 0; i < numSpins; i++)
      {
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 2.0, 1e-2));
         conductor.simulate();

         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), 0.0, 1e-2));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 200000)
   public void rotate720InPlaceLeft() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitStanceWidthInput().set(0.35);
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      variables.getYoPlanarVelocityInputZ().set(0.5);

      int numSpins = 2;
      for (int i = 0; i < numSpins; i++)
      {
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 2.0, 1e-2));
         conductor.simulate();

         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), 0.0, 1e-2));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
}
