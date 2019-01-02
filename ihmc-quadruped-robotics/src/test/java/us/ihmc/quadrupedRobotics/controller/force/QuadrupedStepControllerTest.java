package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedPlanning.input.NewQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.input.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;

import static org.junit.Assert.assertEquals;

public abstract class QuadrupedStepControllerTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private NewQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getNewStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 200000)
   public void testTakingAStep() throws SimulationExceededMaximumTimeException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      YoDouble frontLeftSolePositionX = (YoDouble) conductor.getScs().getVariable("frontLeftSolePositionX");
      double commandedStepPositionX = frontLeftSolePositionX.getDoubleValue() + 0.2;

      variables.getTimedStepQuadrant().set(RobotQuadrant.FRONT_LEFT);
      variables.getTimedStepGroundClearance().set(0.15);
      variables.getTimedStepDuration().set(0.6);
      variables.getTimedStepGoalPositionX().set(commandedStepPositionX);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_STEP);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 3.0));
      conductor.simulate();

      assertEquals("Didn't step to correct location", commandedStepPositionX, frontLeftSolePositionX.getDoubleValue(), 0.05);
   }
}
