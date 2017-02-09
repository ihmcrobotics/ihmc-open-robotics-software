package us.ihmc.quadrupedRobotics.controller.position;

import java.io.IOException;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedPositionTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedPositionCrawlWalkingForwardStoppingAndTurningTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedPositionTestYoVariables variables;
   
   @Before
   public void setup()
   {
      try
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
         ParameterRegistry.destroyAndRecreateInstance();
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.POSITION);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedPositionTestYoVariables(conductor.getScs());
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
   
   @ContinuousIntegrationTest(estimatedDuration = 700.0)
   @Test(timeout = 2000000)
   public void testWalkingForwardStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(1776L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for (int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(random.nextDouble() * 0.25);
         variables.getYoPlanarVelocityInputZ().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + random.nextDouble() * 5.0 + 15.0));
         conductor.simulate();
         
         variables.getYoPlanarVelocityInputX().set(0.0);
         variables.getYoPlanarVelocityInputZ().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
         conductor.simulate();
         
         variables.getYoPlanarVelocityInputX().set(0.0);
         variables.getYoPlanarVelocityInputZ().set(random.nextDouble() * 0.2 - 0.1);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + random.nextDouble() * 5.0 + 15.0));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
}
