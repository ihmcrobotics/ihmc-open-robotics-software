package us.ihmc.quadrupedRobotics.controller.position;

import java.io.IOException;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
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
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedPositionCrawlWalkingWithStopsTest implements QuadrupedMultiRobotTestInterface
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
   
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingForwardFastWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(1888L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(0.14);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + random.nextDouble() * 3.0 + 5.0));
         conductor.simulate();
         
         variables.getYoPlanarVelocityInputX().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 120.0)
   @Test(timeout = 600000)
   public void testWalkingForwardSlowWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(1987L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(0.1);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + random.nextDouble() * 3.0 + 5.0));
         conductor.simulate();
         
         variables.getYoPlanarVelocityInputX().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingBackwardSlowWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(15567L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(-0.10);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + random.nextDouble() * 3.0 + 5.0));
         conductor.simulate();
         
         variables.getYoPlanarVelocityInputX().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 150.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT, IntegrationCategory.VIDEO})
   @Test(timeout = 600000)
   public void testWalkingBackwardFastWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(1889L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(-0.10);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + random.nextDouble() * 3.0 + 5.0));
         conductor.simulate();
         
         variables.getYoPlanarVelocityInputX().set(0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
         conductor.simulate();
      }
      
      conductor.concludeTesting();
   }
}
