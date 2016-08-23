package us.ihmc.quadrupedRobotics.controller.position;

import java.io.IOException;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedPositionTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

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
   
   @DeployableTestMethod(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingForwardFastWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(1888L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(0.2);
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
   
   @DeployableTestMethod(estimatedDuration = 120.0)
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
   
   @DeployableTestMethod(estimatedDuration = 150.0)
   @Test(timeout = 600000)
   public void testWalkingBackwardSlowWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(15567L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(-0.15);
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
   
   @DeployableTestMethod(estimatedDuration = 150.0, targets = {TestPlanTarget.InDevelopment, TestPlanTarget.Video})
   @Test(timeout = 600000)
   public void testWalkingBackwardFastWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      Random random = new Random(1889L);
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      for(int i = 0; i < 6; i++)
      {
         variables.getYoPlanarVelocityInputX().set(-0.2);
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
