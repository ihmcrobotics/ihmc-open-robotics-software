package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitFlatGroundTrotTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         ParameterRegistry.destroyAndRecreateInstance();
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
   
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testTrottingForwardFast()
   {
      trotFast(1.0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testTrottingBackwardsFast()
   {
      trotFast(-1.0);
   }

   private void trotFast(double directionX) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(180.0);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.getYoPlanarVelocityInputX().set(directionX * 0.6);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      if(Math.signum(directionX) < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX * 2.0));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX * 2.0));
      }
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testTrottingForwardSlow()
   {
      trotSlow(1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testTrottingBackwardsSlow()
   {
      trotSlow(-1.0);
   }
   
   private void trotSlow(double directionX) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(180.0);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
//      variables.getXGaitEndDoubleSupportDurationInput().set(0.3);
      
      variables.getYoPlanarVelocityInputX().set(directionX * 0.1);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 6.0);
      
      if(Math.signum(directionX) < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX * 0.3));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX * 0.3));
      }
      
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testTrottingInAForwardLeftCircle()
   {
      trotInACircle(1.0, 1.0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testTrottingInAForwardRightCircle()
   {
      trotInACircle(1.0, -1.0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testTrottingInABackwardLeftCircle()
   {
      trotInACircle(-1.0, 1.0);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testTrottingInABackwardRightCircle()
   {
      trotInACircle(-1.0, -1.0);
   }

   private void trotInACircle(double directionX, double directionZ) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(180.0);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(directionX * 0.5);//reduced for real robot gains
      variables.getYoPlanarVelocityInputZ().set(directionZ * 0.25);//reduced for real robot gains
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      if(Math.signum(directionX) > 0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), directionX * 0.25));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), directionX * 0.25));
      }
      
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), directionZ * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), directionZ * Math.PI, 0.1));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
