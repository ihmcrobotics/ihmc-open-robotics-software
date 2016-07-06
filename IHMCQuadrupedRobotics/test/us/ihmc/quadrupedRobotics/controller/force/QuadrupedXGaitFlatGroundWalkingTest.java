package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public abstract class QuadrupedXGaitFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      conductor = createGoalOrientedTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
   }
   
   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardFast()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(1.0);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 5.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 2.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardSlow()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(0.1);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 5.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 0.3));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingBackwardsFast()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-1.0);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 5.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), -2.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingBackwardsSlow()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-0.1);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 6.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), -0.3));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInAForwardLeftCircle()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(1.0);
      variables.getYoPlanarVelocityInputZ().set(0.5);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 10.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.1));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInAForwardRightCircle()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(1.0);
      variables.getYoPlanarVelocityInputZ().set(-0.5);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 10.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.1));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInABackwardLeftCircle()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-1.0);
      variables.getYoPlanarVelocityInputZ().set(-0.5);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 10.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), -1.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.1));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 30000)
   public void testWalkingInABackwardRightCircle()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-1.0);
      variables.getYoPlanarVelocityInputZ().set(0.5);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.3));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 10.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), -1.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.1));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
