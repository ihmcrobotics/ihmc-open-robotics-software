package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;

import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
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
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testWalkingForwardFast()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(1.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 8.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 2.0));
      conductor.simulate();
   }
   
   public void testWalkingForwardSlow()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(0.1);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 10.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 0.3));
      conductor.simulate();
   }
   
   public void testWalkingBackwardsFast()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-1.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 10.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), -2.0));
      conductor.simulate();
   }
   
   public void testWalkingBackwardsSlow()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-0.1);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 14.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), -0.4));
      conductor.simulate();
   }
   
   public void testWalkingInAForwardLeftCircle()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(0.6);
      variables.getYoPlanarVelocityInputZ().set(0.3);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 15.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.1));
      conductor.simulate();
   }
   
   public void testWalkingInAForwardRightCircle()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(0.6);
      variables.getYoPlanarVelocityInputZ().set(-0.3);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 15.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.1));
      conductor.simulate();
   }
   
   public void testWalkingInABackwardLeftCircle()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-0.6);
      variables.getYoPlanarVelocityInputZ().set(-0.3);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 15.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), -1.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.1));
      conductor.simulate();
   }
   
   public void testWalkingInABackwardRightCircle()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);

      variables.getXGaitEndPhaseShiftInput().set(90.0);
      variables.getStepTrigger().set(QuadrupedSteppingRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(-0.6);
      variables.getYoPlanarVelocityInputZ().set(0.3);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 15.0));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), -1.5));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.3));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.1));
      conductor.simulate();
   }
}
