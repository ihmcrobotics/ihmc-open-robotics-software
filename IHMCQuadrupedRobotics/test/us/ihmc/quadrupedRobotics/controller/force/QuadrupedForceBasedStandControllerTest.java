package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public abstract class QuadrupedForceBasedStandControllerTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
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
   
   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 50000)
   public void testStandingUpAndAdjustingCoM()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      double stanceHeight = variables.getStanceHeight().getDoubleValue();
      
      variables.getYoComPositionInputZ().set(stanceHeight + 0.05);
      variables.getYoComPositionInputX().set(0.05);
      variables.getYoComPositionInputY().set(-0.05);
      variables.getYoBodyOrientationInputYaw().set(0.05);
      variables.getYoBodyOrientationInputPitch().set(0.05);
      variables.getYoBodyOrientationInputRoll().set(0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight + 0.0);
      variables.getYoComPositionInputX().set(-0.05);
      variables.getYoComPositionInputY().set(-0.05);
      variables.getYoBodyOrientationInputYaw().set(0.05);
      variables.getYoBodyOrientationInputPitch().set(-0.05);
      variables.getYoBodyOrientationInputRoll().set(0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight - 0.05);
      variables.getYoComPositionInputX().set(-0.05);
      variables.getYoComPositionInputY().set(0.05);
      variables.getYoBodyOrientationInputYaw().set(-0.05);
      variables.getYoBodyOrientationInputPitch().set(-0.05);
      variables.getYoBodyOrientationInputRoll().set(0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight - 0.1);
      variables.getYoComPositionInputX().set(0.05);
      variables.getYoComPositionInputY().set(0.05);
      variables.getYoBodyOrientationInputYaw().set(-0.05);
      variables.getYoBodyOrientationInputPitch().set(-0.05);
      variables.getYoBodyOrientationInputRoll().set(-0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight);
      variables.getYoComPositionInputX().set(0.0);
      variables.getYoComPositionInputY().set(0.0);
      variables.getYoBodyOrientationInputYaw().set(0.0);
      variables.getYoBodyOrientationInputPitch().set(0.0);
      variables.getYoBodyOrientationInputRoll().set(0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
   }
}
