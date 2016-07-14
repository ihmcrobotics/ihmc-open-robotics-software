package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RampsGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public abstract class QuadrupedXGaitWalkingOverRampsTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.2, 0.7, 1.5);
      
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
      variables.getYoPlanarVelocityInputX().set(1.0);
      conductor.addTimeLimit(variables.getYoTime(), 15.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(-1.0);
      conductor.addTimeLimit(variables.getYoTime(), 15.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.1, 0.7, 1.2);
      
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      
      walkForwardsAndBack(quadrupedTestFactory);
   }

   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingDownRamp() throws IOException
   {
      InclinedGroundProfile groundProfile = new InclinedGroundProfile(0.2);
      
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      
      walkForwardsAndBack(quadrupedTestFactory);
   }
   
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingUpRamp() throws IOException
   {
      InclinedGroundProfile groundProfile = new InclinedGroundProfile(-0.1);
      
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      
      walkForwardsAndBack(quadrupedTestFactory);
   }

   private void walkForwardsAndBack(QuadrupedTestFactory quadrupedTestFactory) throws IOException, AssertionFailedError
   {
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
      variables.getYoPlanarVelocityInputX().set(1.0);
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoPlanarVelocityInputX().set(-1.0);
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
