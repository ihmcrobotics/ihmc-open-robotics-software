package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;

import junit.framework.AssertionFailedError;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RampsGroundProfile;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitWalkingOverRampsTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      quadrupedTestFactory = createQuadrupedTestFactory();
   }
   
   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testWalkingOverShallowRamps(double comHeightForRoughTerrain) throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.075, 0.75, 1.2);
      
      walkOverRamps(groundProfile, comHeightForRoughTerrain);
   }

   public void testWalkingOverAggressiveRamps(double comHeightForRoughTerrain) throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.15, 0.75, 1.2);
      walkOverRamps(groundProfile, comHeightForRoughTerrain);
   }

   private void walkOverRamps(RampsGroundProfile groundProfile, double comHeightForRoughTerrain) throws IOException, AssertionFailedError
   {
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.05);
      stepTeleopManager.getXGaitSettings().setStanceLength(1.00);
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.30);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.35);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.1);
      stepTeleopManager.setDesiredBodyHeight(comHeightForRoughTerrain);
      stepTeleopManager.setStepSnapper((x, y, minZ) -> new Point3D(x, y, groundProfile.heightAt(x, y, 0.0)));

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      stepTeleopManager.requestXGait();
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 25.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(-0.5, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 25.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
   
   public void testWalkingDownSlope(QuadrupedInitialPositionParameters initialPosition) throws IOException
   {
      walkSlope(0.2, initialPosition);
   }

   public void testWalkingUpSlope(QuadrupedInitialPositionParameters initialPosition) throws IOException
   {
      walkSlope(-0.1, initialPosition);
   }

   private void walkSlope(double slope, QuadrupedInitialPositionParameters initialPosition) throws IOException, AssertionFailedError
   {
      InclinedGroundProfile groundProfile = new InclinedGroundProfile(slope);
      quadrupedTestFactory.setInitialPosition(initialPosition);
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseStateEstimator(false);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      stepTeleopManager.requestXGait();
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(getDesiredWalkingVelocity(), 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 7.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 2.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(-getDesiredWalkingVelocity(), 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 9.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }

   public abstract double getDesiredWalkingVelocity();
}
