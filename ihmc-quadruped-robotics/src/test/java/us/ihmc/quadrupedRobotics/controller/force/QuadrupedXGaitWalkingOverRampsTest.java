package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RampsGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitWalkingOverRampsTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
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

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2200000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.075, 0.75, 1.2);

      walkOverRamps(groundProfile, getComHeightForRoughTerrain());
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 2000000)
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.15, 0.75, 1.2);
      walkOverRamps(groundProfile, getComHeightForRoughTerrain());
   }

   private void walkOverRamps(RampsGroundProfile groundProfile, double comHeightForRoughTerrain) throws IOException, AssertionFailedError
   {
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      stepTeleopManager.setEndDoubleSupportDuration(0.05);
      stepTeleopManager.setStanceLength(1.00);
      stepTeleopManager.setStanceWidth(0.30);
      stepTeleopManager.setStepDuration(0.35);
      stepTeleopManager.setStepGroundClearance(0.1);
      stepTeleopManager.setDesiredBodyHeight(comHeightForRoughTerrain);

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

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingDownSlope() throws IOException
   {
      walkSlope(0.2, getWalkingDownSlopePosition());
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 980000)
   public void testWalkingUpSlope() throws IOException
   {
      walkSlope(-0.1, getWalkingUpSlopePosition());
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
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

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

   public abstract double getComHeightForRoughTerrain();

   public abstract QuadrupedInitialPositionParameters getWalkingDownSlopePosition();

   public abstract QuadrupedInitialPositionParameters getWalkingUpSlopePosition();
}
