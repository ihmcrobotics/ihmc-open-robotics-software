package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.QuadrupedTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RampsGroundProfile;
import us.ihmc.tools.MemoryTools;

@Tag("quadruped-xgait-3")
public abstract class QuadrupedXGaitWalkingOverRampsTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      quadrupedTestFactory = createQuadrupedTestFactory();
   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testWalkingOverShallowRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.075, 0.75, 1.2);

      walkOverRamps(groundProfile, getComHeightForRoughTerrain());
   }

   @Test
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      RampsGroundProfile groundProfile = new RampsGroundProfile(0.15, 0.75, 1.2);
      walkOverRamps(groundProfile, getComHeightForRoughTerrain());
   }

   private void walkOverRamps(RampsGroundProfile groundProfile, double comHeightForRoughTerrain) throws IOException
   {
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      stepTeleopManager.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());

      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.AMBLE.getEndPhaseShift(), 0.05);
      stepTeleopManager.setStepDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.AMBLE.getEndPhaseShift(), 0.3);
      //      stepTeleopManager.setStanceLength(1.00);
      stepTeleopManager.setStanceWidth(0.30);
      stepTeleopManager.setStepGroundClearance(0.08);
      stepTeleopManager.setDesiredBodyHeight(comHeightForRoughTerrain);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      stepTeleopManager.requestXGait();
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.4, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 15.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 3.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(-0.4, 0.0, 0.0);
      conductor.addTimeLimit(variables.getYoTime(), 15.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), 0.0));
      conductor.simulate();

      conductor.concludeTesting();
   }

   @Test
   public void testWalkingDownSlope() throws IOException
   {
      walkSlope(0.1, getWalkingDownSlopePosition());
   }

   @Test
   public void testWalkingUpSlope() throws IOException
   {
      walkSlope(-0.1, getWalkingUpSlopePosition());
   }

   private void walkSlope(double slope, QuadrupedInitialPositionParameters initialPosition) throws IOException
   {
      InclinedGroundProfile groundProfile = new InclinedGroundProfile(slope);
      quadrupedTestFactory.setInitialPosition(initialPosition);
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setUseStateEstimator(false);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
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
