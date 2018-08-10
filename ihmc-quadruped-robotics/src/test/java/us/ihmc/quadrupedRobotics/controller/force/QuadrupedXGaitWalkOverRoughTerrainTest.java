package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.function.DoubleSupplier;

public abstract class QuadrupedXGaitWalkOverRoughTerrainTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testWalkingOverTiledGround(QuadrupedXGaitSettingsReadOnly xGaitSettings) throws IOException
   {
      VaryingHeightTiledGroundEnvironment environment = new VaryingHeightTiledGroundEnvironment(0.75, 10, 4, -0.1, 0.1);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, xGaitSettings);
   }

   public void testWalkingOverSingleStepUp(QuadrupedXGaitSettingsReadOnly xGaitSettings) throws IOException
   {
      SingleStepEnvironment environment = new SingleStepEnvironment(0.1, 1.0);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, xGaitSettings);
   }

   public void testWalkingOverConsecutiveRamps(QuadrupedXGaitSettingsReadOnly xGaitSettings) throws IOException
   {
      ZigZagSlopeEnvironment environment = new ZigZagSlopeEnvironment(0.15, 0.5, 4, -0.1);
      double walkTime = 15.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 3.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, xGaitSettings);
   }

   public void testWalkingOverCinderBlockField(QuadrupedXGaitSettingsReadOnly xGaitSettings) throws IOException
   {
      CinderBlockFieldPlanarRegionEnvironment environment = new CinderBlockFieldPlanarRegionEnvironment();
      double walkTime = 40.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking = 8.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, xGaitSettings);
   }

   public void testWalkingUpStaircase(QuadrupedXGaitSettingsReadOnly xGaitSettings, double stepHeight, double stepLength) throws IOException
   {
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength);
      double walkTime = 20.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking  = numberOfSteps * stepLength + 0.5;

      testWalkingOverTerrain(staircaseEnvironment, walkTime, walkingSpeed, minimumXPositionAfterWalking, xGaitSettings);
   }

   private void testWalkingOverTerrain(PlanarRegionEnvironmentInterface environment, double walkTime, double walkingSpeed,
                                       double minimumXPositionAfterWalking, QuadrupedXGaitSettingsReadOnly xGaitSettings) throws IOException
   {
      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(environment.getTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);

      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(new DefaultPointFootSnapperParameters());
      snapper.setPlanarRegionsList(environment.getPlanarRegionsList());
      stepTeleopManager.setStepSnapper(snapper);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.getXGaitSettings().set(xGaitSettings);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), minimumXPositionAfterWalking));
      conductor.simulate();

      stepTeleopManager.requestStopWalking();
      conductor.addTimeLimit(variables.getYoTime(), 1.5);
      conductor.simulate();
   }
}