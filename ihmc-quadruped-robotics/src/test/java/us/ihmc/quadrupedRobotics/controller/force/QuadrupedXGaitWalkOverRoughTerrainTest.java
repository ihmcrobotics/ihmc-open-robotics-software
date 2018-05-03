package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.function.DoubleSupplier;

public abstract class QuadrupedXGaitWalkOverRoughTerrainTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testWalkingOverTiledGround() throws IOException, AssertionFailedError
   {
      VaryingHeightTiledGroundEnvironment environment = new VaryingHeightTiledGroundEnvironment(0.75, 10, 4, -0.1, 0.1);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking);
   }

   public void testWalkingOverSingleStepUp() throws IOException, AssertionFailedError
   {
      SingleStepEnvironment environment = new SingleStepEnvironment(0.1, 1.0);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking);
   }

   public void testWalkingOverConsecutiveRamps() throws IOException, AssertionFailedError
   {
      ZigZagSlopeEnvironment environment = new ZigZagSlopeEnvironment(0.15, 0.5, 4, -0.1);
      double walkTime = 15.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 3.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking);
   }

   public void testWalkingOverCinderBlockField() throws IOException, AssertionFailedError
   {
      CinderBlockFieldPlanarRegionEnvironment environment = new CinderBlockFieldPlanarRegionEnvironment();
      double walkTime = 15.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 3.0;

      testWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking);
   }

   private void testWalkingOverTerrain(PlanarRegionEnvironmentInterface environment, double walkTime, double walkingSpeed,
                                       double minimumXPositionAfterWalking) throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setTerrainObject3D(environment.getCombinedTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);

      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      stepTeleopManager.setStepSnapper((x, y) -> environment.getPlanarRegionsList()
                                                            .findPlanarRegionsContainingPointByProjectionOntoXYPlane(x, y)
                                                            .stream()
                                                            .mapToDouble(p -> p.getPlaneZGivenXY(x, y))
                                                            .max()
                                                            .orElse(0.0));

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), minimumXPositionAfterWalking));
      conductor.simulate();
   }
}