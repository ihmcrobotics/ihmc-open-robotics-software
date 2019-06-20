package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.assertEquals;

@Tag("quadruped-xgait")
public abstract class QuadrupedXGaitWalkOverRoughTerrainTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract QuadrupedXGaitSettingsReadOnly getXGaitSettings();

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testWalkingOverTiledGround() throws IOException
   {
      VaryingHeightTiledGroundEnvironment environment = new VaryingHeightTiledGroundEnvironment(0.75, 10, 4, -0.1, 0.1);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;
      // Lower a bit the robot so it doesn't fall off by too much, so the state estimator is not thrown off right from the beginning.
      QuadrupedInitialOffsetAndYaw offsetAndYaw = new QuadrupedInitialOffsetAndYaw(0.0, 0.0, -0.04);

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings(), offsetAndYaw, Double.NaN);
   }

   public void testWalkingOverSingleStepUp(double desiredBodyHeight) throws IOException
   {
      SingleStepEnvironment environment = new SingleStepEnvironment(0.1, 1.0);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings(), null, desiredBodyHeight);
   }

   public void testWalkingOverConsecutiveRamps() throws IOException
   {
      ZigZagSlopeEnvironment environment = new ZigZagSlopeEnvironment(0.15, 0.5, 20, -0.1);
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 3.0;
      double walkTime = 3.0 * minimumXPositionAfterWalking / walkingSpeed;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings(), null, Double.NaN);
   }

   public void testWalkingOverCinderBlockField() throws IOException
   {
      CinderBlockFieldPlanarRegionEnvironment environment = new CinderBlockFieldPlanarRegionEnvironment();
      double walkTime = 40.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking = 8.0;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings(), null, Double.NaN);
   }

   public void testWalkingUpStaircase() throws IOException
   {
      double stepHeight = 0.13;
      double stepLength = 0.8;
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength);
      double walkTime = 20.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking = numberOfSteps * stepLength + 0.5;

      runWalkingOverTerrain(staircaseEnvironment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings(), null, Double.NaN);
   }

   protected void runWalkingOverTerrain(PlanarRegionEnvironmentInterface environment, double walkTime, double walkingSpeed, double minimumXPositionAfterWalking,
                                      QuadrupedXGaitSettingsReadOnly xGaitSettings, QuadrupedInitialOffsetAndYaw offsetAndYaw, double desiredBodyHeight) throws IOException
   {
      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      quadrupedTestFactory = createQuadrupedTestFactory();
      if (offsetAndYaw != null)
         quadrupedTestFactory.setInitialOffset(offsetAndYaw);
      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(environment.getTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);


      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      stepTeleopManager.submitPlanarRegionsList(environment.getPlanarRegionsList());
      if(!Double.isNaN(desiredBodyHeight))
         stepTeleopManager.setDesiredBodyHeight(desiredBodyHeight);

      YoEnum<QuadrupedSteppingStateEnum> steppingCurrentState = (YoEnum<QuadrupedSteppingStateEnum>) conductor.getScs().getVariable("steppingCurrentState");


      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.setXGaitSettings(xGaitSettings);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), minimumXPositionAfterWalking));
      conductor.addTimeLimit(variables.getYoTime(), variables.getYoTime().getDoubleValue() + walkTime);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      conductor.simulate();

      assertEquals(QuadrupedSteppingStateEnum.STAND, steppingCurrentState.getEnumValue());
   }
}