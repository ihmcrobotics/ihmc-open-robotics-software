package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.commons.PrintTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StaircaseEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class Quadruped2018PIDemoTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;
   private QuadrupedTestFactory quadrupedTestFactory;
   private RemoteQuadrupedTeleopManager stepTeleopManager;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   private void createTest()
   {
      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         quadrupedTestFactory.setUsePushRobotController(true);

         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         pusher = new PushRobotTestConductor(conductor.getScs(), "body");
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }


   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();

      quadrupedTestFactory = null;
      conductor = null;
      variables = null;
      pusher = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testTrottingFast()
   {
      QuadrupedSpeed quadrupedSpeed = QuadrupedSpeed.FAST;
      double endPhaseShift = 180.0;
      double walkingSpeed = 2.0;
      createTest();

      stepTeleopManager.setStanceWidth(0.25);
      stepTeleopManager.setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double initialDoubleSupportDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();
      double singleSupportDuration = 0.3;

      // this was sending a CoMPositionPacket and wasn't being used in the controller
//      stepTeleopManager.setDesiredBodyHeight(0.8);

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.FAST);
      stepTeleopManager.setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setStepDuration(quadrupedSpeed, endPhaseShift, 0.15);
      stepTeleopManager.setStepGroundClearance(0.05);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.5 * walkingSpeed, 0.0, 0.0);

      double initialWalkTime = initialDoubleSupportDuration + singleSupportDuration;
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), initialWalkTime));

      double walkTime = 5.0;
      stepTeleopManager.setEndDoubleSupportDuration(quadrupedSpeed, endPhaseShift, 0.01);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));



      double finalPositionX = walkTime * walkingSpeed * 0.7;
      if(walkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();
   }

   @Test
   public void testTrottingWithPush()
   {
      QuadrupedSpeed speed = QuadrupedSpeed.FAST;
      double endPhaseShift = 180.0;
      double walkingSpeed = 1.0;
      createTest();

      stepTeleopManager.setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setQuadrupedSpeed(speed);
      stepTeleopManager.setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setStepDuration(speed, endPhaseShift, 0.2);
      stepTeleopManager.setEndDoubleSupportDuration(speed, endPhaseShift, 0.001);


      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      stepTeleopManager.requestXGait();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.simulate();

      double duration = 0.2;
      double magnitude = 225;
      pusher.applyForce(new Vector3D(0.0, -1.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.75);
      conductor.simulate();

      magnitude = 200;
      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.75);
      conductor.simulate();

      duration = 0.5;
      magnitude = 500.0;
      pusher.applyForce(new Vector3D(-1.0, 0.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 2.5);
      conductor.simulate();

      duration = 0.4;
      magnitude = 400.0;
      pusher.applyForce(new Vector3D(1.0, 0.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 5.0);
      conductor.simulate();
   }

   @Test
   public void testMultiGait()
   {
      createTest();

      stepTeleopManager.setStanceWidth(0.25);
      stepTeleopManager.setStanceLength(0.7);
      //      stepTeleopManager.setDesiredBodyHeight(0.55);

      stepTeleopManager.setStepDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.AMBLE.getEndPhaseShift(), 0.3);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.AMBLE.getEndPhaseShift(), 0.15);

      stepTeleopManager.setStepDuration(QuadrupedSpeed.FAST, QuadrupedGait.AMBLE.getEndPhaseShift(), 0.25);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.FAST, QuadrupedGait.AMBLE.getEndPhaseShift(), 0.001);

      stepTeleopManager.setStepDuration(QuadrupedSpeed.FAST, QuadrupedGait.PACE.getEndPhaseShift(), 0.25);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.FAST, QuadrupedGait.PACE.getEndPhaseShift(), 0.001);

      stepTeleopManager.setStepDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.TROT.getEndPhaseShift(), 0.25);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.MEDIUM, QuadrupedGait.TROT.getEndPhaseShift(), 0.15);

      stepTeleopManager.setStepDuration(QuadrupedSpeed.FAST, QuadrupedGait.TROT.getEndPhaseShift(), 0.25);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.FAST, QuadrupedGait.TROT.getEndPhaseShift(), 0.001);


      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());
      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);

      double startingStanceDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();

      stepTeleopManager.requestXGait();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.5);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.7, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 3.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.6, 0.2, 0.4);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 3.0);
      conductor.simulate();

      stepTeleopManager.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());

      stepTeleopManager.setDesiredVelocity(0.7 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.FAST);

      stepTeleopManager.setDesiredVelocity(1.0 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 4.0);
      conductor.simulate();

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);

      stepTeleopManager.setDesiredVelocity(0.6 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.6, -0.2, -0.2);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 6.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.8, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();


      stepTeleopManager.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());

      stepTeleopManager.setDesiredVelocity(0.4 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.FAST);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setEndPhaseShift(QuadrupedGait.PACE.getEndPhaseShift());

      stepTeleopManager.setDesiredVelocity(0.5 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 5.0);
      conductor.simulate();

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      stepTeleopManager.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());

      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();

      stepTeleopManager.requestStanding();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();
   }

   @Test
   public void testWalkingUpStaircase() throws IOException
   {
      double stepHeight = 0.13;
      double stepLength = 0.40;
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength, true);
      double walkTime = 20.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking  = numberOfSteps * stepLength + 1.2;

      double bodyHeight = 0.50;
      double stanceLength = 0.7;

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      quadrupedTestFactory = createQuadrupedTestFactory();

      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(staircaseEnvironment.getTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);

      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      stepTeleopManager.setStanceLength(stanceLength);
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);
      stepTeleopManager.submitPlanarRegionsList(staircaseEnvironment.getPlanarRegionsList());

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), walkTime);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), minimumXPositionAfterWalking));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();
   }

   @Test
   public void testWalkingDownStaircase() throws IOException
   {
      double stepHeight = 0.13;
      double stepLength = 0.40;
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength, true);
      double walkTime = 20.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking  = 3.4 + numberOfSteps * stepLength + 1.2;

      double bodyHeight = 0.43;
      double stanceLength = 0.65;
      double stepGroundClearance = 0.06;
      double endDoubleSupportDuration = 0.15;
      double stanceWidth = 0.42;
      double endPhaseShift = 180.0;

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      quadrupedTestFactory = createQuadrupedTestFactory();

      double heightOffset = stepHeight * numberOfSteps + 0.045;
      QuadrupedInitialOffsetAndYaw initialOffsetAndYaw = new QuadrupedInitialOffsetAndYaw(3.4, 0.0, heightOffset, 0.0);

      quadrupedTestFactory.setInitialOffset(initialOffsetAndYaw);
      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(staircaseEnvironment.getTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);

      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      stepTeleopManager.setStanceLength(stanceLength);
      stepTeleopManager.setStepGroundClearance(stepGroundClearance);
      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.FAST);
      stepTeleopManager.setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setStepDuration(QuadrupedSpeed.FAST, endPhaseShift, 0.3);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.FAST, endPhaseShift, endDoubleSupportDuration);
      stepTeleopManager.setStanceWidth(stanceWidth);
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);
      stepTeleopManager.submitPlanarRegionsList(staircaseEnvironment.getPlanarRegionsList());

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), walkTime);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), minimumXPositionAfterWalking));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();
   }

   @Test
   public void testWalkingOverCinderBlocks() throws IOException
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      PlanarRegionsListExamples.generateCinderBlockField(generator, 0.4, 0.1, 9, 4, 0.03, 0.0, 1.2);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("testEnvironment", planarRegionsList, 1e-2, false);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      quadrupedTestFactory = createQuadrupedTestFactory();

      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(environment.getTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);

      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      stepTeleopManager.submitPlanarRegionsList(planarRegionsList);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.3, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 30.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 4.65));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();
   }

   @Test
   public void testTrottingOverAggressiveBumpyTerrain() throws IOException
   {
      double xAmp1 = 0.03, xFreq1 = 0.5, xAmp2 = 0.02, xFreq2 = 0.5;
      double yAmp1 = 0.02, yFreq1 = 0.07, yAmp2 = 0.02, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, 1.2);

      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.5));
      conductor.simulate();

      stepTeleopManager.setQuadrupedSpeed(QuadrupedSpeed.FAST);
      stepTeleopManager.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.FAST, QuadrupedGait.TROT.getEndPhaseShift(), 0.07);
      stepTeleopManager.setStepDuration(QuadrupedSpeed.FAST, QuadrupedGait.TROT.getEndPhaseShift(), 0.25);
      stepTeleopManager.setStanceWidth(0.35);
      stepTeleopManager.setStepGroundClearance(0.08);
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.4, 0.0, 0.0);
      conductor.addSustainGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.0));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 5.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();
   }
}
