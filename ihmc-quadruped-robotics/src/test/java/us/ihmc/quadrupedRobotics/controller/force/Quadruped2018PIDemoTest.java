package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.HeightMapFootSnapper;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StaircaseEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class Quadruped2018PIDemoTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;
   private QuadrupedTestFactory quadrupedTestFactory;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
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
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   
   @After
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
   
   public void testFlatGroundWalking(double endPhaseShift, double walkingSpeed)
   {
      createTest();

      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double initialDoubleSupportDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();
      double singleSupportDuration = 0.3;

      // this was sending a CoMPositionPacket and wasn't being used in the controller
//      stepTeleopManager.setDesiredBodyHeight(0.8);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.15);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.05);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.5 * walkingSpeed, 0.0, 0.0);

      double initialWalkTime = initialDoubleSupportDuration + singleSupportDuration;
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), initialWalkTime));

      double walkTime = 5.0;
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.01);
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

   public void testWalkingWithPush(double endPhaseShift, double walkingSpeed)
   {
      createTest();

      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.2);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.001);


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

   public void testMultiGait()
   {
      createTest();

      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);
      stepTeleopManager.setDesiredBodyHeight(0.55);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(90);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.3);

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

      stepTeleopManager.setDesiredVelocity(0.7 , 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.25);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.001);

      stepTeleopManager.setDesiredVelocity(1.0 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 4.0);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(startingStanceDuration);
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

      stepTeleopManager.getXGaitSettings().setStepDuration(0.3);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(startingStanceDuration);


      stepTeleopManager.setDesiredVelocity(0.4 , 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(90);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.25);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.001);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.5 , 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 5.0);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.3);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(startingStanceDuration);

      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(90);
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

   public void testWalkingUpStaircase() throws IOException
   {
      double stepHeight = 0.13;
      double stepLength = 0.40;
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength, true);
      double walkTime = 20.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking  = numberOfSteps * stepLength + 1.2;

      double bodyHeight = 0.58;
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
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      YoQuadrupedXGaitSettings xGaitSettings = stepTeleopManager.getXGaitSettings();
      xGaitSettings.setStanceLength(stanceLength);
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);

      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(new DefaultPointFootSnapperParameters());
      snapper.setPlanarRegionsList(staircaseEnvironment.getPlanarRegionsList());
      stepTeleopManager.setStepSnapper(snapper);

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

   public void testWalkingDownStaircase() throws IOException
   {
      double stepHeight = 0.13;
      double stepLength = 0.40;
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength, true);
      double walkTime = 20.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking  = 3.4 + numberOfSteps * stepLength + 1.2;

      double bodyHeight = 0.54;
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
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();

      YoQuadrupedXGaitSettings xGaitSettings = stepTeleopManager.getXGaitSettings();
      xGaitSettings.setStanceLength(stanceLength);
      xGaitSettings.setStepGroundClearance(stepGroundClearance);
      xGaitSettings.setEndDoubleSupportDuration(endDoubleSupportDuration);
      xGaitSettings.setStanceWidth(stanceWidth);
      xGaitSettings.setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);

      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(new DefaultPointFootSnapperParameters());
      snapper.setPlanarRegionsList(staircaseEnvironment.getPlanarRegionsList());
      stepTeleopManager.setStepSnapper(snapper);

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
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(new DefaultPointFootSnapperParameters());
      snapper.setPlanarRegionsList(planarRegionsList);
      stepTeleopManager.setStepSnapper(snapper);

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

   public void testTrottingOverAggressiveBumpyTerrain() throws IOException
   {
      double xAmp1 = 0.03, xFreq1 = 0.5, xAmp2 = 0.02, xFreq2 = 0.5;
      double yAmp1 = 0.02, yFreq1 = 0.07, yAmp2 = 0.02, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, 1.2);

      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundProfile3D(groundProfile);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      stepTeleopManager.setStepSnapper(new PlanarGroundPointFootSnapper(quadrupedTestFactory.getRobotName(), stepTeleopManager.getReferenceFrames(), stepTeleopManager.getRos2Node()));

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.5));
      conductor.simulate();
      stepTeleopManager.setDesiredBodyHeight(0.52);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.07);
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.35);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.25);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.08);
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
