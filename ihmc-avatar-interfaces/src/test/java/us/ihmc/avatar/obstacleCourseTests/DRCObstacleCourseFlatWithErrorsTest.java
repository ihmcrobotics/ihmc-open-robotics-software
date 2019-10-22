package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.InputStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.OscillateFeetPerturber;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisKinematicsBasedLinearStateCalculator;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public abstract class DRCObstacleCourseFlatWithErrorsTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String name = "DRCSimpleFlatGroundScriptTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(name);
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      setupCameraForWalkingUpToRamp();
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(robot, 1002L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.01, 0.01, 0.0}, new double[] {0.06, 0.06, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.03, 0.0, 0.0}, new double[] {0.3, 0.0, 0.0});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.005, 0.25);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.005, 0.5);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);

      robot.setController(slipRandomOnEachStepPerturber, 10);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      loadScriptFileInLeftSoleFrame(scriptName);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(16.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.2315617729419353, 0.14530717103231391, 0.8358344340816537);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), flatGround);
      drcSimulationTestHelper.getSCSInitialSetup().enableGroundSlipping(0.7, 0.7);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[] {0.008, 0.011, 0.004});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] {0.012, 0.047, 0.009});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] {1.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] {2.0, 0.5, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] {5.0, 0.5, 0.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] {0.2, 3.4, 1.11});

      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // Load script file:
      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(16.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.199355605426889, 0.15130115291430654, 0.8414863015120644);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.3, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void loadScriptFileInLeftSoleFrame(String scriptName)
   {
      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      ReferenceFrame leftSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.LEFT);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, leftSoleFrame);
   }

   @Test
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[] {0.01, 0.015, 0.005});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] {0.017, 0.012, 0.011});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] {5.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] {2.0, 6.5, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] {5.0, 0.5, 7.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] {0.2, 3.4, 1.11});

      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.011508654344298094, -0.005208268357032689, 0.780662368979778);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testStandingWithStateEstimatorDrift() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      setupCameraForWalkingUpToRamp();
      Vector3DReadOnly slidingVelocity = new Vector3D(0.10, 0.0, 0.0);
      double simDT = simulationConstructionSet.getDT();

      Script stateEstimatorDriftator = createStateEstimatorDriftator(simulationConstructionSet, fullRobotModel, slidingVelocity, simDT);
      simulationConstructionSet.addScript(stateEstimatorDriftator);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.0, 0.0, 0.86);
      Vector3D plusMinusVector = new Vector3D(0.06, 0.06, 0.05);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private Script createStateEstimatorDriftator(SimulationConstructionSet simulationConstructionSet, FullHumanoidRobotModel fullRobotModel,
                                                Vector3DReadOnly slidingVelocity, double simDT)
   {
      SideDependentList<YoFramePoint3D> stateEstimatorFootPosition = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         String nameSpace = PelvisKinematicsBasedLinearStateCalculator.class.getSimpleName();
         String namePrefix = fullRobotModel.getFoot(robotSide).getName() + "FootPositionInWorld";
         YoDouble x = (YoDouble) simulationConstructionSet.getVariable(nameSpace, namePrefix + "X");
         YoDouble y = (YoDouble) simulationConstructionSet.getVariable(nameSpace, namePrefix + "Y");
         YoDouble z = (YoDouble) simulationConstructionSet.getVariable(nameSpace, namePrefix + "Z");
         stateEstimatorFootPosition.put(robotSide, new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame()));
      }

      Script stateEstimatorDriftator = new Script()
      {
         @Override
         public void doScript(double t)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               YoFramePoint3D position = stateEstimatorFootPosition.get(robotSide);
               position.scaleAdd(simDT, slidingVelocity, position);
            }
         }
      };
      return stateEstimatorDriftator;
   }

   @Test
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCLongStepsMaxHeightPauseAndRestartTest");

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

      SlipOnNextStepPerturber slipOnEachStepPerturber = new SlipOnNextStepPerturber(robot, RobotSide.LEFT);
      slipOnEachStepPerturber.setAmountToSlipNextStep(getFootSlipVector());
      slipOnEachStepPerturber.setRotationToSlipNextStep(-0.15, 0.0, 0.0);
      slipOnEachStepPerturber.setSlipAfterStepTimeDelta(getFootSlipTimeDeltaAfterTouchdown());
      slipOnEachStepPerturber.setPercentToSlipPerTick(0.1);
      robot.setController(slipOnEachStepPerturber, 10);

      setupCameraForSideStepSlipping();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongSideStepsLeft.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      slipOnEachStepPerturber.setSlipNextStep(true);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.09590605437816137, 1.0379918543616593, 0.8383906558584916);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSideStepsWithSlippingTest");

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(robot, 1000L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.0, 0.0, 0.0}, new double[] {0.04, 0.04, 0.01});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.0, 0.0, 0.0}, new double[] {0.2, 0.05, 0.02});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 1.0);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.02, 1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.70);

      robot.setController(slipRandomOnEachStepPerturber, 10);

      setupCameraForSideStepSlipping();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongSideStepsLeft.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.5);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.022704922237925088, 1.0831838988457891, 0.8389256934215261);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForSideStepSlipping()
   {
      Point3D cameraFix = new Point3D(2.0, 0.4, 0.75);
      Point3D cameraPosition = new Point3D(6.5, 0.4, 0.75);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   protected abstract Vector3D getFootSlipVector();

   protected abstract double getFootSlipTimeDeltaAfterTouchdown();

}
