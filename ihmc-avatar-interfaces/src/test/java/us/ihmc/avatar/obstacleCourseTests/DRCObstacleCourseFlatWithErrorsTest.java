package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.InputStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.OscillateFeetPerturber;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.PelvisKinematicsBasedLinearStateCalculator;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DRCObstacleCourseFlatWithErrorsTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public SideDependentList<String> getFootNames()
   {
      return new SideDependentList<>(side -> getRobotModel().getJointMap().getFootName(side));
   }

   @Test
   public void testSimpleFlatGroundScriptWithRandomFootSlip()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getQueuedControllerCommands();
      setupCameraForWalkingUpToRamp();
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                                                                      simulationTestHelper.getRobot(),
                                                                                                      getFootNames(),
                                                                                                      1002L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.01, 0.01, 0.0}, new double[] {0.06, 0.06, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.03, 0.0, 0.0}, new double[] {0.3, 0.0, 0.0});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.005, 0.25);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.005, 0.5);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);

      simulationTestHelper.getRobot().addThrottledController(slipRandomOnEachStepPerturber,
                                                             10.0 * simulationTestHelper.getSimulationConstructionSet().getDT());

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      loadScriptFileInLeftSoleFrame(scriptName);
      success = success && simulationTestHelper.simulateNow(16.0);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.2315617729419353, 0.14530717103231391, 0.8358344340816537);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSimpleFlatGroundScriptWithOscillatingFeet()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);
      //      simulationTestHelperFactory.getSCSInitialSetup().enableGroundSlipping(0.7, 0.7); FIXME
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      simulationTestHelper.getQueuedControllerCommands();
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(simulationTestHelper.getRobot(),
                                                                                 getFootNames(),
                                                                                 simulationTestHelper.getSimulationDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[] {0.008, 0.011, 0.004});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] {0.012, 0.047, 0.009});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] {1.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] {2.0, 0.5, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] {5.0, 0.5, 0.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] {0.2, 3.4, 1.11});

      simulationTestHelper.getRobot().addThrottledController(oscillateFeetPerturber, ticksPerPerturbation * simulationTestHelper.getSimulationDT());

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);

      // Load script file:
      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      success = success && simulationTestHelper.simulateNow(16.0);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.199355605426889, 0.15130115291430654, 0.8414863015120644);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.3, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void loadScriptFileInLeftSoleFrame(String scriptName)
   {
      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      ReferenceFrame leftSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.LEFT);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      simulationTestHelper.loadScriptFile(scriptInputStream, leftSoleFrame);
   }

   @Test
   public void testStandingWithOscillatingFeet()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(simulationTestHelper.getRobot(),
                                                                                 getFootNames(),
                                                                                 simulationTestHelper.getSimulationDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[] {0.01, 0.015, 0.005});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] {0.017, 0.012, 0.011});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] {5.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] {2.0, 6.5, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] {5.0, 0.5, 7.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] {0.2, 3.4, 1.11});

      simulationTestHelper.getRobot().addThrottledController(oscillateFeetPerturber, ticksPerPerturbation * simulationTestHelper.getSimulationDT());

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(6.0);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.011508654344298094, -0.005208268357032689, 0.780662368979778);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testStandingWithStateEstimatorDrift()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getQueuedControllerCommands();
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      setupCameraForWalkingUpToRamp();
      Vector3DReadOnly slidingVelocity = new Vector3D(0.10, 0.0, 0.0);
      double simDT = simulationTestHelper.getSimulationDT();

      TimeConsumer stateEstimatorDriftator = createStateEstimatorDriftator(simulationTestHelper, fullRobotModel, slidingVelocity, simDT);
      simulationTestHelper.getSimulationConstructionSet().addAfterPhysicsCallback(stateEstimatorDriftator);

      boolean success = simulationTestHelper.simulateNow(10.0);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.0, 0.0, 0.86);
      Vector3D plusMinusVector = new Vector3D(0.06, 0.06, 0.05);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private TimeConsumer createStateEstimatorDriftator(SCS2AvatarTestingSimulation simulationTestHelper,
                                                      FullHumanoidRobotModel fullRobotModel,
                                                      Vector3DReadOnly slidingVelocity,
                                                      double simDT)
   {
      SideDependentList<YoFramePoint3D> stateEstimatorFootPosition = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         String namespace = PelvisKinematicsBasedLinearStateCalculator.class.getSimpleName();
         String namePrefix = fullRobotModel.getFoot(robotSide).getName() + "FootPositionInWorld";
         YoDouble x = (YoDouble) simulationTestHelper.findVariable(namespace, namePrefix + "X");
         YoDouble y = (YoDouble) simulationTestHelper.findVariable(namespace, namePrefix + "Y");
         YoDouble z = (YoDouble) simulationTestHelper.findVariable(namespace, namePrefix + "Z");
         stateEstimatorFootPosition.put(robotSide, new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame()));
      }

      TimeConsumer stateEstimatorDriftator = new TimeConsumer()
      {
         @Override
         public void accept(double time)
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
   public void testSideStepsWithSlipping()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getQueuedControllerCommands();

      Robot robot = simulationTestHelper.getRobot();

      SlipOnNextStepPerturber slipOnEachStepPerturber = new SlipOnNextStepPerturber(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                                                    robot,
                                                                                    RobotSide.LEFT,
                                                                                    getFootNames().get(RobotSide.LEFT));
      slipOnEachStepPerturber.setAmountToSlipNextStep(getFootSlipVector());
      slipOnEachStepPerturber.setRotationToSlipNextStep(-0.15, 0.0, 0.0);
      slipOnEachStepPerturber.setSlipAfterStepTimeDelta(getFootSlipTimeDeltaAfterTouchdown());
      slipOnEachStepPerturber.setPercentToSlipPerTick(0.1);
      robot.addThrottledController(slipOnEachStepPerturber, 10 * simulationTestHelper.getSimulationDT());

      setupCameraForSideStepSlipping();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongSideStepsLeft.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      slipOnEachStepPerturber.setSlipNextStep(true);
      success = success && simulationTestHelper.simulateNow(14.0);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.09590605437816137, 1.0379918543616593, 0.8383906558584916);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSideStepsWithRandomSlipping()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getQueuedControllerCommands();

      Robot robot = simulationTestHelper.getRobot();

      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                                                                      robot,
                                                                                                      getFootNames(),
                                                                                                      1000L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.0, 0.0, 0.0}, new double[] {0.04, 0.04, 0.01});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.0, 0.0, 0.0}, new double[] {0.2, 0.05, 0.02});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 1.0);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.02, 1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.70);

      robot.addThrottledController(slipRandomOnEachStepPerturber, 10 * simulationTestHelper.getSimulationDT());

      setupCameraForSideStepSlipping();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongSideStepsLeft.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.5);
      success = success && simulationTestHelper.simulateNow(9.0);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.022704922237925088, 1.0831838988457891, 0.8389256934215261);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private void setupCameraForSideStepSlipping()
   {
      Point3D cameraFix = new Point3D(2.0, 0.4, 0.75);
      Point3D cameraPosition = new Point3D(6.5, 0.4, 0.75);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   protected abstract Vector3D getFootSlipVector();

   protected abstract double getFootSlipTimeDeltaAfterTouchdown();

}
