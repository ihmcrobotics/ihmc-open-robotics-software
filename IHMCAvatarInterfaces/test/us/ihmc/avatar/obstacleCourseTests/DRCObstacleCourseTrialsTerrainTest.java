package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import java.io.InputStream;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.DarpaRoboticsChallengeTrialsWalkingEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseTrialsTerrainTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DarpaRoboticsChallengeTrialsWalkingEnvironment environment;

   protected abstract DRCRobotModel getRobotModelWithAdditionalFootContactPoints();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      environment = new DarpaRoboticsChallengeTrialsWalkingEnvironment();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
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

      if (environment != null)
      {
         environment = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 57.4)
   @Test(timeout = 290000)
   public void testTrialsTerrainSlopeScript() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsSlopeLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING;
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCSlopeTest", selectedLocation, simulationTestingParameters, getRobotModel());
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.01);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      FramePoint pelvisPosition = new FramePoint(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      drcSimulationTestHelper.send(new PelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.05));
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingOntoSlopes(simulationConstructionSet);
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(30.0);
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);

      // Atlas
      Point3D center = new Point3D(3.7337489920899674, 4.102901514571013, 0.7892401231988355);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 61.5)
   @Test(timeout = 310000)
   public void testTrialsTerrainSlopeScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsSlopeLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING;
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCSlopeTest", selectedLocation, simulationTestingParameters, getRobotModel());
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.01);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      FramePoint pelvisPosition = new FramePoint(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      drcSimulationTestHelper.send(new PelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.05));
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingOntoSlopes(simulationConstructionSet);
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(robot, 1201L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.02, 0.03, 0.0}, new double[] {0.025, 0.035, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.02, 0.01, 0.0}, new double[] {0.2, 0.05, 0.01});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 0.5);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.01, 0.03);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);
      robot.setController(slipRandomOnEachStepPerturber, 10);
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.6);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(30.0);
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);

      // Atlas
      Point3D center = new Point3D(3.853111159859177, 4.117657981767957, 0.7897555650626801);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.4)
   @Test(timeout = 200000)
   public void testTrialsTerrainZigzagHurdlesScript() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsZigzagHurdlesLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_ZIGZAG_BLOCKS;
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCZigzagHurdlesTest", selectedLocation, simulationTestingParameters, getRobotModel());
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.01);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingOverHurdles(simulationConstructionSet);
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(13.0);
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3D center = new Point3D(4.437597506324034, 5.699204748831417, 0.8376763465412774);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.4)
   @Test(timeout = 150000)
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsZigzagHurdlesLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_ZIGZAG_BLOCKS;
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCZigzagHurdlesTest", selectedLocation, simulationTestingParameters, getRobotModelWithAdditionalFootContactPoints());
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.01);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingOverHurdles(simulationConstructionSet);
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(robot, 1201L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.03, 0.03, 0.0}, new double[] {0.04, 0.06, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.0, 0.0, 0.0}, new double[] {0.2, 0.05, 0.02});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 0.1);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.01, 0.03);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);
      robot.setController(slipRandomOnEachStepPerturber, 10);
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(1.0);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(12.0);
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3D center = new Point3D(4.433273741150176, 5.75375933959496, 0.8417057558698022);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 49.8)
   @Test(timeout = 250000)
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING;
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCWalkingOntoSlopesTest", selectedLocation, simulationTestingParameters, getRobotModel());
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();
      setupCameraForWalkingOntoSlopes(simulationConstructionSet);
      ThreadTools.sleep(1000);
      FootstepDataListMessage footstepDataList = createFootstepsForWalkingToTheSlopesSideways(scriptedFootstepGenerator);
      footstepDataList.getDataList().addAll(createFootstepsForSteppingOverTheSlopesEdgeSideways(scriptedFootstepGenerator).getDataList());
      drcSimulationTestHelper.send(footstepDataList);
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footstepDataList.size() * stepTime + 2.0);
      assertTrue(success);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3D center = new Point3D(3.3267919256794363, 3.355608873842678, 0.9247970191596047);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingOntoSlopes(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(3.6214, 2.5418, 0.5);
      Point3D cameraPosition = new Point3D(6.6816, -0.5441, 1.5);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverHurdles(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(4.9246, 4.0338, 0.5);
      Point3D cameraPosition = new Point3D(8.1885, 1.1641, 1.5);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverCinderblockField(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(7.8655, 6.8947, 0.5);
      Point3D cameraPosition = new Point3D(10.2989, 18.7661, 3.2746);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverSlantedCinderblockField(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(9.7689, 9.0724, 0.5);
      Point3D cameraPosition = new Point3D(8.0254, 16.6036, 2.5378);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverFlatCinderblockField(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(7.447, 7.0966, 0.5);
      Point3D cameraPosition = new Point3D(6.3809, 14.6839, 2.7821);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   /*
    * // Is now a scripted test private FootstepDataList
    * createFootstepsForWalkingToTheSlopesNormally(ScriptedFootstepGenerator
    * scriptedFootstepGenerator) { double[][][]
    * footstepLocationsAndOrientations = new double[][][] { {
    * {2.1265936534673218, 2.3428927767215417, 0.08591277243384399},
    * {0.020704851858501554, -0.008524153124099326, 0.3678797475257915,
    * 0.9296037539099088} }, { {2.6166814909099094, 2.3996906331962657,
    * 0.08226766576256324}, {2.4508705508237736E-4, 7.204771499494282E-4,
    * 0.35318402355492856, 0.9355535614546948} }, { {2.739624824329704,
    * 2.8475642035481017, 0.09205292111010316}, {0.03188071876413438,
    * -0.038174623620904354, 0.3675020633742088, 0.928691849484092} }, {
    * {3.193896638477976, 2.9213552160981826, 0.17477524869701494},
    * {0.04675359881912196, -0.116811005963886, 0.34844159401254376,
    * 0.9288475361678918} }, { {3.017845920846812, 3.1060500699939313,
    * 0.17155859233956158}, {0.03567318375247706, -0.13777549770429312,
    * 0.36045274233562125, 0.9218563644820306} }, };
    *
    * RobotSide[] robotSides =
    * drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT,
    * footstepLocationsAndOrientations.length);
    *
    * return
    * scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations
    * (robotSides, footstepLocationsAndOrientations); }
    *
    * private FootstepDataList
    * createFootstepsForSteppingOverTheSlopeEdge(ScriptedFootstepGenerator
    * scriptedFootstepGenerator) { double[][][]
    * footstepLocationsAndOrientations = new double[][][] { {
    * {3.0196365393148823, 3.157527589669437, 0.18463968610481485},
    * {0.06878939191058485, -0.14859257123868208, 0.36996761202767287,
    * 0.9145010844082091} }, { {3.242221271901224, 2.9525762835818927,
    * 0.19170633363319947}, {0.05074043948683175, -0.1388048392204765,
    * 0.35405122373484255, 0.9234751514694487} }, { {3.2884777064719466,
    * 3.4423998984979947, 0.21600585958795815}, {-0.09042178480364092,
    * 0.12404908940997492, 0.3587959323612516, 0.9207069040528045} }, {
    * {3.504460942426418, 3.2186466197482773, 0.21310781108121274},
    * {-0.10127177343643246, 0.10226561231768722, 0.3474545483212657,
    * 0.9265857268991324} }, };
    *
    * RobotSide[] robotSides =
    * drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT,
    * footstepLocationsAndOrientations.length);
    *
    * return
    * scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations
    * (robotSides, footstepLocationsAndOrientations); }
    */
   private FootstepDataListMessage createFootstepsForWalkingToTheSlopesSideways(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {1.9823966635641284, 2.2652470283072947, 0.0837307038087527}, {0.020640132711869014, -0.008498293160241518, 0.3083105864353867, 0.951023841040206}
         },
         {
            {2.1498164650868086, 1.9068093446433945, 0.08347777099109424},
            {0.0017662883179677942, 0.006293715779592783, 0.10348136532245923, 0.9946099116730455}
         },
         {
            {2.0874687872550273, 2.298441155960018, 0.08427730698998047},
            {0.018633535386299895, -0.007684905338928583, 0.024333507423710935, 0.9995006823436388}
         },
         {
            {2.0314602664501606, 1.9146192377173497, 0.08341613830944179},
            {0.0034096687293227626, 0.011999890346433685, -0.18430794544823617, 0.9827893762325068}
         },
         {
            {2.2034072670888567, 2.281143757931813, 0.08394437770346314}, {0.015080397973109637, -0.006233688252807148, -0.26166319609415833, 0.96502129227159}
         },
         {
            {2.01915833306869, 2.0207468649842344, 0.08359962278744236}, {0.004579288707518639, 0.014818193944121557, -0.36640082496031345, 0.9303278382976451}
         },
         {
            {2.3514127370083746, 2.3597369501954506, 0.0860154559910794},
            {0.013290279881016632, -0.005689963592658761, -0.38133174097309863, 0.9243252112224485}
         },
         {
            {2.25188495618737, 2.254080704383737, 0.08394252424978803}, {-0.007521908318387368, -0.014514380020142312, -0.36682257924428485, 0.9301472727608521}
         },
         {
            {2.5571508061806436, 2.575341468058366, 0.08684300869106024}, {0.01329027988101649, -0.005689963592658824, -0.3813317409730987, 0.9243252112224485}
         },
         {
            {2.457577639970055, 2.4696407711863455, 0.0869664911152282}, {0.004579288707518356, 0.014818193944121443, -0.3664008249603138, 0.930327838297645}
         },
         {
            {2.768315526189699, 2.785782484898131, 0.08433104034516861}, {0.013290279881016347, -0.0056899635926588865, -0.3813317409730988, 0.9243252112224485}
         },
         {
            {2.5635712023164903, 2.5743143400835953, 0.0870408732178602}, {0.0045792887075182115, 0.014818193944121382, -0.36640082496031384, 0.930327838297645}
         },
         {
            {2.885879020571682, 2.856576938293644, 0.10386656274046036}, {0.12558777759117537, -0.06679059297300634, -0.3513269921130159, 0.92538428310775}
         },
         {
            {2.7488021282402655, 2.6966826406653803, 0.08370236609247309},
            {-0.002071973823236298, -0.003865630015040666, -0.34064306248913384, 0.9401824651667817}
         },
         {
            {2.986089110278983, 2.9635615047422394, 0.14058922355834996}, {0.10717475880427008, -0.026653519731933816, -0.35290983266496895, 0.9291166831832962}
         },
         {
            {2.844078254699551, 2.7987722839957434, 0.08895082990782543}, {0.05682759834686232, -0.026260994395297752, -0.34066061044879653, 0.9380998522162506}
         },
         {
            {3.079930510367088, 3.075738188913338, 0.18223629720937254}, {0.09637509142099876, -0.05639328437007134, -0.3547113886462364, 0.9282841536922889}
         },
         {
            {2.9344673839516484, 2.905450415197158, 0.12337655475135587}, {0.11580121216799653, -0.04880027856780968, -0.33742432635314157, 0.9329273476842961}
         },
         {
            {3.128623440850548, 3.133453453117145, 0.20285914961446738}, {0.12598064593469932, -0.06710112170905909, -0.35316275861517443, 0.9246093133008028}
         }
      };
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForSteppingOverTheSlopesEdgeSideways(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {3.0166188930803033, 3.0119111124382747, 0.15483943760187868}, {0.1089192633351566, -0.024058516458074313, -0.36480741929991994, 0.9243772653435911}
         },
         {
            {3.285102645187515, 3.361157755301027, 0.22549038617963604}, {-0.12112164390947337, 0.0472878892769468, -0.34692969244751093, 0.9288343185965263}
         },
         {
            {3.1055260564624887, 3.160607633126951, 0.20546113718754253}, {0.12642092274810612, -0.06414867390038669, -0.3595546575929555, 0.9222923322523894}
         },
         {
            {3.3695983984590763, 3.4737424555716165, 0.18833480541902758}, {-0.12112164390947357, 0.04728788927694677, -0.3469296924475111, 0.9288343185965262}
         },
         {
            {3.2811041904535196, 3.3537632182460775, 0.22458026669614373}, {-0.11259311979747, 0.025105614756717354, -0.36036496334456175, 0.9256509010829258}
         }
      };
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
}
