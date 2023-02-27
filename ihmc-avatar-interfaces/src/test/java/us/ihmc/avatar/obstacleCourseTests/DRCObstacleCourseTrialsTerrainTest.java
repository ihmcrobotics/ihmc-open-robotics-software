package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.InputStream;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.environments.DRCTrialsWalkingEnvironment;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCObstacleCourseTrialsTerrainTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private DRCTrialsWalkingEnvironment environment;

   protected abstract DRCRobotModel getRobotModelWithAdditionalFootContactPoints();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      environment = new DRCTrialsWalkingEnvironment();
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

      if (environment != null)
      {
         environment = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testTrialsTerrainSlopeScript(double offsetHeight)
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsSlopeLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      simulationTestHelper.simulateNow(0.01);
      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      if (offsetHeight != 0.0)
      {
         FramePoint3D pelvisPosition = new FramePoint3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + offsetHeight));
      }

      setupCameraForWalkingOntoSlopes();
      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(21.0);
      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();
      assertTrue(success);

      // Atlas
      Point3D center = new Point3D(3.7337489920899674, 4.102901514571013, 0.7892401231988355);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testTrialsTerrainSlopeScriptRandomFootSlip()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsSlopeLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      simulationTestHelper.simulateNow(0.5);
      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      setupCameraForWalkingOntoSlopes();
      Robot robot = simulationTestHelper.getRobot();
      SideDependentList<String> footNames = new SideDependentList<>(side -> fullRobotModel.getFoot(side).getName());
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                                                                      robot,
                                                                                                      footNames,
                                                                                                      1201L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.02, 0.03, 0.0}, new double[] {0.025, 0.035, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.02, 0.01, 0.0}, new double[] {0.2, 0.05, 0.01});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 0.5);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.01, 0.03);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);
      robot.addThrottledController(slipRandomOnEachStepPerturber, 10 * simulationTestHelper.getSimulationDT());
      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.6);
      success = success && simulationTestHelper.simulateNow(20.0);
      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();
      assertTrue(success);

      // Atlas
      Point3D center = new Point3D(3.853111159859177, 4.117657981767957, 0.7897555650626801);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testTrialsTerrainZigzagHurdlesScript()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsZigzagHurdlesLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_ZIGZAG_BLOCKS;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      simulationTestHelper.simulateNow(0.01);
      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      setupCameraForWalkingOverHurdles();
      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(8.0);
      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
//      simulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3D center = new Point3D(4.437597506324034, 5.699204748831417, 0.8376763465412774);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String scriptName = "scripts/ExerciseAndJUnitScripts/DRCTrialsZigzagHurdlesLeftFootPose.xml";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_ZIGZAG_BLOCKS;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(), environment, simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      simulationTestHelper.simulateNow(0.01);
      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      setupCameraForWalkingOverHurdles();
      Robot robot = simulationTestHelper.getRobot();
      SideDependentList<String> footNames = new SideDependentList<>(side -> fullRobotModel.getFoot(side).getName());
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(simulationTestHelper.getSimulationConstructionSet().getTime(), robot, footNames, 1201L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[] {0.03, 0.03, 0.0}, new double[] {0.04, 0.06, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[] {0.0, 0.0, 0.0}, new double[] {0.2, 0.05, 0.02});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 0.1);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.01, 0.03);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);
      robot.addThrottledController(slipRandomOnEachStepPerturber, 10 * simulationTestHelper.getSimulationDT());
      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(1.0);
      success = success && simulationTestHelper.simulateNow(6.5);
      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
//      simulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3D center = new Point3D(4.433273741150176, 5.75375933959496, 0.8417057558698022);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingOntoAndOverSlopesSideways()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(), environment, simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      setupCameraForWalkingOntoSlopes();
      ThreadTools.sleep(1000);
      Assert.assertTrue(simulationTestHelper.simulateNow(0.5));
      FootstepDataListMessage footstepDataList = createFootstepsForWalkingToTheSlopesSideways();
      List<FootstepDataMessage> dataList = createFootstepsForSteppingOverTheSlopesEdgeSideways().getFootstepDataList();
      for (int i = 0; i < dataList.size(); i++)
      {
         FootstepDataMessage step = dataList.get(i);
         footstepDataList.getFootstepDataList().add().set(step);
      }
      simulationTestHelper.publishToController(footstepDataList);
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      boolean success = simulationTestHelper.simulateNow(footstepDataList.getFootstepDataList().size() * stepTime + 2.0);
      assertTrue(success);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
//      simulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3D center = new Point3D(3.3267919256794363, 3.355608873842678, 0.9247970191596047);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingOntoSlopes()
   {
      Point3D cameraFix = new Point3D(3.6214, 2.5418, 0.5);
      Point3D cameraPosition = new Point3D(6.6816, -0.5441, 1.5);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private void setupCameraForWalkingOverHurdles()
   {
      Point3D cameraFix = new Point3D(4.9246, 4.0338, 0.5);
      Point3D cameraPosition = new Point3D(8.1885, 1.1641, 1.5);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingToTheSlopesSideways()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(1.9823966635641284, 2.2652470283072947, 0.0837307038087527),
                       new Quaternion(0.020640132711869014, -0.008498293160241518, 0.3083105864353867, 0.951023841040206)),
            new Pose3D(new Point3D(2.1498164650868086, 1.9068093446433945, 0.08347777099109424),
                       new Quaternion(0.0017662883179677942, 0.006293715779592783, 0.10348136532245923, 0.9946099116730455)),
            new Pose3D(new Point3D(2.0874687872550273, 2.298441155960018, 0.08427730698998047),
                       new Quaternion(0.018633535386299895, -0.007684905338928583, 0.024333507423710935, 0.9995006823436388)),
            new Pose3D(new Point3D(2.0314602664501606, 1.9146192377173497, 0.08341613830944179),
                       new Quaternion(0.0034096687293227626, 0.011999890346433685, -0.18430794544823617, 0.9827893762325068)),
            new Pose3D(new Point3D(2.2034072670888567, 2.281143757931813, 0.08394437770346314),
                       new Quaternion(0.015080397973109637, -0.006233688252807148, -0.26166319609415833, 0.96502129227159)),
            new Pose3D(new Point3D(2.01915833306869, 2.0207468649842344, 0.08359962278744236),
                       new Quaternion(0.004579288707518639, 0.014818193944121557, -0.36640082496031345, 0.9303278382976451)),
            new Pose3D(new Point3D(2.3514127370083746, 2.3597369501954506, 0.0860154559910794),
                       new Quaternion(0.013290279881016632, -0.005689963592658761, -0.38133174097309863, 0.9243252112224485)),
            new Pose3D(new Point3D(2.25188495618737, 2.254080704383737, 0.08394252424978803),
                       new Quaternion(-0.007521908318387368, -0.014514380020142312, -0.36682257924428485, 0.9301472727608521)),
            new Pose3D(new Point3D(2.5571508061806436, 2.575341468058366, 0.08684300869106024),
                       new Quaternion(0.01329027988101649, -0.005689963592658824, -0.3813317409730987, 0.9243252112224485)),
            new Pose3D(new Point3D(2.457577639970055, 2.4696407711863455, 0.0869664911152282),
                       new Quaternion(0.004579288707518356, 0.014818193944121443, -0.3664008249603138, 0.930327838297645)),
            new Pose3D(new Point3D(2.768315526189699, 2.785782484898131, 0.08433104034516861),
                       new Quaternion(0.013290279881016347, -0.0056899635926588865, -0.3813317409730988, 0.9243252112224485)),
            new Pose3D(new Point3D(2.5635712023164903, 2.5743143400835953, 0.0870408732178602),
                       new Quaternion(0.0045792887075182115, 0.014818193944121382, -0.36640082496031384, 0.930327838297645)),
            new Pose3D(new Point3D(2.885879020571682, 2.856576938293644, 0.10386656274046036),
                       new Quaternion(0.12558777759117537, -0.06679059297300634, -0.3513269921130159, 0.92538428310775)),
            new Pose3D(new Point3D(2.7488021282402655, 2.6966826406653803, 0.08370236609247309),
                       new Quaternion(-0.002071973823236298, -0.003865630015040666, -0.34064306248913384, 0.9401824651667817)),
            new Pose3D(new Point3D(2.986089110278983, 2.9635615047422394, 0.14058922355834996),
                       new Quaternion(0.10717475880427008, -0.026653519731933816, -0.35290983266496895, 0.9291166831832962)),
            new Pose3D(new Point3D(2.844078254699551, 2.7987722839957434, 0.08895082990782543),
                       new Quaternion(0.05682759834686232, -0.026260994395297752, -0.34066061044879653, 0.9380998522162506)),
            new Pose3D(new Point3D(3.079930510367088, 3.075738188913338, 0.18223629720937254),
                       new Quaternion(0.09637509142099876, -0.05639328437007134, -0.3547113886462364, 0.9282841536922889)),
            new Pose3D(new Point3D(2.9344673839516484, 2.905450415197158, 0.12337655475135587),
                       new Quaternion(0.11580121216799653, -0.04880027856780968, -0.33742432635314157, 0.9329273476842961)),
            new Pose3D(new Point3D(3.128623440850548, 3.133453453117145, 0.20285914961446738),
                       new Quaternion(0.12598064593469932, -0.06710112170905909, -0.35316275861517443, 0.9246093133008028))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSteppingOverTheSlopesEdgeSideways()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(3.0166188930803033, 3.0119111124382747, 0.15483943760187868),
                       new Quaternion(0.1089192633351566, -0.024058516458074313, -0.36480741929991994, 0.9243772653435911)),
            new Pose3D(new Point3D(3.285102645187515, 3.361157755301027, 0.22549038617963604),
                       new Quaternion(-0.12112164390947337, 0.0472878892769468, -0.34692969244751093, 0.9288343185965263)),
            new Pose3D(new Point3D(3.1055260564624887, 3.160607633126951, 0.20546113718754253),
                       new Quaternion(0.12642092274810612, -0.06414867390038669, -0.3595546575929555, 0.9222923322523894)),
            new Pose3D(new Point3D(3.3695983984590763, 3.4737424555716165, 0.18833480541902758),
                       new Quaternion(-0.12112164390947357, 0.04728788927694677, -0.3469296924475111, 0.9288343185965262)),
            new Pose3D(new Point3D(3.2811041904535196, 3.3537632182460775, 0.22458026669614373),
                       new Quaternion(-0.11259311979747, 0.025105614756717354, -0.36036496334456175, 0.9256509010829258))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }
}
