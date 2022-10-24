package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.RequestedControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StandPrepControllerStateFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListCorruptor;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class DRCObstacleCourseRampsTest implements MultiRobotTestInterface
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

   // The default height seems to be a bit too low for the ramp
   //   private final ComHeightPacket comHeightPacket = new ComHeightPacket(0.05, 1.0);

   @Test
   public void testWalkingUpRampWithShortSteps()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      doUpRampTest(null, StepLength.SHORT);

      Point3D center = new Point3D(6.135997212353164, 0.008329425009630976, 1.3724038542384285);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingDownRampWithMediumSteps()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_TOP;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverRamp();
      ThreadTools.sleep(1000);

      boolean success = simulationTestHelper.simulateNow(1.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingDownRampMediumSteps();
      simulationTestHelper.publishToController(footstepDataList);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepDuration = walkingControllerParameters.getDefaultTransferTime() + walkingControllerParameters.getDefaultSwingTime();
      double totalDuration = footstepDataList.getFootstepDataList().size() * stepDuration;
      totalDuration += walkingControllerParameters.getDefaultFinalTransferTime() - walkingControllerParameters.getDefaultTransferTime()
            + walkingControllerParameters.getDefaultInitialTransferTime();
      totalDuration += 3.0;

      success = success && simulationTestHelper.simulateNow(totalDuration);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.36, 0.0212, 0.993);
      Vector3D plusMinusVector = new Vector3D(0.3, 0.3, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingUpRampWithMediumSteps()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      doUpRampTest(null, StepLength.MEDIUM);

      Point3D center = new Point3D(7.579638943201888, 0.020725665285290903, 1.46537366331119);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingUpRampWithShortStepsALittleTooHigh()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Vector3D minLocationCorruption = new Vector3D(0.0, 0.0, 0.0);
      Vector3D maxLocationCorruption = new Vector3D(0.0, 0.0, 0.04);
      double maxRotationCorruption = getMaxRotationCorruption();
      FootstepDataListCorruptor footstepDataListCorruptor = new FootstepDataListCorruptor(minLocationCorruption, maxLocationCorruption, maxRotationCorruption);

      doUpRampTest(footstepDataListCorruptor, StepLength.SHORT);

      Point3D center = new Point3D(6.135997212353164, 0.008329425009630976, 1.3724038542384285);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingUpRampWithShortStepsALittleTooLow()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Vector3D minLocationCorruption = new Vector3D(0.0, 0.0, -0.06);
      Vector3D maxLocationCorruption = new Vector3D(0.0, 0.0, 0.0);
      double maxRotationCorruption = getMaxRotationCorruption();
      FootstepDataListCorruptor footstepDataListCorruptor = new FootstepDataListCorruptor(minLocationCorruption, maxLocationCorruption, maxRotationCorruption);

      doUpRampTest(footstepDataListCorruptor, StepLength.SHORT);

      Point3D center = new Point3D(6.135997212353164, 0.008329425009630976, 1.3724038542384285);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private enum StepLength
   {
      SHORT, MEDIUM, LONG;
   }

   public void testHeightReinitialization()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setRunMultiThreaded(true);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;

      YoEnum<HighLevelControllerName> localRequestedControllerState = new YoEnum<>("test", new YoRegistry("Dummy"), HighLevelControllerName.class, true);
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      HighLevelHumanoidControllerFactory controllerFactory = simulationTestHelperFactory.getHighLevelHumanoidControllerFactory();
      controllerFactory.addCustomControlState(new StandPrepControllerStateFactory());
      controllerFactory.addCustomStateTransition(new RequestedControllerStateTransitionFactory<>(localRequestedControllerState,
                                                                                                 HighLevelControllerName.STAND_PREP_STATE,
                                                                                                 HighLevelControllerName.WALKING));
      controllerFactory.addCustomStateTransition(new RequestedControllerStateTransitionFactory<>(localRequestedControllerState,
                                                                                                 HighLevelControllerName.WALKING,
                                                                                                 HighLevelControllerName.STAND_PREP_STATE));
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverRamp();

      FloatingJointBasics controllerRootJoint = simulationTestHelper.getControllerFullRobotModel().getRootJoint();

      assertTrue(simulationTestHelper.simulateNow(1.0));
      Pose3DReadOnly initialPose = new Pose3D(controllerRootJoint.getJointPose());
      Pose3D aboveInitialPose = new Pose3D(initialPose);
      aboveInitialPose.prependTranslation(0.0, 0.0, 0.1);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingUpRamp(StepLength.MEDIUM);
      simulationTestHelper.publishToController(footstepDataList);
      double totalDuration = computeWalkingDuration(footstepDataList);

      assertTrue(simulationTestHelper.simulateNow(totalDuration));

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double desiredHeight = controllerRootJoint.getJointPose().getZ() - 0.2;
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, desiredHeight, worldFrame, worldFrame));

      assertTrue(simulationTestHelper.simulateNow(1.0));

      SimFloatingJointBasics scsRootJoint = simulationTestHelper.getRobot().getFloatingRootJoint();

      // Moving the robot back to the starting position
      scsRootJoint.setPinned(true);
      localRequestedControllerState.set(HighLevelControllerName.STAND_PREP_STATE);
      assertTrue(simulationTestHelper.simulateNow(0.2));

      scsRootJoint.getJointPose().set(aboveInitialPose);

      assertTrue(simulationTestHelper.simulateNow(0.2));

      List<GroundContactPoint> allGCPs = simulationTestHelper.getRobot().getAllJoints().stream()
                                                             .flatMap(j -> j.getAuxialiryData().getGroundContactPoints().stream()).collect(Collectors.toList());

      int count = 0;
      int maxIteration = 100;

      for (double alpha = 0.0; allGCPs.stream().anyMatch(gc -> gc.getPose().getZ() > 0.01); alpha += 0.05)
      { // Putting the robot down gently
         Point3D position = new Point3D();
         position.interpolate(aboveInitialPose.getPosition(), initialPose.getPosition(), alpha);
         scsRootJoint.getJointPose().getPosition().set(position);
         assertTrue(simulationTestHelper.simulateNow(0.025), "Sim crashed, alpha value: " + alpha);

         count++;
         assertTrue(count++ < maxIteration, "Something went wrong when lowering the robot.");
      }

      localRequestedControllerState.set(HighLevelControllerName.WALKING);
      assertTrue(simulationTestHelper.simulateNow(0.2));
      scsRootJoint.setPinned(false);
      assertTrue(simulationTestHelper.simulateNow(2.0));

      EuclidCoreTestTools.assertEquals(initialPose, new Pose3D(scsRootJoint.getJointPose()), 0.05);
   }

   private void doUpRampTest(FootstepDataListCorruptor footstepDataListCorruptor, StepLength stepLength)
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(), simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOverRamp();
      ThreadTools.sleep(1000);

      boolean success = simulationTestHelper.simulateNow(1.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingUpRamp(stepLength);

      if (footstepDataListCorruptor != null)
      {
         footstepDataList = footstepDataListCorruptor.corruptDataList(footstepDataList);
      }

      simulationTestHelper.publishToController(footstepDataList);

      double totalDuration = computeWalkingDuration(footstepDataList);

      success = success && simulationTestHelper.simulateNow(totalDuration);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);

//      simulationTestHelper.checkNothingChanged();
      assertTrue(success);
   }

   private double computeWalkingDuration(FootstepDataListMessage footstepDataList)
   {
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepDuration = walkingControllerParameters.getDefaultTransferTime() + walkingControllerParameters.getDefaultSwingTime();
      double totalDuration = footstepDataList.getFootstepDataList().size() * stepDuration;
      totalDuration += walkingControllerParameters.getDefaultFinalTransferTime() - walkingControllerParameters.getDefaultTransferTime()
            + walkingControllerParameters.getDefaultInitialTransferTime();
      totalDuration += 0.5;
      return totalDuration;
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRamp(StepLength stepLength)
   {
      switch (stepLength)
      {
         case SHORT:
         {
            return createFootstepsForWalkingUpRampShortSteps();
         }

         case MEDIUM:
         {
            return createFootstepsForWalkingUpRampMediumSteps();
         }
         case LONG:
         {
            return createFootstepsForWalkingUpRampLongSteps();
         }

      }

      throw new RuntimeException();
   }

   private void setupCameraForWalkingOverRamp()
   {
      Point3D cameraFix = new Point3D(5.0, -0.2, 0.89);
      Point3D cameraPosition = new Point3D(5.0, 7.8, 1.6);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRampShortSteps()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(2.8351427759651995, -0.09694911182412484, 0.08802441745426925),
                       new Quaternion(-0.003121843074205739, 0.013029344215583364, 0.003893401251972101, 0.9999026611184826)),
            new Pose3D(new Point3D(3.074520627354574, 0.10730668915721077, 0.08352181741194085),
                       new Quaternion(-0.003138368218867929, -0.004117464859445681, 0.003840685316534686, 0.9999792229163331)),
            new Pose3D(new Point3D(3.320864939704435, -0.09193371045698896, 0.08507793790294434),
                       new Quaternion(0.003584052020572927, 0.0017180402341382128, 0.0038475364764751895, 0.9999846995689133)),
            new Pose3D(new Point3D(3.545678648483338, 0.11240071557575276, 0.09590741902024505),
                       new Quaternion(-0.01140194817540447, -0.10273541702708931, 0.002615002207584757, 0.9946399305424701)),
            new Pose3D(new Point3D(3.7906598672926064, -0.08710050380011673, 0.14539339478058277),
                       new Quaternion(-0.0032000970795792102, -0.10503815495588137, 0.0034516427122286324, 0.9944570536452203)),
            new Pose3D(new Point3D(4.033521430244125, 0.11399420124778985, 0.19672970285385633),
                       new Quaternion(0.0014221699189987877, -0.10151659122040031, 0.003938979344512634, 0.9948250316419642)),
            new Pose3D(new Point3D(4.278848103525166, -0.0843365855044057, 0.24733780119159066),
                       new Quaternion(0.002700255557592924, -0.1017216320343805, 0.0040697254090160066, 0.9948009125102776)),
            new Pose3D(new Point3D(4.52241785253613, 0.11448913357058678, 0.29975135146746346),
                       new Quaternion(0.020795142183303052, -0.09404379375500953, 0.005767914571257194, 0.9953341439332021)),
            new Pose3D(new Point3D(4.7669499200426095, -0.08112143629512504, 0.34980817204772424),
                       new Quaternion(0.005929619423401327, -0.09892494034464223, 0.004386419554999209, 0.9950675630904617)),
            new Pose3D(new Point3D(5.007512119256335, 0.11963061986871831, 0.39761424662922423),
                       new Quaternion(0.012520157577486044, -0.10934673784718243, 0.005161876711006989, 0.9939114103405972)),
            new Pose3D(new Point3D(5.253783473785037, -0.07589729216272391, 0.4503040379818391),
                       new Quaternion(-0.002768215802721581, -0.1037404647965144, 0.003502485316391724, 0.9945943824201307)),
            new Pose3D(new Point3D(5.496393274813743, 0.12528689501201606, 0.5016796856627599),
                       new Quaternion(0.0013191212731254657, -0.10174062303095144, 0.003928498337639808, 0.9948023283271665)),
            new Pose3D(new Point3D(5.742142760881751, -0.07169360534955915, 0.5523316338352129),
                       new Quaternion(-0.005390474700669975, -0.09936215014013182, 0.003258174259210824, 0.9950314016163148)),
            new Pose3D(new Point3D(5.983218795105767, 0.1272533065544719, 0.6018939390774926),
                       new Quaternion(0.01190743053633726, -0.1067114604566984, 0.005065817370162187, 0.9942058211454975)),
            new Pose3D(new Point3D(6.227865242475841, -0.06899602335005377, 0.6534736822900773),
                       new Quaternion(8.499381935746491E-4, -0.11096744195086783, 0.003877037816252163, 0.9938161162960275)),
            new Pose3D(new Point3D(6.227890274639702, 0.13309848577114403, 0.6545998273165854),
                       new Quaternion(-0.011553591447604863, -0.1014539720762872, 0.002615910457716054, 0.9947697035430276))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRampMediumSteps()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(2.96830428414849, -0.09766543994755546, 0.08399999999999998),
                       new Quaternion(-2.720672111457406E-20, 5.415915594351002E-20, 0.0018763016130482887, 0.9999982397445792)),
            new Pose3D(new Point3D(3.349989144127723, 0.1048803018182974, 0.084), new Quaternion(0.0, 0.0, 0.0018763016130482887, 0.9999982397445792)),
            new Pose3D(new Point3D(3.733179215666347, -0.09368313850247431, 0.08399999999999998),
                       new Quaternion(-2.720672111457406E-20, 5.415915594351002E-20, 0.0018763016130482887, 0.9999982397445792)),
            new Pose3D(new Point3D(4.097845385981999, 0.10855694052559439, 0.21126371895363616),
                       new Quaternion(-0.004961863853467075, -0.10185092940802969, 0.0013389870407204197, 0.9947863967704449)),
            new Pose3D(new Point3D(4.482409385830916, -0.08979946125987102, 0.2907229850807759),
                       new Quaternion(-0.006178121664549205, -0.09355272624775302, 0.001271081900870645, 0.9955943463957156)),
            new Pose3D(new Point3D(4.862209672750061, 0.10835917126334947, 0.36806892250913914),
                       new Quaternion(0.013196743471666848, -0.1050242695228723, 0.003239072589792082, 0.9943768185057856)),
            new Pose3D(new Point3D(5.245124320290557, -0.0877310654977739, 0.44776346824345264),
                       new Quaternion(-0.001447090852386699, -0.1066362325507391, 0.001688950777778143, 0.9942956136284218)),
            new Pose3D(new Point3D(5.628010222560295, 0.11351051235407777, 0.529646092583943),
                       new Quaternion(-2.9713796970907996E-4, -0.09939803265092252, 0.001818698125945603, 0.9950460467492345)),
            new Pose3D(new Point3D(6.00919845769285, -0.08799087419764393, 0.6029648538145679),
                       new Quaternion(0.0170627239767379, -0.11160025921878289, 0.003757776815021594, 0.9935995796628553)),
            new Pose3D(new Point3D(6.397906847050047, 0.11747882088357856, 0.6897727409562333),
                       new Quaternion(-0.006763453237380005, -0.06919806861450803, 0.0013936914450255998, 0.997579039788068)),
            new Pose3D(new Point3D(6.791772471706161, -0.0820029184512979, 0.709914829576844),
                       new Quaternion(-0.0011948636118072641, -0.005492872382347826, 0.0018696547555749577, 0.999982452368558)),
            new Pose3D(new Point3D(7.16726116567599, 0.13224394932170272, 0.6980262408021847),
                       new Quaternion(-0.07769715333044895, -0.04268274134711664, -0.0014525809551700932, 0.9960618585027782)),
            new Pose3D(new Point3D(7.555321531515414, -0.07933162374116386, 0.7110548590792264),
                       new Quaternion(-2.5158057233426777E-5, -0.013413125296120916, 0.0018754577477367784, 0.9999082808413475)),
            new Pose3D(new Point3D(7.5508615685355345, 0.11059069063054038, 0.7018996990839376),
                       new Quaternion(0.059980529587038575, -0.035821498052187935, 0.00402990721894804, 0.9975484530565731))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRampLongSteps()
   {
      Pose3D[] footstepPoses = {};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForWalkingDownRampMediumSteps()
   {
      Quaternion quaternion = new Quaternion(Math.PI, 0.205, 0.0);
      double[] quatArray = new double[4];
      quaternion.get(quatArray);

      Pose3D[] footstepPoses = {new Pose3D(new Point3D(6.955, 0.12, 0.722), new Quaternion(0.0, 0.0, 1.0, 0.0)),
            new Pose3D(new Point3D(6.571, -0.08, 0.722), new Quaternion(0.0, 0.0, 1.0, 0.0)), new Pose3D(new Point3D(6.175, 0.12, 0.644), quaternion),
            new Pose3D(new Point3D(5.804, -0.08, 0.581), quaternion), new Pose3D(new Point3D(5.419, 0.12, 0.490), quaternion),
            new Pose3D(new Point3D(5.047, -0.08, 0.410), quaternion), new Pose3D(new Point3D(4.671, 0.12, 0.345), quaternion),
            new Pose3D(new Point3D(4.289, -0.08, 0.261), quaternion), new Pose3D(new Point3D(3.912, 0.12, 0.170), quaternion),
            new Pose3D(new Point3D(3.541, -0.08, 0.100), new Quaternion(0.0, 0.0, 1.0, 0.0)),
            new Pose3D(new Point3D(3.170, 0.12, 0.090), new Quaternion(0.0, 0.0, 1.0, 0.0)),
            new Pose3D(new Point3D(3.168, -0.08, 0.090), new Quaternion(0.0, 0.0, 1.0, 0.0)),};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

   protected abstract double getMaxRotationCorruption();

}
