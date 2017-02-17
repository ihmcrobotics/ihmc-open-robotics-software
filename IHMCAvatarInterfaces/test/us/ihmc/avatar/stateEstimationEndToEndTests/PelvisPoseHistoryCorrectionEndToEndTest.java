package us.ihmc.avatar.stateEstimationEndToEndTests;

import static org.junit.Assert.assertTrue;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commons.Conversions;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationToolkit.controllers.OscillateFeetPerturber;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.environments.PointMassRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public abstract class PelvisPoseHistoryCorrectionEndToEndTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   //TODO get that from the StateEstimatorParameters
   private static final boolean USE_ROTATION_CORRECTION = false;

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCFlatGroundWalkingTrack flatGroundWalkingTrack;

   @Before
   public void showMemoryUsageBeforeTest()
   {
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

      if (flatGroundWalkingTrack != null)
      {
         flatGroundWalkingTrack.destroySimulation();
         flatGroundWalkingTrack = null;
      }

      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

//   private final KryoLocalPacketCommunicator kryoLocalObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.CONTROLLER.ordinal(), "PelvisPoseHistoryCorrectionTestLocalControllerCommunicator");
   private final Random random = new Random();
   private FlatGroundEnvironment flatGroundEnvironment;
   private YoVariableRegistry registry;

   private final String simpleFlatGroundScriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;
   private boolean sendPelvisCorrectionPackets = true;
   private HumanoidFloatingRootJointRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private ExternalPelvisPoseCreator externalPelvisPosePublisher;

   private DoubleYoVariable confidenceFactor; // target for alpha filter
   private DoubleYoVariable interpolationTranslationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationTranslationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationTranslationAlphaFilter;
   private DoubleYoVariable interpolationRotationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationRotationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationRotationAlphaFilter;
   private LongYoVariable seNonProcessedPelvisTimeStamp;
   private DoubleYoVariable maxTranslationVelocityClip;
   private DoubleYoVariable maxRotationVelocityClip;
   private DoubleYoVariable translationClippedAlphaValue;
   private DoubleYoVariable rotationClippedAlphaValue;
   private DoubleYoVariable previousTranslationClippedAlphaValue;
   private DoubleYoVariable previousRotationClippedAlphaValue;
   private final double previousTranslationAlphaValue = Double.POSITIVE_INFINITY;
   private final double previousRotationAlphaValue = Double.POSITIVE_INFINITY;

   private DoubleYoVariable pelvisX;
   private DoubleYoVariable pelvisY;
   private DoubleYoVariable pelvisZ;
   private DoubleYoVariable pelvisPitch;
   private DoubleYoVariable pelvisRoll;
   private DoubleYoVariable pelvisYaw;

   private DoubleYoVariable nonCorrectedPelvis_x;
   private DoubleYoVariable nonCorrectedPelvis_y;
   private DoubleYoVariable nonCorrectedPelvis_z;
   private DoubleYoVariable nonCorrectedPelvis_yaw;
   private DoubleYoVariable nonCorrectedPelvis_pitch;
   private DoubleYoVariable nonCorrectedPelvis_roll;
   private DoubleYoVariable correctedPelvis_x;
   private DoubleYoVariable correctedPelvis_y;
   private DoubleYoVariable correctedPelvis_z;
   private DoubleYoVariable correctedPelvis_yaw;
   private DoubleYoVariable correctedPelvis_pitch;
   private DoubleYoVariable correctedPelvis_roll;
   private DoubleYoVariable seBackInTimeFrame_x;
   private DoubleYoVariable seBackInTimeFrame_y;
   private DoubleYoVariable seBackInTimeFrame_z;
   private DoubleYoVariable seBackInTimeFrame_yaw;
   private DoubleYoVariable seBackInTimeFrame_pitch;
   private DoubleYoVariable seBackInTimeFrame_roll;
   private DoubleYoVariable localizationBackInTimeFrame_x;
   private DoubleYoVariable localizationBackInTimeFrame_y;
   private DoubleYoVariable localizationBackInTimeFrame_z;
   private DoubleYoVariable localizationBackInTimeFrame_yaw;
   private DoubleYoVariable localizationBackInTimeFrame_pitch;
   private DoubleYoVariable localizationBackInTimeFrame_roll;

   private DoubleYoVariable totalTranslationErrorFrame_x;
   private DoubleYoVariable totalTranslationErrorFrame_y;
   private DoubleYoVariable totalTranslationErrorFrame_z;
   private DoubleYoVariable totalTranslationErrorFrame_yaw;
   private DoubleYoVariable totalTranslationErrorFrame_pitch;
   private DoubleYoVariable totalTranslationErrorFrame_roll;

   private DoubleYoVariable totalRotationErrorFrame_x;
   private DoubleYoVariable totalRotationErrorFrame_y;
   private DoubleYoVariable totalRotationErrorFrame_z;
   private DoubleYoVariable totalRotationErrorFrame_yaw;
   private DoubleYoVariable totalRotationErrorFrame_pitch;
   private DoubleYoVariable totalRotationErrorFrame_roll;

   private DoubleYoVariable interpolatedTranslationCorrectionFrame_x;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_y;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_z;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_yaw;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_pitch;
   private DoubleYoVariable interpolatedTranslationCorrectionFrame_roll;
   private DoubleYoVariable interpolationTranslationStartFrame_x;
   private DoubleYoVariable interpolationTranslationStartFrame_y;
   private DoubleYoVariable interpolationTranslationStartFrame_z;
   private DoubleYoVariable interpolationTranslationStartFrame_yaw;
   private DoubleYoVariable interpolationTranslationStartFrame_pitch;
   private DoubleYoVariable interpolationTranslationStartFrame_roll;

   private DoubleYoVariable interpolatedRotationCorrectionFrame_x;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_y;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_z;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_yaw;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_pitch;
   private DoubleYoVariable interpolatedRotationCorrectionFrame_roll;
   private DoubleYoVariable interpolationRotationStartFrame_x;
   private DoubleYoVariable interpolationRotationStartFrame_y;
   private DoubleYoVariable interpolationRotationStartFrame_z;
   private DoubleYoVariable interpolationRotationStartFrame_yaw;
   private DoubleYoVariable interpolationRotationStartFrame_pitch;
   private DoubleYoVariable interpolationRotationStartFrame_roll;

   private DoubleYoVariable manualTranslationOffsetX;
   private DoubleYoVariable manualTranslationOffsetY;
   private BooleanYoVariable manuallyTriggerLocalizationUpdate;

//   private BlockingSimulationRunner blockingSimulationRunner;

   @Before
   public void setUp()
   {
      flatGroundEnvironment = new FlatGroundEnvironment();
   }

   private void setupYoVariables(YoVariableRegistry registry, String nameSpace)
   {
      interpolationTranslationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisTranslationErrorCorrectionAlphaFilter");
      interpolationTranslationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationAlphaFilterAlphaValue");
      interpolationTranslationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationAlphaFilterBreakFrequency");
      interpolationRotationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisRotationErrorCorrectionAlphaFilter");
      interpolationRotationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationAlphaFilterAlphaValue");
      interpolationRotationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationAlphaFilterBreakFrequency");
      confidenceFactor = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionConfidenceFactor");
      seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");
      maxTranslationVelocityClip = (DoubleYoVariable)registry.getVariable(nameSpace, "maxTranslationVelocityClip");
      maxRotationVelocityClip = (DoubleYoVariable)registry.getVariable(nameSpace, "maxRotationVelocityClip");
      translationClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "translationClippedAlphaValue");
      rotationClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "rotationClippedAlphaValue");
      previousTranslationClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "previousTranslationClippedAlphaValue");
      previousRotationClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "previousRotationClippedAlphaValue");

      pelvisX = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisX");
      pelvisY = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisY");
      pelvisZ = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisZ");
      pelvisPitch = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisPitch");
      pelvisRoll = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisRoll");
      pelvisYaw = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisYaw");

      nonCorrectedPelvis_x = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_x");
      nonCorrectedPelvis_y = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_y");
      nonCorrectedPelvis_z = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_z");
      nonCorrectedPelvis_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_yaw");
      nonCorrectedPelvis_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_pitch");
      nonCorrectedPelvis_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "nonCorrectedPelvis_roll");
      correctedPelvis_x = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_x");
      correctedPelvis_y = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_y");
      correctedPelvis_z = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_z");
      correctedPelvis_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_yaw");
      correctedPelvis_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_pitch");
      correctedPelvis_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_roll");
      seBackInTimeFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_x");
      seBackInTimeFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_y");
      seBackInTimeFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_z");
      seBackInTimeFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_yaw");
      seBackInTimeFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_pitch");
      seBackInTimeFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "seBackInTimeFrame_roll");
      localizationBackInTimeFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_x");
      localizationBackInTimeFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_y");
      localizationBackInTimeFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_z");
      localizationBackInTimeFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_yaw");
      localizationBackInTimeFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_pitch");
      localizationBackInTimeFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "localizationBackInTimeFrame_roll");

      totalTranslationErrorFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_x");
      totalTranslationErrorFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_y");
      totalTranslationErrorFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_z");
      totalTranslationErrorFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_yaw");
      totalTranslationErrorFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_pitch");
      totalTranslationErrorFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalTranslationErrorFrame_roll");

      totalRotationErrorFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_x");
      totalRotationErrorFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_y");
      totalRotationErrorFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_z");
      totalRotationErrorFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_yaw");
      totalRotationErrorFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_pitch");
      totalRotationErrorFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalRotationErrorFrame_roll");

      interpolatedTranslationCorrectionFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_x");
      interpolatedTranslationCorrectionFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_y");
      interpolatedTranslationCorrectionFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_z");
      interpolatedTranslationCorrectionFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_yaw");
      interpolatedTranslationCorrectionFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_pitch");
      interpolatedTranslationCorrectionFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedTranslationCorrectionFrame_roll");
      interpolationTranslationStartFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_x");
      interpolationTranslationStartFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_y");
      interpolationTranslationStartFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_z");
      interpolationTranslationStartFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_yaw");
      interpolationTranslationStartFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_pitch");
      interpolationTranslationStartFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationTranslationStartFrame_roll");

      interpolatedRotationCorrectionFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_x");
      interpolatedRotationCorrectionFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_y");
      interpolatedRotationCorrectionFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_z");
      interpolatedRotationCorrectionFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_yaw");
      interpolatedRotationCorrectionFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_pitch");
      interpolatedRotationCorrectionFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedRotationCorrectionFrame_roll");
      interpolationRotationStartFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_x");
      interpolationRotationStartFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_y");
      interpolationRotationStartFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_z");
      interpolationRotationStartFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_yaw");
      interpolationRotationStartFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_pitch");
      interpolationRotationStartFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationRotationStartFrame_roll");

      manualTranslationOffsetX = (DoubleYoVariable) registry.getVariable(nameSpace, "manualTranslationOffset_X");
      manualTranslationOffsetY = (DoubleYoVariable) registry.getVariable(nameSpace, "manualTranslationOffset_Y");
      manuallyTriggerLocalizationUpdate = (BooleanYoVariable) registry.getVariable(nameSpace, "manuallyTriggerLocalizationUpdate");

   }

	@ContinuousIntegrationTest(estimatedDuration = 7.8)
	@Test(timeout = 39000)
   public void testPelvisCorrectionControllerOutOfTheLoop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupSimulationWithStandingControllerAndCreateExternalPelvisThread();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015, 0.015);
      maxTranslationVelocityClip.set(1.0);
      maxRotationVelocityClip.set(1.0);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(testInterpolationToRandomTargetsWithFastAlphaValue(robot, registry, 10));

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      sendPelvisCorrectionPackets = false;

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void runPelvisCorrectionControllerOutOfTheLoop() throws SimulationExceededMaximumTimeException
   {
      setupSimulationWithStandingControllerAndCreateExternalPelvisThread();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015, 0.015);
      maxTranslationVelocityClip.set(1.0);
      maxRotationVelocityClip.set(1.0);
      activatePelvisPoseHistoryCorrector(registry, true);
      simulationConstructionSet.simulate();
      while (simulationConstructionSet.isSimulationThreadUpAndRunning())
      {
         ThreadTools.sleep(100);
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 12.1)
	@Test(timeout = 61000)
   public void testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Runnable pelvisCorrectorSource = setupSimulationWithFeetPertuberAndCreateExternalPelvisThread();
      Thread t = new Thread(pelvisCorrectorSource);
      t.start();

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(25.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	/**
	 * Work in progress. Fix these tests in order to make Atlas more robust to Localization drift.
	 *
	 * @throws SimulationExceededMaximumTimeException
	 */
	@ContinuousIntegrationTest(estimatedDuration = 7.7)
	@Test(timeout = 39000)
   public void testBigYawInDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, false);

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(yawBigInDoubleSupport(externalPelvisPosePublisher));

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	/**
	 * Work in progress. Fix these tests in order to make Atlas more robust to Localization drift.
	 *
	 * @throws SimulationExceededMaximumTimeException
	 */
	@ContinuousIntegrationTest(estimatedDuration = 9.4)
	@Test(timeout = 47000)
   public void testBigYawInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, false);

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(yawBigInSingleSupport(externalPelvisPosePublisher));

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(100.0);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


	/**
	 * Work in progress. Fix these tests in order to make Atlas more robust to Localization drift
	 *
	 * @throws SimulationExceededMaximumTimeException
	 */
	@ContinuousIntegrationTest(estimatedDuration = 8.1)
	@Test(timeout = 41000)
   public void testLocalizationOffsetOutsideOfFootInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, false);

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(localizeOutsideOfFootInSingleSupport(externalPelvisPosePublisher));

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(100.0);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private boolean yawBigInSingleSupport(ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
//      FootPosePacket packet = new FootPosePacket(RobotSide.RIGHT, new Point3D(1, 1, 0.3), new Quat4d(), 0.6);
//      drcSimulationTestHelper.send(packet);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2);
      long timeStamp = Conversions.secondsToNanoSeconds(simulationConstructionSet.getTime());
      RigidBodyTransform yawTransform = TransformTools.createTransformFromTranslationAndEulerAngles(1, 1, 0.8, 0, 0, Math.PI);
      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(yawTransform, timeStamp);
      StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1);
      externalPelvisPoseCreator.setNewestPose(posePacket);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10);
   }

   private boolean localizeOutsideOfFootInSingleSupport(ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
//      FootPosePacket packet = new FootPosePacket(RobotSide.RIGHT, new Point3D(1.0, 1.0, 0.3), new Quat4d(), 0.6);
//      drcSimulationTestHelper.send(packet);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      long timeStamp = Conversions.secondsToNanoSeconds(simulationConstructionSet.getTime());
      RigidBodyTransform outsideOfFootTransform = TransformTools.createTransformFromTranslationAndEulerAngles(1.5, 1.0, 0.8, 0.0, 0.0, 0.0);
      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(outsideOfFootTransform, timeStamp);
      StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      externalPelvisPoseCreator.setNewestPose(posePacket);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
   }

   /**
    * Work in progress. Fix these tests in order to make Atlas more robust to Localization drift.
    *
    * @throws SimulationExceededMaximumTimeException
    * @throws ControllerFailureException
    */
	@ContinuousIntegrationTest(estimatedDuration = 5.0)
	@Test(timeout = 30000)
   public void testWalkingDuringBigPelvisCorrection() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      flatGroundWalkingTrack = setupWalkingSim();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 1.0, 1.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 1.0, 1.0);
      activatePelvisPoseHistoryCorrector(registry, true);

      blockingSimulationRunner = new BlockingSimulationRunner(simulationConstructionSet, 60.0 * 10.0);
      blockingSimulationRunner.simulateAndBlock(1.0);

      boolean success = true;

      DoubleYoVariable transferTime = (DoubleYoVariable) registry.getVariable("swingTime");
      DoubleYoVariable swingTime = (DoubleYoVariable) registry.getVariable("transferTime");

      try
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
         transferTime.set(1.2);
         swingTime.set(1.2);
         manualTranslationOffsetX.set(-.5);
         maxTranslationVelocityClip.set(.1);
         maxRotationVelocityClip.set(.1);
         manuallyTriggerLocalizationUpdate.set(true);
         blockingSimulationRunner.simulateAndBlock(20.0);
      }
      catch (SimulationExceededMaximumTimeException e)
      {
         success = false;
      }


      assertTrue(success);
      sendPelvisCorrectionPackets = false;

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private boolean yawBigInDoubleSupport(ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      long timeStamp = Conversions.secondsToNanoSeconds(simulationConstructionSet.getTime());
      RigidBodyTransform yawTransform = TransformTools.createTransformFromTranslationAndEulerAngles(1, 1, 0.8, 0, 0, Math.PI);
      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(yawTransform, timeStamp);
      StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1);
      externalPelvisPoseCreator.setNewestPose(posePacket);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10);
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   public void createSCS()
   {
      Robot robot = new PointMassRobot();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      simulationConstructionSet = new SimulationConstructionSet(robot, parameters);

      Thread myThread = new Thread(simulationConstructionSet);
      myThread.start();
   }


   private RigidBodyTransform[] createRandomCorrectionTargets(int numTargets)
   {
      RigidBodyTransform[] targets = new RigidBodyTransform[numTargets];
      for (int i = 0; i < numTargets; i++)
      {
         targets[i] = new RigidBodyTransform();
         targets[i].setRotationEulerAndZeroTranslation(0, 0, random.nextDouble() * 2.0 * Math.PI);
         targets[i].setTranslation(RandomTools.generateRandomVector(random, 1.0));
      }
      return targets;
   }

   private boolean testInterpolationToRandomTargetsWithFastAlphaValue(final Robot robot, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5.0, 5.0);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100.0, 100.0);

      if(USE_ROTATION_CORRECTION)
         return testTranslationAndRotationInterpolationToRandomTargets(robot, registry, numTargets);
      else
         return testTranslationInterpolationToRandomTargets(robot, registry, numTargets);
   }


   //TODO retune the test because of the change of rotation handling (which now has its own alphaFilter) in the PelvisPoseHistoryCorrection
   private boolean testTranslationAndRotationInterpolationToRandomTargets(final Robot robot, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {
      YoFramePose target = new YoFramePose("target_", ReferenceFrame.getWorldFrame(), registry);
      RigidBodyTransform[] targets = createRandomCorrectionTargets(numTargets);
      boolean success = true;

      Vector3D targetTranslation = new Vector3D();
      Quaternion targetQuat = new Quaternion();
      double[] yawPitchRoll = new double[3];
      double translationFudgeFactor = 0.015;
      double rotationFudgeFactor = 0.015;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 10);
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetQuat);

         YawPitchRollConversion.convertQuaternionToYawPitchRoll(targetQuat, yawPitchRoll);
         target.setYawPitchRoll(yawPitchRoll);
         target.setXYZ(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ());

         long timeStamp = Conversions.secondsToNanoSeconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 3);
         externalPelvisPosePublisher.setNewestPose(posePacket);

         while (translationClippedAlphaValue.getDoubleValue() > 0.2 || rotationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         }

         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999 && rotationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 100);
         }

         double xError = Math.abs(pelvisX.getDoubleValue() - targetTranslation.getX());
         double yError = Math.abs(pelvisY.getDoubleValue() - targetTranslation.getY());
         double zError = Math.abs(pelvisZ.getDoubleValue() - targetTranslation.getZ());
         double yawError = Math.abs(pelvisYaw.getDoubleValue() - yawPitchRoll[0]);
         double pitchError = Math.abs(pelvisPitch.getDoubleValue() - yawPitchRoll[1]);
         double rollError = Math.abs(pelvisRoll.getDoubleValue() - yawPitchRoll[2]);

         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         if (yawError > largestError)
            largestError = yawError;
         if (pitchError > largestError)
            largestError = pitchError;
         if (rollError > largestError)
            largestError = rollError;

         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= pitchError <= rotationFudgeFactor;
         success &= rollError <= rotationFudgeFactor;
         success &= yawError <= rotationFudgeFactor;
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }

   private boolean testTranslationInterpolationToRandomTargets(final Robot robot, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {
      YoFramePose target = new YoFramePose("target_", ReferenceFrame.getWorldFrame(), registry);
      RigidBodyTransform[] targets = createRandomCorrectionTargets(numTargets);
      boolean success = true;

      Vector3D targetTranslation = new Vector3D();
      Quaternion targetQuat = new Quaternion();
      double[] yawPitchRoll = new double[3];
      double translationFudgeFactor = 0.015;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 10);
         targets[i].getTranslation(targetTranslation);
         targets[i].getRotation(targetQuat);

         YawPitchRollConversion.convertQuaternionToYawPitchRoll(targetQuat, yawPitchRoll);
         target.setYawPitchRoll(yawPitchRoll);
         target.setXYZ(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ());

         long timeStamp = Conversions.secondsToNanoSeconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 3);
         externalPelvisPosePublisher.setNewestPose(posePacket);

         double yawErrorBeforeCorrection = Math.abs(pelvisYaw.getDoubleValue() - yawPitchRoll[0]);
         double pitchErrorBeforeCorrection = Math.abs(pelvisPitch.getDoubleValue() - yawPitchRoll[1]);
         double rollErrorBeforeCorrection = Math.abs(pelvisRoll.getDoubleValue() - yawPitchRoll[2]);

         while (translationClippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         }

         while (translationClippedAlphaValue.getDoubleValue() < 0.9999999)
         {
            success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 100);
         }

         double xError = Math.abs(pelvisX.getDoubleValue() - targetTranslation.getX());
         double yError = Math.abs(pelvisY.getDoubleValue() - targetTranslation.getY());
         double zError = Math.abs(pelvisZ.getDoubleValue() - targetTranslation.getZ());
         double yawErrorAfterCorrection = Math.abs(pelvisYaw.getDoubleValue() - yawPitchRoll[0]);
         double pitchErrorAfterCorrection = Math.abs(pelvisPitch.getDoubleValue() - yawPitchRoll[1]);
         double rollErrorAfterCorrection = Math.abs(pelvisRoll.getDoubleValue() - yawPitchRoll[2]);

         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;

         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= MathTools.epsilonEquals(yawErrorBeforeCorrection, yawErrorAfterCorrection, rotationFudgeFactor);
         success &= MathTools.epsilonEquals(pitchErrorBeforeCorrection, pitchErrorAfterCorrection, rotationFudgeFactor);
         success &= MathTools.epsilonEquals(rollErrorBeforeCorrection, rollErrorAfterCorrection, rotationFudgeFactor);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }

   private Runnable createPelvisCorrectorProducerUsingSCSActual(final FloatingRootJointRobot robot,
         final ExternalPelvisPoseCreator externalPelvisPoseCreator)
   {
      Runnable pelvisCorrectorSource = new Runnable()
      {

         FloatingJoint pelvis = robot.getRootJoint();
         RigidBodyTransform pelvisTransform = new RigidBodyTransform();

         @Override
         public void run()
         {
            while (running())
            {
               try
               {
                  Thread.sleep(900);
                  pelvisTransform.set(pelvis.getJointTransform3D());
                  long timeStamp = Conversions.secondsToNanoSeconds(simulationConstructionSet.getTime());
                  Thread.sleep((int) (random.nextDouble() * 200));
                  TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(pelvisTransform, timeStamp);
                  StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
                  externalPelvisPoseCreator.setNewestPose(posePacket);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }
         }
      };
      return pelvisCorrectorSource;
   }

   private OscillateFeetPerturber generateFeetPertuber(final SimulationConstructionSet simulationConstructionSet, HumanoidFloatingRootJointRobot robot, int ticksPerPerturbation)
   {
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[] { 0.008, 0.012, 0.005 });
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] { 0.010, 0.06, 0.010 });

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] { 1.0, 2.5, 3.3 });
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] { 2.0, 0.5, 1.3 });

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] { 5.0, 0.5, 0.3 });
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] { 0.2, 3.4, 1.11 });
      return oscillateFeetPerturber;
   }

   private boolean running()
   {
      return sendPelvisCorrectionPackets;
   }

   private void setPelvisPoseHistoryCorrectorAlphaBreakFreq(YoVariableRegistry registry, double breakFrequencyTranslation, double breakFrequencyRotation)
   {
      DoubleYoVariable pelvisTranslationCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationTranslationAlphaFilterBreakFrequency");
      pelvisTranslationCorrectorAlphaFilterBF.set(breakFrequencyTranslation);

      DoubleYoVariable pelvisRotationCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationRotationAlphaFilterBreakFrequency");
      pelvisRotationCorrectorAlphaFilterBF.set(breakFrequencyRotation);
   }

   private void setPelvisPoseHistoryCorrectorMaxVelocity(YoVariableRegistry registry, double maxVelocityTranslation, double maxVelocityRotation)
   {
      DoubleYoVariable maxTranslationVelocityCap = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "maxTranslationVelocityClip");
      maxTranslationVelocityCap.set(maxVelocityTranslation);

      DoubleYoVariable maxRotationVelocityCap = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "maxRotationVelocityClip");
      maxRotationVelocityCap.set(maxVelocityRotation);
   }

   private void activatePelvisPoseHistoryCorrector(YoVariableRegistry registry, boolean activate)
   {
      BooleanYoVariable useExternalPelvisCorrector = (BooleanYoVariable) registry.getVariable("DRCKinematicsBasedStateEstimator", "useExternalPelvisCorrector");
      useExternalPelvisCorrector.set(activate);
   }

   private void setupSim(DRCObstacleCourseStartingLocation startingLocation, boolean useScript) throws SimulationExceededMaximumTimeException
   {
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, "PelvisCorrectionTest",
            startingLocation, simulationTestingParameters, getRobotModel());
      if (useScript)
      {
         String scriptName = simpleFlatGroundScriptName;
         FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001);
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));
      }
      simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      robot = drcSimulationTestHelper.getRobot();
      registry = robot.getRobotsYoVariableRegistry();

      externalPelvisPosePublisher = new ExternalPelvisPoseCreator();
      AvatarSimulation avatarSimulation = drcSimulationTestHelper.getAvatarSimulation();
      avatarSimulation.setExternalPelvisCorrectorSubscriber(externalPelvisPosePublisher);

      setupCameraForWalkingUpToRamp();
   }

   private DRCFlatGroundWalkingTrack setupWalkingSim()
   {
      DRCRobotModel robotModel = getRobotModel();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);
      GroundProfile3D groundProfile = new FlatGroundProfile();
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(false);

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotModel.getDefaultRobotInitialSetup(0.0, 0.0), guiInitialSetup,
            scsInitialSetup, true, false,
            getRobotModel());

      simulationConstructionSet = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      robot = drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot();
      registry = robot.getRobotsYoVariableRegistry();

      externalPelvisPosePublisher = new ExternalPelvisPoseCreator();
      AvatarSimulation avatarSimulation = drcFlatGroundWalkingTrack.getAvatarSimulation();
      avatarSimulation.setExternalPelvisCorrectorSubscriber(externalPelvisPosePublisher);
      BooleanYoVariable walk = (BooleanYoVariable) simulationConstructionSet.getVariable("walk");
      walk.set(true);
      return drcFlatGroundWalkingTrack;
   }

   private Runnable setupSimulationWithFeetPertuberAndCreateExternalPelvisThread() throws SimulationExceededMaximumTimeException
   {
      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, true);
      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeetPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      Runnable pelvisCorrectorSource = createPelvisCorrectorProducerUsingSCSActual(robot,
            externalPelvisPosePublisher);
      return pelvisCorrectorSource;
   }

   private void setupSimulationWithStandingControllerAndCreateExternalPelvisThread() throws SimulationExceededMaximumTimeException
   {
      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, false);
      StandStillDoNothingPelvisPoseHistoryCorrectorController robotController = new StandStillDoNothingPelvisPoseHistoryCorrectorController();
      robot.setController(robotController, 1);

      registry = robot.getRobotsYoVariableRegistry();
   }

   private class ExternalPelvisPoseCreator implements PelvisPoseCorrectionCommunicatorInterface
   {
      private StampedPosePacket newestStampedPosePacket;
      boolean newPose;

      public void setNewestPose(StampedPosePacket newestStampedPosePacket)
      {
         this.newestStampedPosePacket = newestStampedPosePacket;
         newPose = true;
      }

      @Override
      public boolean hasNewPose()
      {
         return newPose;
      }

      @Override
      public StampedPosePacket getNewExternalPose()
      {
         newPose = false;
         return this.newestStampedPosePacket;
      }

      @Override
      public void receivedPacket(StampedPosePacket object)
      {
         //doNothing
      }

      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
      {
        //doNothing
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
         // TODO Auto-generated method stub

      }
   }

   private class StandStillDoNothingPelvisPoseHistoryCorrectorController implements RobotController
   {
      private final YoVariableRegistry controllerRegistry = new YoVariableRegistry(getName());
      private final DRCRobotJointMap jointMap;
      private final LinkedHashMap<OneDegreeOfFreedomJoint, Double> qDesireds;
      private final ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints;

      private final EnumYoVariable<HighLevelState> requestedHighLevelState;


      public StandStillDoNothingPelvisPoseHistoryCorrectorController()
      {
         requestedHighLevelState = (EnumYoVariable<HighLevelState>) simulationConstructionSet.getVariable(
               HighLevelHumanoidControllerManager.class.getSimpleName(), "requestedHighLevelState");
         requestedHighLevelState.set(HighLevelState.DO_NOTHING_BEHAVIOR);

         jointMap = getRobotModel().getJointMap();

         Vector3D robotLocation = new Vector3D();

         qDesireds = new LinkedHashMap<>();
         oneDegreeOfFreedomJoints = new ArrayList<>();
         robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

         for (OneDegreeOfFreedomJoint joint : oneDegreeOfFreedomJoints)
         {
            qDesireds.put(joint, joint.getQYoVariable().getDoubleValue());
         }
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return new YoVariableRegistry("Dummy");
      }

      @Override
      public String getName()
      {
         return "Dummy";
      }

      @Override
      public String getDescription()
      {
         return null;
      }

      @Override
      public void doControl()
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            OneDegreeOfFreedomJoint shoulderYaw = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW));
            shoulderYaw.setKp(200.0);
            shoulderYaw.setKd(20.0);
            shoulderYaw.setqDesired(qDesireds.get(shoulderYaw));
            OneDegreeOfFreedomJoint shoulderRoll = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL));
            shoulderRoll.setKp(200.0);
            shoulderRoll.setKd(20.0);
            shoulderRoll.setqDesired(qDesireds.get(shoulderRoll));
            OneDegreeOfFreedomJoint elbowPitch = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH));
            elbowPitch.setKp(200.0);
            elbowPitch.setKd(20.0);
            elbowPitch.setqDesired(qDesireds.get(elbowPitch));
            OneDegreeOfFreedomJoint elbowRoll = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL));
            elbowRoll.setKp(200.0);
            elbowRoll.setKd(20.0);
            elbowRoll.setqDesired(qDesireds.get(elbowRoll));
            OneDegreeOfFreedomJoint wristPitch = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH));
            wristPitch.setKp(20.0);
            wristPitch.setKd(2.0);
            wristPitch.setqDesired(qDesireds.get(wristPitch));
            OneDegreeOfFreedomJoint wristRoll = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL));
            wristRoll.setKp(20.0);
            wristRoll.setKd(2.0);
            wristRoll.setqDesired(qDesireds.get(wristRoll));

            OneDegreeOfFreedomJoint hipPitch = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH));
            hipPitch.setKp(5000.0);
            hipPitch.setKd(150.0);
            hipPitch.setqDesired(qDesireds.get(hipPitch));
            OneDegreeOfFreedomJoint hipRoll = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL));
            hipRoll.setKp(500.0);
            hipRoll.setKd(50.0);
            hipRoll.setqDesired(qDesireds.get(hipRoll));
            OneDegreeOfFreedomJoint hipYaw = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW));
            hipYaw.setKp(100.0);
            hipYaw.setKd(10.0);
            hipYaw.setqDesired(qDesireds.get(hipYaw));
            OneDegreeOfFreedomJoint knee = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH));
            knee.setKp(5000.0);
            knee.setKd(300.0);
            knee.setqDesired(qDesireds.get(knee));
            OneDegreeOfFreedomJoint anklePitch = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH));
            anklePitch.setKp(2000.0);
            anklePitch.setKd(200.0);
            anklePitch.setqDesired(qDesireds.get(anklePitch));
            OneDegreeOfFreedomJoint ankleRoll = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL));
            ankleRoll.setKp(100.0);
            ankleRoll.setKd(10.0);
            ankleRoll.setqDesired(qDesireds.get(ankleRoll));
         }

         OneDegreeOfFreedomJoint spinePitch = robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH));
         spinePitch.setKp(5000.0);
         spinePitch.setKd(300.0);
         spinePitch.setqDesired(qDesireds.get(spinePitch));
         OneDegreeOfFreedomJoint spineRoll = robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL));
         spineRoll.setKp(1000.0);
         spineRoll.setKd(100.0);
         spineRoll.setqDesired(qDesireds.get(spineRoll));
         OneDegreeOfFreedomJoint spineYaw = robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW));
         spineYaw.setKp(500.0);
         spineYaw.setKd(50.0);
         spineYaw.setqDesired(qDesireds.get(spineYaw));
      }

   }
}
