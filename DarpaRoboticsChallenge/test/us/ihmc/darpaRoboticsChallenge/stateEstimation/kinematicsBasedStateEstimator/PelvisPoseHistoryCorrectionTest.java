package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.util.OscillateFeetPerturber;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.environments.PointMassRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.net.KryoLocalObjectCommunicator;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public abstract class PelvisPoseHistoryCorrectionTest implements MultiRobotTestInterface
{

   private static final boolean KEEP_SCS_UP = false;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = true;//KEEP_SCS_UP || createMovie;

   private KryoLocalObjectCommunicator kryoLocalObjectCommunicator = new KryoLocalObjectCommunicator(new IHMCCommunicationKryoNetClassList());
   private final Random random = new Random();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private FlatGroundEnvironment flatGroundEnvironment;
   private YoVariableRegistry registry;

   private final String simpleFlatGroundScriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;
   private boolean sendPelvisCorrectionPackets = true;
   private SDFRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private ExternalPelvisPoseCreator externalPelvisPosePublisher;

   private DoubleYoVariable confidenceFactor; // target for alpha filter
   private DoubleYoVariable interpolationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationAlphaFilter;
   private LongYoVariable seNonProcessedPelvisTimeStamp;
   private DoubleYoVariable maxVelocityClip;
   private DoubleYoVariable clippedAlphaValue;
   private DoubleYoVariable previousClippedAlphaValue;
   private double previousAlphaValue = Double.POSITIVE_INFINITY;
   
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
   private DoubleYoVariable totalErrorFrame_x;
   private DoubleYoVariable totalErrorFrame_y;
   private DoubleYoVariable totalErrorFrame_z;
   private DoubleYoVariable totalErrorFrame_yaw;
   private DoubleYoVariable totalErrorFrame_pitch;
   private DoubleYoVariable totalErrorFrame_roll;
   private DoubleYoVariable interpolatedCorrectionFrame_x;
   private DoubleYoVariable interpolatedCorrectionFrame_y;
   private DoubleYoVariable interpolatedCorrectionFrame_z;
   private DoubleYoVariable interpolatedCorrectionFrame_yaw;
   private DoubleYoVariable interpolatedCorrectionFrame_pitch;
   private DoubleYoVariable interpolatedCorrectionFrame_roll;
   private DoubleYoVariable interpolationStartFrame_x;
   private DoubleYoVariable interpolationStartFrame_y;
   private DoubleYoVariable interpolationStartFrame_z;
   private DoubleYoVariable interpolationStartFrame_yaw;
   private DoubleYoVariable interpolationStartFrame_pitch;
   private DoubleYoVariable interpolationStartFrame_roll;
   private DoubleYoVariable manualTranslationOffsetX;
   private DoubleYoVariable manualTranslationOffsetY;
   private BooleanYoVariable manuallyTriggerLocalizationUpdate;

   private BlockingSimulationRunner blockingSimulationRunner;

   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @Before
   public void setUp()
   {
      flatGroundEnvironment = new FlatGroundEnvironment();
      showMemoryUsageBeforeTest();
   }

   private void setupYoVariables(YoVariableRegistry registry, String nameSpace)
   {
      interpolationAlphaFilter = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionAlphaFilter");
      interpolationAlphaFilterAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationAlphaFilterAlphaValue");
      interpolationAlphaFilterBreakFrequency = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationAlphaFilterBreakFrequency");
      confidenceFactor = (DoubleYoVariable) registry.getVariable(nameSpace, "PelvisErrorCorrectionConfidenceFactor");
      seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");
      maxVelocityClip = (DoubleYoVariable) registry.getVariable(nameSpace, "maxVelocityClip");
      clippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "clippedAlphaValue");
      previousClippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "previousClippedAlphaValue");

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
      totalErrorFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_x");
      totalErrorFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_y");
      totalErrorFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_z");
      totalErrorFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_yaw");
      totalErrorFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_pitch");
      totalErrorFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorFrame_roll");
      interpolatedCorrectionFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_x");
      interpolatedCorrectionFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_y");
      interpolatedCorrectionFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_z");
      interpolatedCorrectionFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_yaw");
      interpolatedCorrectionFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_pitch");
      interpolatedCorrectionFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedCorrectionFrame_roll");
      interpolationStartFrame_x = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationStartFrame_x");
      interpolationStartFrame_y = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationStartFrame_y");
      interpolationStartFrame_z = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationStartFrame_z");
      interpolationStartFrame_yaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationStartFrame_yaw");
      interpolationStartFrame_pitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationStartFrame_pitch");
      interpolationStartFrame_roll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolationStartFrame_roll");
      manualTranslationOffsetX = (DoubleYoVariable) registry.getVariable(nameSpace, "manualTranslationOffset_X");
      manualTranslationOffsetY = (DoubleYoVariable) registry.getVariable(nameSpace, "manualTranslationOffset_Y");
      manuallyTriggerLocalizationUpdate = (BooleanYoVariable) registry.getVariable(nameSpace, "manuallyTriggerLocalizationUpdate");

   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
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
   public void testPelvisCorrectionControllerOutOfTheLoop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      setupSimulationWithStandingControllerAndCreateExternalPelvisThread();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
      maxVelocityClip.set(1.0);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(testInterpolationToRandomTargetsWithFastAlphaValue(robot, registry, 10));

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      sendPelvisCorrectionPackets = false;
      
      assertTrue(success);
      BambooTools.reportTestFinishedMessage();
   }

   public void runPelvisCorrectionControllerOutOfTheLoop() throws SimulationExceededMaximumTimeException
   {
      setupSimulationWithStandingControllerAndCreateExternalPelvisThread();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
      maxVelocityClip.set(1.0);
      activatePelvisPoseHistoryCorrector(registry, true);
      simulationConstructionSet.simulate();
      while (simulationConstructionSet.isSimulationThreadUpAndRunning())
      {
         ThreadTools.sleep(100);
      }

   }

   @Test
   public void testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      Runnable pelvisCorrectorSource = setupSimulationWithFeetPertuberAndCreateExternalPelvisThread();
      Thread t = new Thread(pelvisCorrectorSource);
      t.start();

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(25.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }


   @Test
   public void testBigYawInDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, false);

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(yawBigInDoubleSupport(externalPelvisPosePublisher));

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testBigYawInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, false);

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(yawBigInSingleSupport(externalPelvisPosePublisher));

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(100.0);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testLocalizationOffsetOutsideOfFootInSingleSupport() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, false);

      ThreadTools.sleep(1000);

      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 0.015);
      activatePelvisPoseHistoryCorrector(registry, true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(localizeOutsideOfFootInSingleSupport(externalPelvisPosePublisher));

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(100.0);
      drcSimulationTestHelper.checkNothingChanged();

      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private boolean yawBigInSingleSupport(ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      FootPosePacket packet = new FootPosePacket(RobotSide.RIGHT, new Point3d(1, 1, 0.3), new Quat4d());
      kryoLocalObjectCommunicator.consumeObject(packet);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2);
      long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
      RigidBodyTransform yawTransform = TransformTools.createTransformFromTranslationAndEulerAngles(1, 1, 0.8, 0, 0, Math.PI);
      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(yawTransform, timeStamp);
      StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1);
      externalPelvisPoseCreator.setNewestPose(posePacket);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10);
   }

   private boolean localizeOutsideOfFootInSingleSupport(ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      FootPosePacket packet = new FootPosePacket(RobotSide.RIGHT, new Point3d(1, 1, 0.3), new Quat4d());
      kryoLocalObjectCommunicator.consumeObject(packet);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2);
      long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
      RigidBodyTransform outsideOfFootTransform = TransformTools.createTransformFromTranslationAndEulerAngles(1.5, 1, 0.8, 0, 0, 0);
      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(outsideOfFootTransform, timeStamp);
      StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1);
      externalPelvisPoseCreator.setNewestPose(posePacket);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10);
   }

   @Test
   public void testWalkingDuringBigPelvisCorrection() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      setupWalkingSim();
      setupYoVariables(registry, "PelvisPoseHistoryCorrection");
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 1);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 1);
      activatePelvisPoseHistoryCorrector(registry, true);

      blockingSimulationRunner.simulateAndBlock(1.0);

      assertTrue(walkForwardAndLocalizeInFrontOfFoot());
      sendPelvisCorrectionPackets = false;

      BambooTools.reportTestFinishedMessage();
   }

   private boolean walkForwardAndLocalizeInFrontOfFoot()
   {
      DoubleYoVariable transferTime = (DoubleYoVariable) registry.getVariable("swingTime");
      DoubleYoVariable swingTime = (DoubleYoVariable) registry.getVariable("transferTime");

      try
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
         transferTime.set(1.2);
         swingTime.set(1.2);
         manualTranslationOffsetX.set(-.5);
         maxVelocityClip.set(.1);
         manuallyTriggerLocalizationUpdate.set(true);
         blockingSimulationRunner.simulateAndBlock(20.0);
      }
      catch (SimulationExceededMaximumTimeException e)
      {
         return false;
      }
      return true;
   }

   private boolean yawBigInDoubleSupport(ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
      RigidBodyTransform yawTransform = TransformTools.createTransformFromTranslationAndEulerAngles(1, 1, 0.8, 0, 0, Math.PI);
      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(yawTransform, timeStamp);
      StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1);
      externalPelvisPoseCreator.setNewestPose(posePacket);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10);
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   public void createSCS()
   {
      Robot robot = new PointMassRobot();
      simulationConstructionSet = new SimulationConstructionSet(robot, true);

      Thread myThread = new Thread(simulationConstructionSet);
      myThread.start();
   }


   private RigidBodyTransform[] createRandomCorrectionTargets(int numTargets)
   {
      RigidBodyTransform[] targets = new RigidBodyTransform[numTargets];
      for (int i = 0; i < numTargets; i++)
      {
         targets[i] = new RigidBodyTransform();
         targets[i].setEuler(0, 0, random.nextDouble() * 2 * Math.PI);
         targets[i].setTranslation(RandomTools.generateRandomVector(random, 1));
      }
      return targets;
   }

   private boolean testInterpolationToRandomTargetsWithFastAlphaValue(final Robot robot, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, 5);
      setPelvisPoseHistoryCorrectorMaxVelocity(registry, 100);

      return testInterpolationToRandomTargets(robot, registry, numTargets);
   }

   private boolean testInterpolationToRandomTargets(final Robot robot, YoVariableRegistry registry, int numTargets)
         throws SimulationExceededMaximumTimeException
   {


      YoFramePose target = new YoFramePose("target_", ReferenceFrame.getWorldFrame(), registry);
      RigidBodyTransform[] targets = createRandomCorrectionTargets(numTargets);
      boolean success = true;

      Vector3d targetTranslation = new Vector3d();
      Quat4d targetQuat = new Quat4d();
      double[] yawPitchRoll = new double[3];
      double translationFudgeFactor = 0.015;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 10);
         targets[i].get(targetTranslation);
         targets[i].get(targetQuat);

         RotationFunctions.setYawPitchRollBasedOnQuaternion(yawPitchRoll, targetQuat);
         target.setYawPitchRoll(yawPitchRoll);
         target.setXYZ(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ());

         long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(targets[i], timeStamp);
         StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);

         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 3);
         externalPelvisPosePublisher.setNewestPose(posePacket);

         while (clippedAlphaValue.getDoubleValue() > 0.2)
         {
            success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         }

         while (clippedAlphaValue.getDoubleValue() < 0.9999999)
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

   private Runnable createPelvisCorrectorProducerUsingSCSActual(final SDFRobot robot,
         final ExternalPelvisPoseCreator externalPelvisPoseCreator)
   {
      Runnable pelvisCorrectorSource = new Runnable()
      {

         FloatingJoint pelvis = robot.getPelvisJoint();
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
                  long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
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

   private OscillateFeetPerturber generateFeetPertuber(final SimulationConstructionSet simulationConstructionSet, SDFRobot robot, int ticksPerPerturbation)
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

   private void setPelvisPoseHistoryCorrectorAlphaBreakFreq(YoVariableRegistry registry, double breakFrequency)
   {
      DoubleYoVariable pelvisCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
            "interpolationAlphaFilterBreakFrequency");
      pelvisCorrectorAlphaFilterBF.set(breakFrequency);
   }

   private void setPelvisPoseHistoryCorrectorMaxVelocity(YoVariableRegistry registry, double maxVel)
   {
      DoubleYoVariable maxVelocityCap = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection",
 "maxVelocityClip");
      maxVelocityCap.set(maxVel);
   }

   private void activatePelvisPoseHistoryCorrector(YoVariableRegistry registry, boolean activate)
   {
      BooleanYoVariable useExternalPelvisCorrector = (BooleanYoVariable) registry.getVariable("DRCKinematicsBasedStateEstimator", "useExternalPelvisCorrector");
      useExternalPelvisCorrector.set(activate);
   }
   
   private void setupSim(DRCObstacleCourseStartingLocation startingLocation, boolean useScript)
   {
      String script = "";
      if (useScript)
      {
         script = simpleFlatGroundScriptName;
      }
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, kryoLocalObjectCommunicator, "PelvisCorrectionTest", script,
            startingLocation, checkNothingChanged, showGUI, createMovie, false, getRobotModel());
      simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      robot = drcSimulationTestHelper.getRobot();
      registry = robot.getRobotsYoVariableRegistry();

      externalPelvisPosePublisher = new ExternalPelvisPoseCreator();
      DRCSimulationFactory drcSimulationFactory = drcSimulationTestHelper.getDRCSimulationFactory();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPosePublisher);

      setupCameraForWalkingUpToRamp();
   }

   private void setupWalkingSim()
   {
      DRCRobotModel robotModel = getRobotModel();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
      guiInitialSetup.setIsGuiShown(showGUI);
      GroundProfile3D groundProfile = new FlatGroundProfile();
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(false);

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotModel.getDefaultRobotInitialSetup(0.0, 0.0), guiInitialSetup,
            scsInitialSetup, true, false,
            getRobotModel());

      simulationConstructionSet = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      robot = drcFlatGroundWalkingTrack.getDrcSimulation().getRobot();
      registry = robot.getRobotsYoVariableRegistry();

      externalPelvisPosePublisher = new ExternalPelvisPoseCreator();
      DRCSimulationFactory drcSimulationFactory = drcFlatGroundWalkingTrack.getDrcSimulation();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPosePublisher);
      blockingSimulationRunner = new BlockingSimulationRunner(simulationConstructionSet, 1000.0);
      BooleanYoVariable walk = (BooleanYoVariable) simulationConstructionSet.getVariable("walk");
      walk.set(true);
   }

   private Runnable setupSimulationWithFeetPertuberAndCreateExternalPelvisThread()
   {
      setupSim(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, true);
      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeetPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      Runnable pelvisCorrectorSource = createPelvisCorrectorProducerUsingSCSActual(robot,
            externalPelvisPosePublisher);
      return pelvisCorrectorSource;
   }

   private void setupSimulationWithStandingControllerAndCreateExternalPelvisThread()
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
      public void consumeObject(StampedPosePacket object)
      {
         //doNothing
      }
      
      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket) 
      {
        //doNothing
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

         Vector3d robotLocation = new Vector3d();

         qDesireds = new LinkedHashMap<>();
         oneDegreeOfFreedomJoints = new ArrayList<>();
         robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

         for (OneDegreeOfFreedomJoint joint : oneDegreeOfFreedomJoints)
         {
            qDesireds.put(joint, joint.getQ().getDoubleValue());
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
            OneDegreeOfFreedomJoint shoulderPitch = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH));
            shoulderPitch.setKp(200.0);
            shoulderPitch.setKd(20.0);
            shoulderPitch.setqDesired(qDesireds.get(shoulderPitch));
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
            OneDegreeOfFreedomJoint wristPitch = robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_PITCH));
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
            OneDegreeOfFreedomJoint knee = robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE));
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
