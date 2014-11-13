package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.communication.subscribers.TimeStampedPelvisPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepDataListObjectCommunicator;
import us.ihmc.darpaRoboticsChallenge.util.OscillateFeetPerturber;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.environments.PointMassRobot;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public abstract class PelvisPoseHistoryCorrectionTest implements MultiRobotTestInterface
{

   private static final boolean KEEP_SCS_UP = false;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = true;//KEEP_SCS_UP || createMovie;

   private final Random random = new Random();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCSimulationNetworkTestHelper drcNetworkingSimulationTestHelper;
   private FlatGroundEnvironment flatGroundEnvironment;
   private YoVariableRegistry registry;

   private final String simpleFlatGroundScriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";

   private boolean sendPelvisCorrectionPackets = true;
   private SDFRobot robot;
   private SimulationConstructionSet simulationConstructionSet;
   private ExternalPelvisPoseCreator externalPelvisPosePublisher;

   private DoubleYoVariable seNonProcessedPelvisQuaternionX;
   private DoubleYoVariable seNonProcessedPelvisQuaternionY;
   private DoubleYoVariable seNonProcessedPelvisQuaternionZ;
   private DoubleYoVariable seNonProcessedPelvisQuaternionS;

   private DoubleYoVariable confidenceFactor; // target for alpha filter
   private DoubleYoVariable interpolationAlphaFilterBreakFrequency;
   private DoubleYoVariable interpolationAlphaFilterAlphaValue;
   private DoubleYoVariable interpolationAlphaFilter;

   private DoubleYoVariable externalPelvisPositionX;
   private DoubleYoVariable externalPelvisPositionY;
   private DoubleYoVariable externalPelvisPositionZ;
   private DoubleYoVariable externalPelvisPitch;
   private DoubleYoVariable externalPelvisRoll;
   private DoubleYoVariable externalPelvisYaw;

   private DoubleYoVariable totalErrorPelvisPositionX;
   private DoubleYoVariable totalErrorPelvisPositionY;
   private DoubleYoVariable totalErrorPelvisPositionZ;
   private DoubleYoVariable totalErrorPelvisPitch;
   private DoubleYoVariable totalErrorPelvisRoll;
   private DoubleYoVariable totalErrorPelvisYaw;

   private DoubleYoVariable interpolatedPelvisErrorPositionX;
   private DoubleYoVariable interpolatedPelvisErrorPositionY;
   private DoubleYoVariable interpolatedPelvisErrorPositionZ;
   private DoubleYoVariable interpolatedPelvisErrorPitch;
   private DoubleYoVariable interpolatedPelvisErrorRoll;
   private DoubleYoVariable interpolatedPelvisErrorYaw;

   private DoubleYoVariable previousInterpolatedPelvisErrorPositionX;
   private DoubleYoVariable previousInterpolatedPelvisErrorPositionY;
   private DoubleYoVariable previousInterpolatedPelvisErrorPositionZ;
   private DoubleYoVariable previousInterpolatedPelvisErrorPitch;
   private DoubleYoVariable previousInterpolatedPelvisErrorRoll;
   private DoubleYoVariable previousInterpolatedPelvisErrorYaw;

   private DoubleYoVariable seNonProcessedPelvisPositionX;
   private DoubleYoVariable seNonProcessedPelvisPositionY;
   private DoubleYoVariable seNonProcessedPelvisPositionZ;
   private DoubleYoVariable seNonProcessedPelvisPitch;
   private DoubleYoVariable seNonProcessedPelvisRoll;
   private DoubleYoVariable seNonProcessedPelvisYaw;
   private LongYoVariable seNonProcessedPelvisTimeStamp;

   private DoubleYoVariable correctedPelvisPositionX;
   private DoubleYoVariable correctedPelvisPositionY;
   private DoubleYoVariable correctedPelvisPositionZ;
   private DoubleYoVariable correctedPelvisPitch;
   private DoubleYoVariable correctedPelvisRoll;
   private DoubleYoVariable correctedPelvisYaw;

   private DoubleYoVariable pelvisX;
   private DoubleYoVariable pelvisY;
   private DoubleYoVariable pelvisZ;
   private DoubleYoVariable pelvisPitch;
   private DoubleYoVariable pelvisRoll;
   private DoubleYoVariable pelvisYaw;

   private DoubleYoVariable maxVelocityClip;
   private DoubleYoVariable clippedAlphaValue;

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

      externalPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_positionX");
      externalPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_positionY");
      externalPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_positionZ");
      externalPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_yaw");
      externalPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_pitch");
      externalPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "newExternalPelvis_roll");

      totalErrorPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_positionX");
      totalErrorPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_positionY");
      totalErrorPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_positionZ");
      totalErrorPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_yaw");
      totalErrorPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_pitch");
      totalErrorPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "totalErrorPelvis_roll");

      interpolatedPelvisErrorPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_positionX");
      interpolatedPelvisErrorPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_positionY");
      interpolatedPelvisErrorPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_positionZ");
      interpolatedPelvisErrorYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_yaw");
      interpolatedPelvisErrorPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_pitch");
      interpolatedPelvisErrorRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "interpolatedPelvisError_roll");

      previousInterpolatedPelvisErrorPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_positionX");
      previousInterpolatedPelvisErrorPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_positionY");
      previousInterpolatedPelvisErrorPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_positionZ");
      previousInterpolatedPelvisErrorYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_yaw");
      previousInterpolatedPelvisErrorPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_pitch");
      previousInterpolatedPelvisErrorRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "previousInterpolatedPelvisError_roll");

      seNonProcessedPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_positionX");
      seNonProcessedPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_positionY");
      seNonProcessedPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_positionZ");
      seNonProcessedPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_yaw");
      seNonProcessedPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_pitch");
      seNonProcessedPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_roll");

      seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_timestamp");

      correctedPelvisPositionX = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_positionX");
      correctedPelvisPositionY = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_positionY");
      correctedPelvisPositionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_positionZ");
      correctedPelvisYaw = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_yaw");
      correctedPelvisPitch = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_pitch");
      correctedPelvisRoll = (DoubleYoVariable) registry.getVariable(nameSpace, "correctedPelvis_roll");

      pelvisX = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisX");
      pelvisY = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisY");
      pelvisZ = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisZ");
      pelvisPitch = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisPitch");
      pelvisRoll = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisRoll");
      pelvisYaw = (DoubleYoVariable) registry.getVariable("CommonHumanoidReferenceFramesVisualizer", "pelvisYaw");

      seNonProcessedPelvisQuaternionX = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQx");
      seNonProcessedPelvisQuaternionY = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQy");
      seNonProcessedPelvisQuaternionZ = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQz");
      seNonProcessedPelvisQuaternionS = (DoubleYoVariable) registry.getVariable(nameSpace, "seNonProcessedPelvis_quaternionQs");
      maxVelocityClip = (DoubleYoVariable) registry.getVariable(nameSpace, "maxVelocityClip");
      clippedAlphaValue = (DoubleYoVariable) registry.getVariable(nameSpace, "clippedAlphaValue");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      if (drcNetworkingSimulationTestHelper != null)
      {
         //         drcNetworkingSimulationTestHelper.disconnect();
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
      assertTrue(testInterpolationToRandomTargetsWithFastAlphaValue(robot, registry, 30));

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      sendPelvisCorrectionPackets = false;
      
      assertTrue(success);
      BambooTools.reportTestFinishedMessage();
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

   private void setupCameraForWalkingUpToRamp()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private boolean testNewExternalPelvisTransformAfterUpdate(SDFRobot robot, YoVariableRegistry registry, SimulationConstructionSet simulationConstructionSet,
         ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, Double.POSITIVE_INFINITY);
      boolean success = true;
      FloatingJoint pelvis = robot.getPelvisJoint();
      RigidBodyTransform groundTruthPelvisTransform = new RigidBodyTransform();
      Vector3d translation = new Vector3d();
      Matrix3d rotation = new Matrix3d();

      final Boolean[] pelvisPoseHistoryChanged = new Boolean[1];
      pelvisPoseHistoryChanged[0] = false;
      externalPelvisPositionX.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            pelvisPoseHistoryChanged[0] = true;
         }
      });

      int numberOfTicks = 0;
      int tickThresholdForNewPacket = 4;
      long timeStamp = 0;
      int numberOfIterations = 100;
      boolean isFirstRun = true;
      for (int i = 0; i < numberOfIterations; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         numberOfTicks++;

         if (pelvisPoseHistoryChanged[0])
         {
            pelvisPoseHistoryChanged[0] = false;
            if (!isFirstRun)
            {
               double epsilon = 0.00000001;
               success &= MathTools.epsilonEquals(translation.getX(), externalPelvisPositionX.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getY(), externalPelvisPositionY.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getZ(), externalPelvisPositionZ.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getPitch(rotation), externalPelvisPitch.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getRoll(rotation), externalPelvisRoll.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getYaw(rotation), externalPelvisYaw.getDoubleValue(), epsilon);
            }
            else
            {
               isFirstRun = false;
            }
         }

         if (numberOfTicks < 4)
         {
            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            groundTruthPelvisTransform.get(translation);
            groundTruthPelvisTransform.get(rotation);
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }

         if ((numberOfTicks % tickThresholdForNewPacket) == 0 && numberOfTicks > 4)
         {
            groundTruthPelvisTransform.get(translation);
            groundTruthPelvisTransform.get(rotation);
            TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(groundTruthPelvisTransform, timeStamp);
            StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
            externalPelvisPoseCreator.setNewestPose(posePacket);

            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }
      }
      return success;
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
         targets[i] = RandomTools.generateRandomTransform(random);
         targets[i].setEuler(0, 0, random.nextDouble() * 2 * Math.PI);
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
      RigidBodyTransform[] targets = createRandomCorrectionTargets(numTargets);
      boolean success = true;

      Vector4d error = new Vector4d();
      Vector3d targetTranslation = new Vector3d();
      Matrix3d targetRotation = new Matrix3d();
      double targetYaw = 0;
      double translationFudgeFactor = 0.01;
      double rotationFudgeFactor = 0.035;
      double largestError = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < targets.length; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT() * 10);
         targets[i].get(targetTranslation);
         targets[i].get(targetRotation);
         targetYaw = RotationFunctions.getYaw(targetRotation);
         error.set(targetTranslation.getX(), targetTranslation.getY(), targetTranslation.getZ(), targetYaw);

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
         
         double xError = Math.abs(pelvisX.getDoubleValue() - error.getX());
         double yError = Math.abs(pelvisY.getDoubleValue() - error.getY());
         double zError = Math.abs(pelvisZ.getDoubleValue() - error.getZ());
         double yawError = Math.abs(pelvisYaw.getDoubleValue() - error.getW());

         if (xError > largestError)
            largestError = xError;
         if (yError > largestError)
            largestError = yError;
         if (zError > largestError)
            largestError = zError;
         if (yawError > largestError)
            largestError = yawError;

         success &= xError <= translationFudgeFactor;
         success &= yError <= translationFudgeFactor;
         success &= zError <= translationFudgeFactor;
         success &= yawError <= rotationFudgeFactor;

         error.set(xError, yError, zError, yawError);
      }
      System.out.println(" max fudge factor: " + largestError);
      return success;
   }

   private boolean testTotalErrorWithNoPitchOrRollCorrection(SDFRobot robot, YoVariableRegistry registry, SimulationConstructionSet simulationConstructionSet,
         ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry, Double.POSITIVE_INFINITY);
      TimeStampedPelvisPoseBuffer pelvisPoseBuffer = new TimeStampedPelvisPoseBuffer(500);
      RigidBodyTransform seNonProcessedPelvisPose = new RigidBodyTransform();
      boolean success = true;
      FloatingJoint pelvis = robot.getPelvisJoint();
      RigidBodyTransform groundTruthPelvisTransform = new RigidBodyTransform();
      RigidBodyTransform totalError = new RigidBodyTransform();
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      Vector3d translation = new Vector3d();
      Matrix3d rotation = new Matrix3d();

      Vector3d seNonProcessedTranslation = new Vector3d();
      Quat4d seNonProcessedQuat4d = new Quat4d();

      setupYoVariables(registry, "PelvisPoseHistoryCorrection");
      final Boolean[] pelvisPoseHistoryChanged = new Boolean[1];
      pelvisPoseHistoryChanged[0] = false;
      externalPelvisPositionX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            pelvisPoseHistoryChanged[0] = true;
         }
      });
      boolean isFirstRun = true;
      int numberOfTicks = 0;
      int tickThresholdForNewPacket = 2;
      long timeStamp = 0;
      int numberOfIterations = 100;
      long lastNonProcessedSePoseTimestamp = 0;
      for (int i = 0; i < numberOfIterations; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         numberOfTicks++;

         seNonProcessedQuat4d.set(seNonProcessedPelvisQuaternionX.getDoubleValue(), seNonProcessedPelvisQuaternionY.getDoubleValue(),
               seNonProcessedPelvisQuaternionZ.getDoubleValue(), seNonProcessedPelvisQuaternionS.getDoubleValue());
         seNonProcessedPelvisPose.setRotation(seNonProcessedQuat4d);

         seNonProcessedTranslation.setX(seNonProcessedPelvisPositionX.getDoubleValue());
         seNonProcessedTranslation.setY(seNonProcessedPelvisPositionY.getDoubleValue());
         seNonProcessedTranslation.setZ(seNonProcessedPelvisPositionZ.getDoubleValue());
         seNonProcessedPelvisPose.setTranslation(seNonProcessedTranslation);
         lastNonProcessedSePoseTimestamp = seNonProcessedPelvisTimeStamp.getLongValue();
         pelvisPoseBuffer.put(seNonProcessedPelvisPose, lastNonProcessedSePoseTimestamp);

         if (pelvisPoseHistoryChanged[0])
         {
            pelvisPoseHistoryChanged[0] = false;
            if (!isFirstRun)
            {
               double epsilon = 0.000001;
               success &= MathTools.epsilonEquals(translation.getX(), totalErrorPelvisPositionX.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getY(), totalErrorPelvisPositionY.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getZ(), totalErrorPelvisPositionZ.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(0, totalErrorPelvisPitch.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(0, totalErrorPelvisRoll.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getYaw(rotation), totalErrorPelvisYaw.getDoubleValue(), epsilon);
            }
            else
            {
               isFirstRun = false;
            }
         }

         if (numberOfTicks < 4)
         {
            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }

         if ((numberOfTicks % tickThresholdForNewPacket) == 0 && numberOfTicks > 4)
         {
            TimeStampedTransform3D nonProcessedSePose = pelvisPoseBuffer.interpolate(timeStamp);
            totalError.set(groundTruthPelvisTransform);
            tempTransform.set(nonProcessedSePose.getTransform3D());
            tempTransform.invert();
            totalError.multiply(tempTransform);
            totalError.get(translation);
            totalError.get(rotation);

            TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(groundTruthPelvisTransform, timeStamp);
            StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
            externalPelvisPoseCreator.setNewestPose(posePacket);

            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }
      }
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
   
   private void setupSim()
   {
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGroundEnvironment, new ScriptedFootstepDataListObjectCommunicator("Team"),
            "PelvisCorrectionTest", simpleFlatGroundScriptName, DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, false,
            getRobotModel());
      simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      robot = drcSimulationTestHelper.getRobot();

      externalPelvisPosePublisher = new ExternalPelvisPoseCreator();
      DRCSimulationFactory drcSimulationFactory = drcSimulationTestHelper.getDRCSimulationFactory();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPosePublisher);

      setupCameraForWalkingUpToRamp();
   }

   private Runnable setupSimulationWithFeetPertuberAndCreateExternalPelvisThread()
   {
      setupSim();
      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeetPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      registry = robot.getRobotsYoVariableRegistry();
      Runnable pelvisCorrectorSource = createPelvisCorrectorProducerUsingSCSActual(robot,
            externalPelvisPosePublisher);
      return pelvisCorrectorSource;
   }

   private void setupSimulationWithStandingControllerAndCreateExternalPelvisThread()
   {
      setupSim();
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
