package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.subscribers.ExternalPelvisPoseSubscriberInterface;
import us.ihmc.communication.subscribers.ExternalTimeStampedPoseSubscriber;
import us.ihmc.communication.subscribers.TimeStampedPelvisPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.util.OscillateFeetPerturber;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.BoundingBox3d;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.net.KryoLocalObjectCommunicator;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public abstract class PelvisPoseHistoryCorrectionTest implements MultiRobotTestInterface
{

   private static final boolean KEEP_SCS_UP = false;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;
   
   private Random random = new Random();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCSimulationNetworkTestHelper drcNetworkingSimulationTestHelper;
   DRCDemo01NavigationEnvironment demo01NavEnvironmant;
   
   private final String simpleFlatGroundScriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
   private final String fileName = BambooTools.getFullFilenameUsingClassRelativeURL(DRCObstacleCourseFlatTest.class, simpleFlatGroundScriptName);

   private boolean sendPelvisCorrectionPackets = true;

   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @Before
   public void setUp()
   {
      demo01NavEnvironmant = new DRCDemo01NavigationEnvironment();
      showMemoryUsageBeforeTest();
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
   public void testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeetUsingNetworkPackets() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      drcNetworkingSimulationTestHelper = new DRCSimulationNetworkTestHelper(getRobotModel(),demo01NavEnvironmant,fileName,showGUI,false);
      drcSimulationTestHelper = drcNetworkingSimulationTestHelper.getDRCSimulationTestHelper();
      
      ExternalPelvisPoseSubscriberInterface externalPelvisPoseSubscriber = new ExternalTimeStampedPoseSubscriber();
      DRCSimulationFactory drcSimulationFactory = drcSimulationTestHelper.getDRCSimulationFactory();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
      
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      SDFRobot robot = drcSimulationTestHelper.getRobot();
      
      setupCameraForWalkingUpToRamp();
      
      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeerPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      
      Runnable pelvisCorrectorSource = createPelvisCorrectorPacketGenerator(simulationConstructionSet, robot);
      Thread t = new Thread(pelvisCorrectorSource);
      t.start();
      
      ThreadTools.sleep(1000);
      
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry,0.015);
      activatePelvisPoseHistoryCorrector(registry,true);
      
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);
      
      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      
      sendPelvisCorrectionPackets = false;
      assertTrue(success);
      
      Point3d center = new Point3d(0.6965598483152976, 0.14814849146917242, 0.8455816468141424);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      
      BambooTools.reportTestFinishedMessage();
   }
   @Test
   public void testPelvisCorrectionDuringSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      drcSimulationTestHelper = new DRCSimulationTestHelper(demo01NavEnvironmant.getTerrainObject3D(), new KryoLocalObjectCommunicator(new IHMCCommunicationKryoNetClassList()),
            simpleFlatGroundScriptName, fileName, DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie,
            false, getRobotModel());
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      SDFRobot robot = drcSimulationTestHelper.getRobot();

      ExternalPelvisPoseCreator externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      DRCSimulationFactory drcSimulationFactory = drcSimulationTestHelper.getDRCSimulationFactory();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPoseCreator);
      
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeerPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      
      Runnable pelvisCorrectorSource = createPelvisCorrectorProducer(simulationConstructionSet, robot, externalPelvisPoseCreator);
      Thread t = new Thread(pelvisCorrectorSource);
      t.start();
      
      ThreadTools.sleep(1000);
      
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry,0.015);
      activatePelvisPoseHistoryCorrector(registry,true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      
      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testPelvisCorrectionWithOscillatingFeetWhileStanding() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(demo01NavEnvironmant.getTerrainObject3D(), new KryoLocalObjectCommunicator(
            new IHMCCommunicationKryoNetClassList()),
            simpleFlatGroundScriptName, fileName, DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie,
            false, getRobotModel());
      
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      SDFRobot robot = drcSimulationTestHelper.getRobot();
      
      ExternalPelvisPoseCreator externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      DRCSimulationFactory drcSimulationFactory = drcSimulationTestHelper.getDRCSimulationFactory();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPoseCreator);
      
      setupCameraForWalkingUpToRamp();
      
      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeerPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      
      Runnable pelvisCorrectorSource = createPelvisCorrectorProducer(simulationConstructionSet, robot, externalPelvisPoseCreator);
      Thread t = new Thread(pelvisCorrectorSource);
      t.start();
      
      ThreadTools.sleep(1000);
      
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry,0.015);
      activatePelvisPoseHistoryCorrector(registry,true);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      
      sendPelvisCorrectionPackets = false;
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }
   
   @Test
   public void testInstantaneousPelvisCorrectionWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(demo01NavEnvironmant.getTerrainObject3D(), new KryoLocalObjectCommunicator(
            new IHMCCommunicationKryoNetClassList()),
            simpleFlatGroundScriptName, fileName, DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie,
            false, getRobotModel());
      
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      SDFRobot robot = drcSimulationTestHelper.getRobot();
      
      ExternalPelvisPoseCreator externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      DRCSimulationFactory drcSimulationFactory = drcSimulationTestHelper.getDRCSimulationFactory();
      drcSimulationFactory.setExternelPelvisCorrectorSubscriber(externalPelvisPoseCreator);
      
      setupCameraForWalkingUpToRamp();
      
      int ticksPerPerturbation = 1;
      OscillateFeetPerturber oscillateFeetPerturber = generateFeerPertuber(simulationConstructionSet, robot, ticksPerPerturbation);
      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
      
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      
      setPelvisPoseHistoryCorrectorAlphaBreakFreq(registry,Double.POSITIVE_INFINITY);
      activatePelvisPoseHistoryCorrector(registry,true);
      
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      
      assertTrue(testNewExternalPelvisTransformAfterUpdate(robot, registry, simulationConstructionSet, externalPelvisPoseCreator));
      assertTrue(testTotalErrorWithNoPitchOrRollCorrection(robot, registry, simulationConstructionSet, externalPelvisPoseCreator));
      
//      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      
      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   private void setupCameraForWalkingUpToRamp()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   
   private boolean testNewExternalPelvisTransformAfterUpdate(SDFRobot robot, YoVariableRegistry registry, SimulationConstructionSet simulationConstructionSet, ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
      boolean success = true;
      FloatingJoint pelvis = robot.getPelvisJoint();
      RigidBodyTransform groundTruthPelvisTransform = new RigidBodyTransform();
      Vector3d translation = new Vector3d();
      Matrix3d rotation = new Matrix3d();
      
      DoubleYoVariable externalPelvisPositionX = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_positionX");
      DoubleYoVariable externalPelvisPositionY = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_positionY");
      DoubleYoVariable externalPelvisPositionZ = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_positionZ");
      DoubleYoVariable externalPelvisPitch = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_pitch");
      DoubleYoVariable externalPelvisRoll = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_roll");
      DoubleYoVariable externalPelvisYaw = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_yaw");
      
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
      for(int i = 0 ; i < numberOfIterations ; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         numberOfTicks++;
         
         if(pelvisPoseHistoryChanged[0])
         {
            pelvisPoseHistoryChanged[0] = false;
            if(!isFirstRun)
            {
               double epsilon = 0.00000001;
               success &= MathTools.epsilonEquals(translation.getX(), externalPelvisPositionX.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getY(), externalPelvisPositionY.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getZ(), externalPelvisPositionZ.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getPitch(rotation), externalPelvisPitch.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getRoll(rotation), externalPelvisRoll.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getYaw(rotation), externalPelvisYaw.getDoubleValue(), epsilon);
            } else {
               isFirstRun = false;
            }
         }
         
         if(numberOfTicks < 4)
         {
            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            groundTruthPelvisTransform.get(translation);
            groundTruthPelvisTransform.get(rotation);
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }
         
         if((numberOfTicks % tickThresholdForNewPacket) == 0 && numberOfTicks > 4)
         {
            groundTruthPelvisTransform.get(translation);
            groundTruthPelvisTransform.get(rotation);
            TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(groundTruthPelvisTransform,timeStamp);
            StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
            externalPelvisPoseCreator.setNewestPose(posePacket);

            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }
      }
      return success;
   }
   
   private boolean testTotalErrorWithNoPitchOrRollCorrection(SDFRobot robot, YoVariableRegistry registry, SimulationConstructionSet simulationConstructionSet, ExternalPelvisPoseCreator externalPelvisPoseCreator) throws SimulationExceededMaximumTimeException
   {
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
      
      DoubleYoVariable externalPelvisPositionX = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "newExternalPelvis_positionX");
      
      DoubleYoVariable totalErrorPelvisPositionX = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "totalErrorPelvis_positionX");
      DoubleYoVariable totalErrorPelvisPositionY = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "totalErrorPelvis_positionY");
      DoubleYoVariable totalErrorPelvisPositionZ = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "totalErrorPelvis_positionZ");
      DoubleYoVariable totalErrorPelvisPitch = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "totalErrorPelvis_pitch");
      DoubleYoVariable totalErrorPelvisRoll = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "totalErrorPelvis_roll");
      DoubleYoVariable totalErrorPelvisYaw = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "totalErrorPelvis_yaw");
      
      DoubleYoVariable seNonProcessedPelvisPositionX = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_positionX");
      DoubleYoVariable seNonProcessedPelvisPositionY = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_positionY");
      DoubleYoVariable seNonProcessedPelvisPositionZ = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_positionZ");
      DoubleYoVariable seNonProcessedPelvisQuaternionX = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_quaternionQx");
      DoubleYoVariable seNonProcessedPelvisQuaternionY = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_quaternionQy");
      DoubleYoVariable seNonProcessedPelvisQuaternionZ = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_quaternionQz");
      DoubleYoVariable seNonProcessedPelvisQuaternionS = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_quaternionQs");
      LongYoVariable seNonProcessedPelvisTimeStamp = (LongYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "seNonProcessedPelvis_timestamp");
      
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
      for(int i = 0 ; i < numberOfIterations ; i++)
      {
         success &= drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getEstimatorDT());
         numberOfTicks++;
         
         seNonProcessedQuat4d.set(seNonProcessedPelvisQuaternionX.getDoubleValue(), seNonProcessedPelvisQuaternionY.getDoubleValue(),
               seNonProcessedPelvisQuaternionZ.getDoubleValue(), seNonProcessedPelvisQuaternionS.getDoubleValue());
         seNonProcessedPelvisPose.setRotationAndZeroTranslation(seNonProcessedQuat4d);
         
         seNonProcessedTranslation.setX(seNonProcessedPelvisPositionX.getDoubleValue());
         seNonProcessedTranslation.setY(seNonProcessedPelvisPositionY.getDoubleValue());
         seNonProcessedTranslation.setZ(seNonProcessedPelvisPositionZ.getDoubleValue());
         seNonProcessedPelvisPose.setTranslation(seNonProcessedTranslation);
         lastNonProcessedSePoseTimestamp = seNonProcessedPelvisTimeStamp.getLongValue();
         pelvisPoseBuffer.put(seNonProcessedPelvisPose, lastNonProcessedSePoseTimestamp);
         
         if(pelvisPoseHistoryChanged[0])
         {
            pelvisPoseHistoryChanged[0] = false;
            if(!isFirstRun)
            {
               double epsilon = 0.000001;
               success &= MathTools.epsilonEquals(translation.getX(), totalErrorPelvisPositionX.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getY(), totalErrorPelvisPositionY.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(translation.getZ(), totalErrorPelvisPositionZ.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(0, totalErrorPelvisPitch.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(0, totalErrorPelvisRoll.getDoubleValue(), epsilon);
               success &= MathTools.epsilonEquals(RotationFunctions.getYaw(rotation), totalErrorPelvisYaw.getDoubleValue(), epsilon);
            } else {
               isFirstRun = false;
            }
         }
         
         if(numberOfTicks < 4)
         {
            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }
         
         if((numberOfTicks % tickThresholdForNewPacket) == 0 && numberOfTicks > 4)
         {
            TimeStampedTransform3D nonProcessedSePose = pelvisPoseBuffer.interpolate(timeStamp);
            totalError.set(groundTruthPelvisTransform);
            tempTransform.set(nonProcessedSePose.getTransform3D());
            tempTransform.invert();
            totalError.multiply(tempTransform);
            totalError.get(translation);
            totalError.get(rotation);
            
            TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(groundTruthPelvisTransform,timeStamp);
            StampedPosePacket posePacket = new StampedPosePacket("/pelvis", timeStampedTransform, 1.0);
            externalPelvisPoseCreator.setNewestPose(posePacket);
            
            groundTruthPelvisTransform.set(pelvis.getJointTransform3D());
            timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
         }
      }
      return success;
   }
   
   private Runnable createPelvisCorrectorPacketGenerator(final SimulationConstructionSet simulationConstructionSet, final SDFRobot robot)
   {
      Runnable pelvisCorrectorSource = new Runnable()
      {
         
         FloatingJoint pelvis = robot.getPelvisJoint();
         RigidBodyTransform pelvisTransform = new RigidBodyTransform();

         @Override
         public void run()
         {
            while(running())
            {
               try
               {
                  Thread.sleep(900);
                  pelvisTransform.set(pelvis.getJointTransform3D());
                  long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
                  Thread.sleep((int) (random.nextDouble() * 200));
                  TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(pelvisTransform,timeStamp);
                  StampedPosePacket posePacket = new StampedPosePacket("/pelvis",timeStampedTransform,1.0);
                  drcNetworkingSimulationTestHelper.sendObjectToKryo(posePacket);
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
   
   private Runnable createPelvisCorrectorProducer(final SimulationConstructionSet simulationConstructionSet, final SDFRobot robot, final ExternalPelvisPoseCreator externalPelvisPoseCreator)
   {
      Runnable pelvisCorrectorSource = new Runnable()
      {
         
         FloatingJoint pelvis = robot.getPelvisJoint();
         RigidBodyTransform pelvisTransform = new RigidBodyTransform();
         
         @Override
         public void run()
         {
            while(running())
            {
               try
               {
                  Thread.sleep(900);
                  pelvisTransform.set(pelvis.getJointTransform3D());
                  long timeStamp = TimeTools.secondsToNanoSeconds(simulationConstructionSet.getTime());
                  Thread.sleep((int) (random.nextDouble() * 200));
                  TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D(pelvisTransform,timeStamp);
                  StampedPosePacket posePacket = new StampedPosePacket("/pelvis",timeStampedTransform,1.0);
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
   
   private OscillateFeetPerturber generateFeerPertuber(final SimulationConstructionSet simulationConstructionSet, SDFRobot robot, int ticksPerPerturbation)
   {
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[]{0.008, 0.012, 0.005});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[]{0.010, 0.06, 0.010});
      
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[]{1.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[]{2.0, 0.5, 1.3});
      
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[]{5.0, 0.5, 0.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[]{0.2, 3.4, 1.11});
      return oscillateFeetPerturber;
   }
   
   private boolean running()
   {
      return sendPelvisCorrectionPackets;
   }
   
   private void setPelvisPoseHistoryCorrectorAlphaBreakFreq(YoVariableRegistry registry, double breakFrequency)
   {
      DoubleYoVariable pelvisCorrectorAlphaFilterBF = (DoubleYoVariable) registry.getVariable("PelvisPoseHistoryCorrection", "interpolationAlphaFilterBreakFrequency");
      pelvisCorrectorAlphaFilterBF.set(breakFrequency);
   }
   
   private void activatePelvisPoseHistoryCorrector(YoVariableRegistry registry, boolean activate)
   {
      BooleanYoVariable useExternalPelvisCorrector = (BooleanYoVariable) registry.getVariable("DRCKinematicsBasedStateEstimator", "useExternalPelvisCorrector");
      useExternalPelvisCorrector.set(activate);
   }
   
   private class ExternalPelvisPoseCreator implements ExternalPelvisPoseSubscriberInterface
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
   }
}
