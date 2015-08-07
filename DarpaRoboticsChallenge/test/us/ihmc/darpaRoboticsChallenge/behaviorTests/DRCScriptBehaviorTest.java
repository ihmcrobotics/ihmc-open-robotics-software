package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.fest.util.Files;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.networking.CommandRecorder;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngineSettings;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.pathGeneration.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footsepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCScriptBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCScriptBehaviorTest.class + " after class.");
   }

   private final double POSITION_THRESHOLD = 0.007;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private static final boolean DEBUG = true;

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame recordFrame;
   String fileName = "1_ScriptBehaviorTest";
   TimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private static File file;

   private double nominalComHeightAboveGround;

   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private final LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      file = new File(ScriptEngineSettings.scriptSavingDirectory + fileName + ScriptEngineSettings.extension);
      System.out.println(file.getAbsolutePath());

      armJointNames = drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }
   }

   @After
   public void destroyFilesAfterTests()
   {
      Files.delete(file);
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testSimpleScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d nominalComPosition = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double trajectoryTime = 2.0;
      double desiredFinalHeightOffset = createValidComHeightOffset(0.1);
      ComHeightPacket comHeightPacket = new ComHeightPacket(desiredFinalHeightOffset, trajectoryTime);

      recordScriptFile(comHeightPacket, fileName);
      
      PrintTools.debug(this, "Initializing Behavior and Executing It");
      ScriptBehavior scriptBehavior = setupNewScriptBehaviorAndTestIt(file, trajectoryTime);
      PrintTools.debug(this, "Behavior Should Be Done");

      assertTrue(scriptBehavior.isDone());
      assertProperHeightOffset(desiredFinalHeightOffset);

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithTwoComHeightScriptPackets() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d nominalComPosition = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double trajectoryTime = 3.0; //FIXME: Test fails if trajectory time is too short
      double desiredIntermediateHeightOffset = ComHeightPacket.MAX_COM_HEIGHT;
      double desiredFinalHeightOffset = ComHeightPacket.MIN_COM_HEIGHT;

      ComHeightPacket comHeightPacket0 = new ComHeightPacket(desiredIntermediateHeightOffset, trajectoryTime);
      ComHeightPacket comHeightPacket1 = new ComHeightPacket(desiredFinalHeightOffset, trajectoryTime);

      ArrayList<Packet<?>> scriptPackets = new ArrayList<Packet<?>>();
      scriptPackets.add(comHeightPacket0);
      scriptPackets.add(comHeightPacket1);
      recordScriptFile(scriptPackets, fileName);
      
      PrintTools.debug(this, "Initializing Behavior");
      ScriptBehavior scriptBehavior = setupNewScriptBehavior(file);

      PrintTools.debug(this, "Starting Behavior");
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime);
      assertTrue(success);
      
      PrintTools.debug(this, "Checking that first script packet executed properly");
      assertProperHeightOffset(desiredIntermediateHeightOffset);
      assertTrue(!scriptBehavior.isDone());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should Be Done");

      PrintTools.debug(this, "Checking that second script packet executed properly");
      assertProperHeightOffset(desiredFinalHeightOffset);
      assertTrue(scriptBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testStopScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d nominalComPosition = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double trajectoryTime = 2.0;
      double desiredHeightOffset = -0.10;

      ComHeightPacket comHeightPacket = new ComHeightPacket(desiredHeightOffset, trajectoryTime);
      RobotSide handPoseSide = RobotSide.LEFT;
      double[] desiredArmPose = createArmPoseAtJointLimits(handPoseSide);
      HandPosePacket handPosePacket = new HandPosePacket(handPoseSide, trajectoryTime, desiredArmPose);

      ArrayList<Packet<?>> scriptPackets = new ArrayList<Packet<?>>();
      scriptPackets.add(comHeightPacket);
      scriptPackets.add(handPosePacket);
      recordScriptFile(scriptPackets, fileName);
      
      PrintTools.debug(this, "Initializing Behavior");
      ScriptBehavior scriptBehavior = setupNewScriptBehavior(file);

      PrintTools.debug(this, "Starting Behavior");
      double[] initialHandPose = getCurrentArmPose(handPosePacket.getRobotSide());
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, 0.9 * trajectoryTime);
      assertTrue(success);

      PrintTools.debug(this, "Stopping Behavior");
      scriptBehavior.stop();
      assertProperHeightOffset(desiredHeightOffset);

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);

      assertHandPosePacketDidNotExecute(handPoseSide, initialHandPose, handPosePacket);
      assertProperHeightOffset(desiredHeightOffset);
      assertTrue(!scriptBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPauseAndResumeScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Point3d nominalComPosition = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double trajectoryTime = 2.0;
      double desiredIntermediateHeightOffset = -0.1;

      ComHeightPacket comHeightPacket0 = new ComHeightPacket(desiredIntermediateHeightOffset, trajectoryTime);
      RobotSide handPoseSide = RobotSide.LEFT;
      double[] desiredArmPose = createRandomArmPose(handPoseSide);
      HandPosePacket handPosePacket = new HandPosePacket(handPoseSide, trajectoryTime, desiredArmPose);

      ArrayList<Packet<?>> scriptPackets = new ArrayList<Packet<?>>();
      scriptPackets.add(comHeightPacket0);
      scriptPackets.add(handPosePacket);
      recordScriptFile(scriptPackets, fileName);
      
      PrintTools.debug(this, "Initializing Behavior");
      ScriptBehavior scriptBehavior = setupNewScriptBehavior(file);

      PrintTools.debug(this, "Starting Behavior");
      double[] initialArmPose = getCurrentArmPose(handPoseSide);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, 0.99 * trajectoryTime);
      assertTrue(success);

      PrintTools.debug(this, "Pausing Behavior");
      scriptBehavior.pause();
      PrintTools.debug(this, "Waiting for Robot To Settle");
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      assertProperHeightOffset(desiredIntermediateHeightOffset);

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime);
      assertTrue(success);
      assertProperHeightOffset(desiredIntermediateHeightOffset);
      assertHandPosePacketDidNotExecute(handPoseSide, initialArmPose, handPosePacket);
      assertTrue(!scriptBehavior.isDone());

      PrintTools.debug(this, "Resuming Behavior");
      scriptBehavior.resume();
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should Be Done");

      
      double[] finalArmPose = getCurrentArmPose(handPoseSide);
      assertArmPosesAreWithinThresholds(desiredArmPose, finalArmPose, handPoseSide);
      assertTrue(scriptBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithOneHandPosePacket() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;

      double[] desiredArmPose = createRandomArmPose(robotSide);
      HandPosePacket handPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredArmPose);

      ArrayList<Packet<?>> scriptPackets = new ArrayList<Packet<?>>();
      scriptPackets.add(handPosePacket);

      recordScriptFile(scriptPackets, fileName);
      PrintTools.debug(this, "Initializing and Starting Behavior");
      ScriptBehavior scriptBehavior = setupNewScriptBehaviorAndTestIt(file, trajectoryTime);
      PrintTools.debug(this, "Behavior Should Be Done");

      assertTrue(scriptBehavior.isDone());

      double[] finalArmPose = getCurrentArmPose(robotSide);
      assertArmPosesAreWithinThresholds(desiredArmPose, finalArmPose, robotSide);

      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithTwoHandPosePackets() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime1 = 2.0;
      double settlingTime = 1.0;
      double trajectoryTime2 = 2.0;

      double[] desiredArmPose1 = createRandomArmPose(robotSide);
      double[] desiredArmPose2 = getCurrentArmPose(robotSide);

      HandPosePacket handPosePacket1 = new HandPosePacket(robotSide, trajectoryTime1, desiredArmPose1);
      PelvisPosePacket trivialPacketToAllowRobotToSettle = new PelvisPosePacket(null, null, true, settlingTime);
      HandPosePacket handPosePacket2 = new HandPosePacket(robotSide, trajectoryTime2, desiredArmPose2);

      ArrayList<Packet<?>> scriptPackets = new ArrayList<Packet<?>>();
      scriptPackets.add(handPosePacket1);
      scriptPackets.add(trivialPacketToAllowRobotToSettle);
      scriptPackets.add(handPosePacket2);
      recordScriptFile(scriptPackets, fileName);

      PrintTools.debug(this, "Initializing Behavior");
      ScriptBehavior scriptBehavior = setupNewScriptBehavior(file);

      PrintTools.debug(this, "Start Executing Behavior");
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime1);
      assertTrue(success);
      
      PrintTools.debug(this, "Allow robot to settle by sending trivial pelvis pose packet");
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trivialPacketToAllowRobotToSettle.getTrajectoryTime());
      assertTrue(success);
      
      PrintTools.debug(this, "Checking that first script packet executed properly");
      double[] armPoseAfterHandPosePacket1 = getCurrentArmPose(robotSide);
      assertArmPosesAreWithinThresholds(desiredArmPose1, armPoseAfterHandPosePacket1, robotSide);
      assertTrue(!scriptBehavior.isDone());

      PrintTools.debug(this, "Continue Executing Behavior");
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime2 + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should Be Done");

      
      PrintTools.debug(this, "Checking that last script packet executed properly");
      double[] armPoseAfterHandPosePacket2 = getCurrentArmPose(robotSide);
      assertArmPosesAreWithinThresholds(desiredArmPose2, armPoseAfterHandPosePacket2, robotSide);
      assertTrue(scriptBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private double createValidComHeightOffset(double relativeOffsetFromMinHeightBetweenZeroAndOne)
   {
      double alpha = MathTools.clipToMinMax(relativeOffsetFromMinHeightBetweenZeroAndOne, 0.0, 1.0);
      double ret = (1 - alpha) * ComHeightPacket.MIN_COM_HEIGHT + alpha * ComHeightPacket.MAX_COM_HEIGHT;

      return ret;
   }

   private void recordScriptFile(Packet<?> scriptPacket, String fileName)
   {
      ArrayList<Packet<?>> scriptPackets = new ArrayList<Packet<?>>();
      scriptPackets.add(scriptPacket);

      recordScriptFile(scriptPackets, fileName);
   }

   private void recordScriptFile(ArrayList<Packet<?>> scriptPackets, String fileName)
   {
      CommandRecorder commandRecorder = new CommandRecorder(timestampProvider);
      recordFrame = worldFrame;
      boolean overwriteExistingFile = true;
      commandRecorder.startRecording(fileName, worldFrame, overwriteExistingFile);

      for (Packet<?> scriptPacket : scriptPackets)
      {
         commandRecorder.recordObject(scriptPacket);
      }
      commandRecorder.stopRecording();
   }

   private FramePose getCurrentHandPose(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      drcBehaviorTestHelper.updateRobotModel();
      fullRobotModel.updateFrames();

      ReferenceFrame handFrame = fullRobotModel.getHandControlFrame(robotSide);
      FramePose ret = new FramePose();
      ret.setToZero(handFrame);
      ret.changeFrame(worldFrame);

      return ret;
   }

   private HandPosePacket createHandPosePacket(FramePose desiredHandPose, RobotSide robotSide, double trajectoryTime)
   {
      Point3d desiredHandPostion = new Point3d();
      desiredHandPose.getPosition(desiredHandPostion);

      Quat4d desiredHandOrientation = new Quat4d();
      desiredHandPose.getOrientation(desiredHandOrientation);

      HandPosePacket ret = new HandPosePacket(robotSide, Frame.WORLD, desiredHandPostion, desiredHandOrientation, trajectoryTime);

      return ret;
   }

   private ScriptBehavior setupNewScriptBehavior(File filePath) throws FileNotFoundException
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();
      BooleanYoVariable yoDoubleSupport = drcBehaviorTestHelper.getCapturePointUpdatable().getYoDoubleSupport();

      final ScriptBehavior scriptBehavior = new ScriptBehavior(communicationBridge, fullRobotModel, yoTime, yoDoubleSupport, getRobotModel().getWalkingControllerParameters());

      InputStream inputStream = new FileInputStream(new File(filePath.getAbsolutePath()));
      RigidBodyTransform scriptTransformToWorld = recordFrame.getTransformToDesiredFrame(worldFrame);

      scriptBehavior.initialize();
      scriptBehavior.importChildInputPackets(inputStream.toString(), inputStream, scriptTransformToWorld);
      assertTrue(scriptBehavior.hasInputBeenSet());

      return scriptBehavior;
   }

   private ScriptBehavior setupNewScriptBehaviorAndTestIt(File filePath, double trajectoryTime) throws SimulationExceededMaximumTimeException,
         FileNotFoundException
   {
      ScriptBehavior scriptBehavior = setupNewScriptBehavior(filePath);

      boolean success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(scriptBehavior, trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);
      assertTrue(success);

      return scriptBehavior;
   }

   private Quat4d createQuat4d(Vector3d axis, double rotationAngle)
   {
      AxisAngle4d desiredAxisAngle = new AxisAngle4d();
      desiredAxisAngle.set(axis, rotationAngle);

      Quat4d ret = new Quat4d();
      ret.set(desiredAxisAngle);

      return ret;
   }

   private HandPosePacket createHandPosePacket(RobotSide robotside)
   {
      drcBehaviorTestHelper.updateRobotModel();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      ReferenceFrame handFrame = fullRobotModel.getHandControlFrame(robotside);
      FramePose currentHandPose = new FramePose();
      currentHandPose.setToZero(handFrame);
      currentHandPose.changeFrame(worldFrame);

      Point3d desiredHandPosition = new Point3d();
      currentHandPose.getPosition(desiredHandPosition);
      desiredHandPosition.setZ(desiredHandPosition.getZ() + 0.1);
      PrintTools.debug(this, "desired Hand Position : " + desiredHandPosition);

      Quat4d desiredHandOrientation = new Quat4d();
      currentHandPose.getOrientation(desiredHandOrientation);

      HandPosePacket ret = new HandPosePacket(robotside, Frame.WORLD, desiredHandPosition, desiredHandOrientation, 1.0);

      return ret;
   }

   private FootstepDataList createFootStepDataList(Vector2d walkDeltaXY, HumanoidReferenceFrames referenceFrames)
   {
      drcBehaviorTestHelper.updateRobotModel();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      FootstepDataList footsepDataList = new FootstepDataList();

      SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>();
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();

      for (RobotSide robotSide : RobotSide.values)
      {
         feet.put(robotSide, fullRobotModel.getFoot(robotSide));
         soleFrames.put(robotSide, fullRobotModel.getSoleFrame(robotSide));
      }

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();

      SimplePathParameters pathType = new SimplePathParameters(walkingControllerParameters.getMaxStepLength(), walkingControllerParameters.getInPlaceWidth(),
            0.0, Math.toRadians(20.0), Math.toRadians(10.0), 0.4);

      TurnStraightTurnFootstepGenerator footstepGenerator;

      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

      footsteps.clear();
      FramePose2d endPose = new FramePose2d(worldFrame);

      referenceFrames.updateFrames();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      FramePose currentPose = new FramePose();
      currentPose.setToZero(midFeetFrame);
      currentPose.changeFrame(worldFrame);

      Point2d targetLocation = new Point2d();
      targetLocation.set(currentPose.getX(), currentPose.getY());
      targetLocation.add(walkDeltaXY);

      endPose.setPosition(new FramePoint2d(worldFrame, targetLocation.getX(), targetLocation.getY()));
      endPose.setOrientation(new FrameOrientation2d(worldFrame, currentPose.getYaw()));

      footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose, pathType);
      footstepGenerator.initialize();
      footsteps.addAll(footstepGenerator.generateDesiredFootstepList());

      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         Point3d location = new Point3d(footstep.getX(), footstep.getY(), footstep.getZ());
         Quat4d orientation = new Quat4d();
         footstep.getOrientation(orientation);

         RobotSide footstepSide = footstep.getRobotSide();
         FootstepData footstepData = new FootstepData(footstepSide, location, orientation);
         footsepDataList.add(footstepData);
      }

      return footsepDataList;
   }

   private double getTotalFingerJointQ(RobotSide robotSide)
   {
      drcBehaviorTestHelper.updateRobotModel();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      SDFJointNameMap jointNameMap = (SDFJointNameMap) fullRobotModel.getRobotSpecificJointNames();
      Joint wristJoint = drcBehaviorTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      double totalFingerJointQ = 0.0;

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ().getDoubleValue();
         totalFingerJointQ += q;
         PrintTools.debug(this, fingerJoint.getName() + " q : " + q);
      }

      return totalFingerJointQ;
   }

   private double[] getCurrentArmPose(RobotSide robotSide)
   {
      drcBehaviorTestHelper.updateRobotModel();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = fullRobotModel.getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
   }

   private double[] createArmPoseAtJointLimits(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         double qDesired = clipDesiredJointQToJointLimits(robotSide, armJointNames[jointNum], Double.POSITIVE_INFINITY);
         armPose[jointNum] = qDesired;
      }

      return armPose;
   }

   private double[] createRandomArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         double qDesired = clipDesiredJointQToJointLimits(robotSide, armJointNames[jointNum], RandomTools.generateRandomDouble(new Random(), 1.5));
         armPose[jointNum] = qDesired;
      }

      return armPose;
   }

   private void assertArmPosesAreWithinThresholds(double[] desiredArmPose, double[] actualArmPose, RobotSide robotSide)
   {
      assertArmPosesAreWithinThresholds(desiredArmPose, actualArmPose, robotSide, DRCHandPoseBehaviorTest.JOINT_POSITION_THRESHOLD);
   }

   private void assertArmPoseDidNotChange(double[] initialArmPose, RobotSide robotSide, double jointPositionThreshold)
   {
      assertArmPosesAreWithinThresholds(initialArmPose, getCurrentArmPose(robotSide), robotSide, jointPositionThreshold);
   }

   private void assertHandPosePacketDidNotExecute(RobotSide robotSide, double[] initialArmPose, HandPosePacket handPosePacket)
   {
      double[] currentArmPose = getCurrentArmPose(robotSide);
      double[] desiredArmPose = handPosePacket.getJointAngles();

      double armPoseDistanceFromInitial = computeTotalArmJointError(initialArmPose, currentArmPose);
      double armPoseDistanceToHandPosePacket = computeTotalArmJointError(desiredArmPose, currentArmPose);

      boolean closerToInitalThanToHandPosePacket = armPoseDistanceFromInitial < armPoseDistanceToHandPosePacket;

      assertTrue("HandPose behavior must have executed when it shouldn't have.", closerToInitalThanToHandPosePacket);
   }

   private double computeTotalArmJointError(double[] desiredArmPose, double[] actualArmPose)
   {
      double ret = 0;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];
         ret += Math.abs(q_desired - q_actual);
      }
      return ret;
   }

   private void assertArmPosesAreWithinThresholds(double[] desiredArmPose, double[] actualArmPose, RobotSide robotSide, double jointPositionThreshold)
   {
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         ArmJointName armJointName = armJointNames[i];

         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];
         double error = Math.abs(q_actual - q_desired);

         if (DEBUG)
         {
            PrintTools.debug(this, armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".");
         }
         assertEquals(armJointName + " position error (" + Math.toDegrees(error) + " degrees) exceeds threshold of " + Math.toDegrees(jointPositionThreshold) + " degrees.", q_desired, q_actual, jointPositionThreshold);
      }
   }

   private double clipDesiredJointQToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      drcBehaviorTestHelper.updateRobotModel();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      double q;
      double qMin = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
      double qMax = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

      if (qMin > qMax)
      {
         double temp = qMax;
         qMax = qMin;
         qMin = temp;
      }

      q = MathTools.clipToMinMax(desiredJointAngle, qMin, qMax);
      return q;
   }

   private void assertProperHeightOffset(double desiredHeightOffset)
   {
      Point3d comPoint = new Point3d();
      drcBehaviorTestHelper.getRobot().computeCenterOfMass(comPoint);
      assertProperComHeightOffsetFromGround(desiredHeightOffset, comPoint);
   }

   private void assertProperComHeightOffsetFromGround(double desiredHeightOffset, Point3d finalComPoint)
   {
      double actualHeightOffset = finalComPoint.getZ() - nominalComHeightAboveGround;

      if (DEBUG)
      {
         PrintTools.debug(this, "desiredHeightOffset: " + desiredHeightOffset);
         PrintTools.debug(this, "actualHeightOffset: " + actualHeightOffset);
      }

      assertEquals("Actual CoM Height Offset :" + actualHeightOffset + " does not match desired offset: " + desiredHeightOffset + " within threshold of " + DRCComHeightBehaviorTest.POSITION_THRESHOLD, desiredHeightOffset, actualHeightOffset, DRCComHeightBehaviorTest.POSITION_THRESHOLD);
   }

   private void assertOrientationsAreWithinThresholds(Quat4d desiredQuat, ReferenceFrame frameToCheck)
   {
      drcBehaviorTestHelper.updateRobotModel();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      FramePose framePose = new FramePose();
      framePose.setToZero(frameToCheck);
      framePose.changeFrame(worldFrame);

      Quat4d quatToCheck = new Quat4d();
      framePose.getOrientation(quatToCheck);

      assertOrientationsAreWithinThresholds(desiredQuat, quatToCheck);
   }

   private void assertOrientationsAreWithinThresholds(Quat4d desiredQuat, Quat4d actualQuat)
   {
      FramePose desiredPose = new FramePose(worldFrame, new Point3d(), desiredQuat);
      FramePose actualPose = new FramePose(worldFrame, new Point3d(), actualQuat);

      double orientationDistance = desiredPose.getOrientationDistance(actualPose);

      PrintTools.debug(this, "orientationDistance=" + orientationDistance);

      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      PrintTools.debug(this, "positionDistance=" + positionDistance);
      PrintTools.debug(this, "orientationDistance=" + orientationDistance);
      
      assertEquals("Pose position error :" + positionDistance + " exceeds threshold: " + POSITION_THRESHOLD, 0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals("Pose orientation error :" + orientationDistance + " exceeds threshold: " + ORIENTATION_THRESHOLD, 0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
