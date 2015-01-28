package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.fest.util.Files;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.networking.CommandRecorder;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngineSettings;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.ihmcPerception.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.humanoidRobot.footstep.footsepGenerator.SimplePathParameters;

public abstract class DRCScriptBehaviorTest implements MultiRobotTestInterface
{
   private final double POSITION_THRESHOLD = 0.007;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame recordFrame;
   String fileName = "1_ScriptBehaviorTest";
   TimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private static File file;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;
   private ReferenceFrames referenceFrames;

   private CapturePointUpdatable capturePointUpdatable;
   private BooleanYoVariable yoDoubleSupport;

   private double nominalComHeightAboveGround;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, false, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      referenceFrames = robotDataReceiver.getReferenceFrames();

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());

      file = new File(ScriptEngineSettings.scriptSavingDirectory + fileName + ScriptEngineSettings.extension);
      System.out.println(file.getAbsolutePath());

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);
      capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, new YoGraphicsListRegistry(), robot.getRobotsYoVariableRegistry());
      yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");

      Files.delete(file);
   }

   @Test(timeout = 300000)
   public void testComHeightScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      Point3d nominalComPosition = new Point3d();
      robot.computeCenterOfMass(nominalComPosition);
      nominalComHeightAboveGround = nominalComPosition.getZ();

      double desiredFinalHeightOffset = 0.8 * ComHeightPacket.MAX_COM_HEIGHT;

      ComHeightPacket comHeightPacket0 = new ComHeightPacket(0.8 * ComHeightPacket.MIN_COM_HEIGHT, 1.0);
      ComHeightPacket comHeightPacket1 = new ComHeightPacket(desiredFinalHeightOffset, 1.0);

      CommandRecorder commandRecorder = new CommandRecorder(timestampProvider);

      recordFrame = worldFrame;
      boolean overwriteExistingFile = true;
      commandRecorder.startRecording(fileName, worldFrame, overwriteExistingFile);

      commandRecorder.recordObject(comHeightPacket0);
      commandRecorder.recordObject(comHeightPacket1);

      commandRecorder.stopRecording();

      ScriptBehavior scriptBehavior = testScriptBehavior(4.0);
      assertTrue(scriptBehavior.isDone());

      Point3d finalComPoint = new Point3d();
      robot.computeCenterOfMass(finalComPoint);

      assertProperComHeightOffsetFromGround(desiredFinalHeightOffset, finalComPoint);

      robotDataReceiver.updateRobotModel();
   }

   @Test(timeout = 300000)
   public void testHandPosePacketScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;

      FramePose initialHandPose = getCurrentHandPose(fullRobotModel, robotSide);

      FramePose desiredHandPose1 = new FramePose(initialHandPose);
      desiredHandPose1.setZ(desiredHandPose1.getZ() + 0.05);
      HandPosePacket handPosePacket1 = createHandPosePacket(desiredHandPose1, robotSide, trajectoryTime);

      FramePose desiredHandPose2 = new FramePose(initialHandPose);
      desiredHandPose2.setZ(initialHandPose.getZ() + 0.2);
      desiredHandPose2.setOrientation(new double[] { 0.0, 0.0, 0.6 });
      HandPosePacket handPosePacket2 = createHandPosePacket(desiredHandPose2, robotSide, trajectoryTime);

      CommandRecorder commandRecorder = new CommandRecorder(timestampProvider);

      recordFrame = worldFrame;
      boolean overwriteExistingFile = true;
      commandRecorder.startRecording(fileName, worldFrame, overwriteExistingFile);

      commandRecorder.recordObject(handPosePacket1);
      commandRecorder.recordObject(handPosePacket2);

      commandRecorder.stopRecording();

      ScriptBehavior scriptBehavior = testScriptBehavior(2.0 * trajectoryTime);
      assertTrue(scriptBehavior.isDone());

      FramePose finalHandPose = getCurrentHandPose(fullRobotModel, robotSide);
      assertPosesAreWithinThresholds(desiredHandPose2, finalHandPose);
   }

   private FramePose getCurrentHandPose(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      robotDataReceiver.updateRobotModel();
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

   private ScriptBehavior testScriptBehavior(double trajectoryTime) throws SimulationExceededMaximumTimeException, FileNotFoundException
   {
      final ScriptBehavior scriptBehavior = new ScriptBehavior(communicationBridge, fullRobotModel, yoTime, yoDoubleSupport);
      communicationBridge.attachGlobalListenerToController(scriptBehavior.getControllerGlobalPacketConsumer());

      InputStream inputStream = new FileInputStream(new File(file.getAbsolutePath()));
      RigidBodyTransform scriptTransformToWorld = recordFrame.getTransformToDesiredFrame(worldFrame);

      scriptBehavior.initialize();
      scriptBehavior.importChildInputPackets(inputStream.toString(), inputStream, scriptTransformToWorld);

      boolean success = executeBehavior(scriptBehavior, trajectoryTime);
      assertTrue(success);

      return scriptBehavior;
   }

   private boolean executeBehavior(final BehaviorInterface behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      SysoutTool.println("\n starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue(), DEBUG);

      Thread behaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;

               while (simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }

                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;

                  behavior.doControl();

                  capturePointUpdatable.update(yoTime.getDoubleValue());
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      SysoutTool.println("done simulating behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue(), DEBUG);

      return ret;
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
      robotDataReceiver.updateRobotModel();
      ReferenceFrame handFrame = fullRobotModel.getHandControlFrame(robotside);
      FramePose currentHandPose = new FramePose();
      currentHandPose.setToZero(handFrame);

      currentHandPose.changeFrame(worldFrame);

      Point3d desiredHandPosition = new Point3d();
      currentHandPose.getPosition(desiredHandPosition);
      desiredHandPosition.setZ(desiredHandPosition.getZ() + 0.1);
      SysoutTool.println("desired Hand Position : " + desiredHandPosition, DEBUG);

      Quat4d desiredHandOrientation = new Quat4d();
      currentHandPose.getOrientation(desiredHandOrientation);

      HandPosePacket ret = new HandPosePacket(robotside, Frame.WORLD, desiredHandPosition, desiredHandOrientation, 1.0);

      return ret;
   }

   private FootstepDataList createFootStepDataList(Vector2d walkDeltaXY, ReferenceFrames referenceFrames)
   {
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
      SDFJointNameMap jointNameMap = (SDFJointNameMap) fullRobotModel.getRobotSpecificJointNames();
      Joint wristJoint = robot.getJoint(jointNameMap.getJointBeforeHandName(robotSide));

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      double totalFingerJointQ = 0.0;

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ().getDoubleValue();
         totalFingerJointQ += q;
         SysoutTool.println(fingerJoint.getName() + " q : " + q, DEBUG);
      }

      return totalFingerJointQ;
   }

   private void assertProperComHeightOffsetFromGround(double desiredHeightOffset, Point3d finalComPoint)
   {
      double actualHeightOffset = finalComPoint.getZ() - nominalComHeightAboveGround;

      if (DEBUG)
      {
         SysoutTool.println("desiredHeightOffset: " + desiredHeightOffset);
         SysoutTool.println("actualHeightOffset: " + actualHeightOffset);
      }

      assertEquals(desiredHeightOffset, actualHeightOffset, POSITION_THRESHOLD);
   }

   private void assertOrientationsAreWithinThresholds(Quat4d desiredQuat, ReferenceFrame frameToCheck)
   {
      fullRobotModel.updateFrames();
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

      SysoutTool.println("orientationDistance=" + orientationDistance, DEBUG);

      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }

   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      SysoutTool.println("positionDistance=" + positionDistance, DEBUG);
      SysoutTool.println("orientationDistance=" + orientationDistance, DEBUG);

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
