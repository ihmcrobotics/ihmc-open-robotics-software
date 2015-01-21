package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCValveEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.ihmcPerception.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
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

public abstract class DRCTurnValveBehaviorTest implements MultiRobotTestInterface
{
   private final double POSITION_THRESHOLD = 0.007;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private static final boolean DEBUG = true;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = true || createMovie;

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCValveEnvironment testEnvironment = new DRCValveEnvironment(2.0 * TurnValveBehavior.howFarToStandBackFromValve, TurnValveBehavior.howFarToStandToTheRightOfValve, 1.0);
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
   private BooleanYoVariable yoTippingDetected;

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

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);
      capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, new YoGraphicsListRegistry(), robot.getRobotsYoVariableRegistry());
      yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      yoTippingDetected = capturePointUpdatable.getTippingDetectedBoolean();

   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      ThreadTools.sleepForever();
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test(timeout = 300000)
   public void testWalkAndTurnValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(worldFrame, valveTransformToWorld);
      SysoutTool.println("Valve Pose = " + valvePose);
      SysoutTool.println("Robot Pose = " + getRobotPose());

      double trajectoryTime = 20.0;

      final TurnValveBehavior turnValveBehavior = new TurnValveBehavior(communicationBridge, fullRobotModel, referenceFrames, yoTime, yoDoubleSupport,
            yoTippingDetected, getRobotModel().getWalkingControllerParameters());
      communicationBridge.attachGlobalListenerToController(turnValveBehavior.getControllerGlobalPacketConsumer());


      ScriptBehaviorInputPacket scriptBehaviorInput = new ScriptBehaviorInputPacket(new String("wasGoingToTurnTheValveButIGotHigh"), valveTransformToWorld);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(scriptBehaviorInput);

      success = executeBehavior(turnValveBehavior, trajectoryTime);
      assertTrue(success);

      assertTrue(turnValveBehavior.isDone());
   }

   private FramePose getRobotPose()
   {
      FramePose ret = new FramePose();

      robotDataReceiver.updateRobotModel();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      ret.setToZero(midFeetFrame);
      ret.changeFrame(worldFrame);

      return ret;
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
