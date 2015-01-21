package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.LookAtBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCLookAtBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

   private final double MAX_ANGLE_TO_TEST_RAD = 15.0 * Math.PI / 180.0;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;

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

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());
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
   }

   //TODO: Fix HeadOrienationManager() so that head actually tracks desired yaw and roll orientations.  Currently, only pitch orientation tracks properly.

   @Test(timeout = 300000)
   public void testLookAtPitch() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 1, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      Quat4d desiredHeadQuat = convertAxisAngleToQuat(axis, rotationAngle);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testLookAtYaw() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(0, 0, 1);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      Quat4d desiredHeadQuat = convertAxisAngleToQuat(axis, rotationAngle);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   //   @Test(timeout = 300000)
   public void testLookAtRoll() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Vector3d axis = new Vector3d(1, 0, 0);
      double rotationAngle = MAX_ANGLE_TO_TEST_RAD * RandomTools.generateRandomDouble(new Random(), 0.3, 1.0);
      Quat4d desiredHeadQuat = convertAxisAngleToQuat(axis, rotationAngle);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   // @Test(timeout = 300000)
   public void testLookAtRandom() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double trajectoryTime = 4.0;
      Quat4d desiredHeadQuat = new Quat4d(RandomTools.generateRandomQuaternion(new Random(), MAX_ANGLE_TO_TEST_RAD));

      LookAtBehavior lookAtBehavior = testLookAtBehavior(trajectoryTime, desiredHeadQuat);

      assertTrue(lookAtBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   private LookAtBehavior testLookAtBehavior(double trajectoryTime, Quat4d desiredHeadQuat) throws SimulationExceededMaximumTimeException
   {
      FramePose initialHeadPose = getCurrentHeadPose(fullRobotModel);
      Point3d desiredLookAtPoint = computeDesiredLookAtPoint(initialHeadPose, desiredHeadQuat);

      LookAtBehavior lookAtBehavior = testLookAtBehavior(desiredLookAtPoint, trajectoryTime);

      FramePose finalHeadPose = getCurrentHeadPose(fullRobotModel);
      Quat4d finalHeadQuat = new Quat4d();
      finalHeadPose.getOrientation(finalHeadQuat);

      assertAxesParallel(desiredHeadQuat, finalHeadQuat);
      
      return lookAtBehavior;
   }
   
   private LookAtBehavior testLookAtBehavior(Point3d pointToLookAt, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final LookAtBehavior lookAtBehavior = new LookAtBehavior(communicationBridge);
      communicationBridge.attachGlobalListenerToController(lookAtBehavior.getControllerGlobalPacketConsumer());

      lookAtBehavior.setLookAtLocation(pointToLookAt);

      success = success && executeBehavior(lookAtBehavior, trajectoryTime);

      assertTrue(success);
      
      return lookAtBehavior;
   }

   private void assertAxesParallel(Quat4d desiredHeadQuat, Quat4d finalHeadQuat)
   {
      Vector3d desiredLookAxis = getQuatAxis(desiredHeadQuat);
      Vector3d actualLookAxis = getQuatAxis(finalHeadQuat);

      desiredLookAxis.normalize();
      actualLookAxis.normalize();

      double angleBetweenAxes = Math.abs(Math.acos(desiredLookAxis.dot(actualLookAxis)));

      assertEquals(0.0, angleBetweenAxes, ORIENTATION_THRESHOLD);
   }

   private Vector3d getQuatAxis(Quat4d quat)
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      axisAngle.set(quat);

      Vector3d ret = new Vector3d(axisAngle.x, axisAngle.y, axisAngle.z);

      return ret;
   }

   private Quat4d convertAxisAngleToQuat(Vector3d axis, double rotationAngle)
   {
      AxisAngle4d desiredAxisAngle = new AxisAngle4d();
      desiredAxisAngle.set(axis, rotationAngle);

      Quat4d desiredHeadQuat = new Quat4d();
      desiredHeadQuat.set(desiredAxisAngle);

      return desiredHeadQuat;
   }

   private FramePose getCurrentHeadPose(FullRobotModel fullRobotModel)
   {
      FramePose ret = new FramePose();

      fullRobotModel.updateFrames();
      ReferenceFrame headFrame = fullRobotModel.getHead().getBodyFixedFrame();

      ret.setToZero(headFrame);
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   private Point3d computeDesiredLookAtPoint(FramePose currentHeadPose, Quat4d desiredHeadQuat)
   {
      Vector3d desiredLookAtAxis = getLookAtAxis(desiredHeadQuat);

      Point3d desiredLookAtPt = new Point3d(currentHeadPose.getFramePointCopy().getPoint());
      desiredLookAtPt.add(desiredLookAtAxis);

      return desiredLookAtPt;
   }

   private Vector3d getLookAtAxis(Quat4d headQuat)
   {
      AxisAngle4d headAxisAngle = new AxisAngle4d();
      headAxisAngle.set(headQuat);

      Vector3d headToLookAtVec = new Vector3d(headAxisAngle.x, headAxisAngle.y, headAxisAngle.z);
      return headToLookAtVec;
   }

   private boolean executeBehavior(final BehaviorInterface behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      if (DEBUG)
      {
         System.out.println("\n");
         SysoutTool.println("starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
      }

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
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      if (DEBUG)
      {
         SysoutTool.println("done with behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
      }

      return ret;
   }
}
