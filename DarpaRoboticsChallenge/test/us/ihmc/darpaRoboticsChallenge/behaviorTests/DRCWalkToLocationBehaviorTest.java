package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.vecmath.Vector2d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
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
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCWalkToLocationBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = false || createMovie;

   private final double ASSUMED_WALKING_SPEED_mPerSec = 0.2;

   private final double POSITION_THRESHOLD = 0.015;
   private final double ORIENTATION_THRESHOLD = 0.05;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private DoubleYoVariable yoTime;

   private ForceSensorDataHolder forceSensorDataHolder;
   private RobotDataReceiver robotDataReceiver;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;

   private ReferenceFrames referenceFrames;
   private WalkingControllerParameters walkingControllerParameters;

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
      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      referenceFrames = robotDataReceiver.getReferenceFrames();
      walkingControllerParameters = getRobotModel().getWalkingControllerParameters();

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

   @Test(timeout = 300000)
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      double walkDistance = RandomTools.generateRandomDouble(new Random(), 1.0, 2.0);
      Vector2d walkDirection = new Vector2d(1, 0);
      double trajectoryTime = walkDistance / ASSUMED_WALKING_SPEED_mPerSec;

      WalkToLocationBehavior walkToLocationBehavior = testWalkToLocationBehavior(walkDistance, walkDirection, trajectoryTime);

      assertTrue(walkToLocationBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testTurn90WalkTurnNeg90() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = RandomTools.generateRandomDouble(new Random(), 1.0, 2.0);
      Vector2d walkDirection = new Vector2d(0, 1);
      double trajectoryTime = walkDistance / ASSUMED_WALKING_SPEED_mPerSec;
      trajectoryTime *= 2.5; // increase sim time since we have to turn in place 90 degrees twice

      WalkToLocationBehavior walkToLocationBehavior = testWalkToLocationBehavior(walkDistance, walkDirection, trajectoryTime);

      assertTrue(walkToLocationBehavior.isDone());
      
      BambooTools.reportTestFinishedMessage();
   }

   private WalkToLocationBehavior testWalkToLocationBehavior(double walkDistance, Vector2d walkDirection, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      FramePose2d initialMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      FramePose2d desiredMidFeetPose = new FramePose2d(initialMidFeetPose);

      walkDirection.normalize();
      desiredMidFeetPose.setX(initialMidFeetPose.getX() + walkDistance * walkDirection.getX());
      desiredMidFeetPose.setY(initialMidFeetPose.getY() + walkDistance * walkDirection.getY());

      final WalkToLocationBehavior walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            walkingControllerParameters);
      communicationBridge.attachGlobalListenerToController(walkToLocationBehavior.getControllerGlobalPacketConsumer());
      
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(desiredMidFeetPose);

      boolean success = executeBehavior(walkToLocationBehavior, trajectoryTime);
      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      if (DEBUG)
      {
         SysoutTool.println(" initial Midfeet Pose :\n" + initialMidFeetPose + "\n");
      }
      assertPosesAreWithinThresholds(desiredMidFeetPose, finalMidFeetPose);

      assertTrue(success);

      return walkToLocationBehavior;
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

   private FramePose2d getCurrentMidFeetPose2d(ReferenceFrames referenceFrames)
   {
      referenceFrames.updateFrames();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      FramePose midFeetPose = new FramePose();
      midFeetPose.setToZero(midFeetFrame);
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(midFeetPose.getReferenceFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose2d getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(SDFRobot robot)
   {
      FramePose midFeetPose = getRobotMidFeetPose(robot);

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose getRobotMidFeetPose(SDFRobot robot)
   {
      FramePose leftFootPose = getRobotFootPose(robot, RobotSide.LEFT);
      FramePose rightFootPose = getRobotFootPose(robot, RobotSide.RIGHT);

      FramePose ret = new FramePose();
      ret.interpolate(leftFootPose, rightFootPose, 0.5);

      return ret;
   }

   private FramePose getRobotFootPose(SDFRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose ret = new FramePose();
      ret.setPose(ankleTransformToWorld);

      return ret;
   }

   private void assertPosesAreWithinThresholds(FramePose2d framePose1, FramePose2d framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         SysoutTool.println(" desired Midfeet Pose :\n" + framePose1 + "\n");
         SysoutTool.println(" actual Midfeet Pose :\n" + framePose2 + "\n");

         SysoutTool.println(" positionDistance = " + positionDistance);
         SysoutTool.println(" orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
