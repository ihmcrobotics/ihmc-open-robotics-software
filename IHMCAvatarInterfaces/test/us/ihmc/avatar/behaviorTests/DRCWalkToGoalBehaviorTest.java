package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToGoalBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket.WalkToGoalAction;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCWalkToGoalBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

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
      
      if(behaviorCommunicatorClient != null)
      {
         behaviorCommunicatorClient.close();
      }
      
      if(behaviorCommunicatorServer != null)
      {
         behaviorCommunicatorServer.close();
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final boolean DEBUG = true;

   private final double ASSUMED_WALKING_SPEED_mPerSec = 0.2;

   private final double POSITION_THRESHOLD = 0.1;
   private final double ORIENTATION_THRESHOLD = 0.05;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();

   private DoubleYoVariable yoTime;

   private ForceSensorDataHolder forceSensorDataHolder;
   private HumanoidRobotDataReceiver robotDataReceiver;

   private CommunicationBridge communicationBridge;

   private HumanoidFloatingRootJointRobot robot;
   private FullHumanoidRobotModel fullRobotModel;

   private PacketCommunicator behaviorCommunicatorServer;

   private PacketCommunicator behaviorCommunicatorClient;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      behaviorCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      behaviorCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      try
      {
         behaviorCommunicatorClient.connect();
         behaviorCommunicatorServer.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, getSimpleRobotName(),
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      drcSimulationTestHelper.attachListener(RobotConfigurationData.class, robotDataReceiver);

      
      
      
      PacketRouter<PacketDestination> networkProcessor = new PacketRouter<>(PacketDestination.class);
      networkProcessor.attachPacketCommunicator(PacketDestination.CONTROLLER, drcSimulationTestHelper.getControllerCommunicator());
      networkProcessor.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorCommunicatorClient);

      communicationBridge = new CommunicationBridge(behaviorCommunicatorServer);
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkDistance = RandomNumbers.nextDouble(new Random(), 1.0, 2.0);
      Vector2D walkDirection = new Vector2D(1, 0);
      double trajectoryTime = walkDistance / ASSUMED_WALKING_SPEED_mPerSec;

      WalkToGoalBehavior walkToGoalBehavior = testWalkToGoalBehavior(walkDistance, walkDirection, trajectoryTime);

      assertTrue(walkToGoalBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private WalkToGoalBehavior testWalkToGoalBehavior(double walkDistance, Vector2D walkDirection, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      FramePose2d initialMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);

      FramePose2d desiredMidFeetPose = offsetMidFeetPose2d(initialMidFeetPose, walkDistance, walkDirection);

      final WalkToGoalBehavior walkToGoalBehavior = new WalkToGoalBehavior(communicationBridge, fullRobotModel, yoTime, getRobotModel().getPhysicalProperties()
            .getAnkleHeight());
      walkToGoalBehavior.initialize();

      WalkToGoalBehaviorPacket requestedGoal = new WalkToGoalBehaviorPacket(desiredMidFeetPose.getX(), desiredMidFeetPose.getY(), desiredMidFeetPose.getYaw(),
            RobotSide.RIGHT);
      walkToGoalBehavior.getCommunicationBridge().consumeObjectFromNetwork(requestedGoal);

      WalkToGoalBehaviorPacket walkToGoalFindPathPacket = new WalkToGoalBehaviorPacket(WalkToGoalAction.FIND_PATH);
      walkToGoalBehavior.getCommunicationBridge().consumeObjectFromNetwork(walkToGoalFindPathPacket);

      WalkToGoalBehaviorPacket walkToGoalExecutePacket = new WalkToGoalBehaviorPacket(WalkToGoalAction.EXECUTE);
      walkToGoalBehavior.getCommunicationBridge().consumeObjectFromNetwork(walkToGoalExecutePacket);
      walkToGoalBehavior.doControl();
      assertTrue( walkToGoalBehavior.hasInputBeenSet() );
      
      boolean success = executeBehavior(walkToGoalBehavior, trajectoryTime);
      assertTrue(success);

      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(robot);
      //      FramePose2d finalMidFeetPose = getCurrentMidFeetPose2d(referenceFrames);

      PrintTools.debug(this, " initial Midfeet Pose :\n" + initialMidFeetPose + "\n");
      assertPosesAreWithinThresholds(desiredMidFeetPose, finalMidFeetPose);

      return walkToGoalBehavior;
   }

   private FramePose2d offsetMidFeetPose2d(FramePose2d initialPose, double walkDistance, Vector2D walkDirection)
   {
      FramePose2d ret = new FramePose2d(initialPose);

      walkDirection.normalize();
      ret.setX(initialPose.getX() + walkDistance * walkDirection.getX());
      ret.setY(initialPose.getY() + walkDistance * walkDirection.getY());

      return ret;
   }

   private boolean executeBehavior(final AbstractBehavior behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      if (DEBUG)
      {
         System.out.println("\n");
         PrintTools.debug(this, "starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
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

                  robotDataReceiver.updateRobotModel();
                  behavior.doControl();
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      PrintTools.debug(this, "done with behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());

      return ret;
   }

   private FramePose2d getCurrentMidFeetPose2d(HumanoidReferenceFrames referenceFrames)
   {
      robotDataReceiver.updateRobotModel();
      referenceFrames.updateFrames();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      FramePose midFeetPose = new FramePose();
      midFeetPose.setToZero(midFeetFrame);
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(midFeetPose.getReferenceFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose2d getCurrentMidFeetPose2dTheHardWayBecauseReferenceFramesDontUpdateProperly(HumanoidFloatingRootJointRobot robot)
   {
      FramePose midFeetPose = getRobotMidFeetPose(robot);

      FramePose2d ret = new FramePose2d();
      ret.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), midFeetPose.getX(), midFeetPose.getY(), midFeetPose.getYaw());

      return ret;
   }

   private FramePose getRobotMidFeetPose(HumanoidFloatingRootJointRobot robot)
   {
      FramePose leftFootPose = getRobotFootPose(robot, RobotSide.LEFT);
      FramePose rightFootPose = getRobotFootPose(robot, RobotSide.RIGHT);

      FramePose ret = new FramePose();
      ret.interpolate(leftFootPose, rightFootPose, 0.5);

      return ret;
   }

   private FramePose getRobotFootPose(HumanoidFloatingRootJointRobot robot, RobotSide robotSide)
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
         PrintTools.debug(this, " desired Midfeet Pose :\n" + framePose1 + "\n");
         PrintTools.debug(this, " actual Midfeet Pose :\n" + framePose2 + "\n");

         PrintTools.debug(this, " positionDistance = " + positionDistance);
         PrintTools.debug(this, " orientationDistance = " + orientationDistance);
      }

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }
}
