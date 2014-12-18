package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.*;

import java.util.Arrays;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCHandPoseBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean KEEP_SCS_UP = false;
   private static final boolean DEBUG = false;

   private final double POSITION_THRESHOLD = 0.001;
   private final double ORIENTATION_THRESHOLD = 0.005;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 2.0;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;

   private DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private ObjectCommunicator controllerCommunicator = new LocalObjectCommunicator();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RobotSide robotSideToTest = RobotSide.LEFT;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private FullRobotModel fullRobotModel;


   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
              DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, false, getRobotModel());


      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      ObjectCommunicator junkyObjectCommunicator = new LocalObjectCommunicator();

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());


      // setupCameraForHandstepsOnWalls();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testSimpleHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseBehavior handPoseBehavior = new HandPoseBehavior(communicationBridge, yoTime);

      communicationBridge.attachGlobalListenerToController(handPoseBehavior.getControllerGlobalObjectConsumer());

      fullRobotModel.updateFrames();

      // ReferenceFrame handFrame = fullRobotModel.getHandControlFrame(robotSideToTest);

      FramePose handPoseStart = new FramePose();
      handPoseStart.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));

      handPoseStart.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose handPoseTarget = new FramePose(handPoseStart);
      handPoseTarget.setZ(handPoseTarget.getZ() + 0.2);
      handPoseTarget.setOrientation(new double[] {0.0, 0.0, 0.6});

      RigidBodyTransform pose = new RigidBodyTransform();    // handFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      handPoseTarget.getPose(pose);

      double swingTrajectoryTime = 2.0;

      handPoseBehavior.setInput(Frame.WORLD, pose, robotSideToTest, swingTrajectoryTime);

      final double simulationRunTime = swingTrajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

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

                  handPoseBehavior.doControl();
               }
            }
         }
      };

      behaviorThread.start();

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      fullRobotModel.updateFrames();
      FramePose handPoseEnd = new FramePose();
      handPoseEnd.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseEnd.changeFrame(ReferenceFrame.getWorldFrame());

      assertPosesAreWithinThresholds(handPoseEnd, handPoseTarget);

      assertTrue(success);
      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testHandPosePause() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseBehavior handPoseBehavior = new HandPoseBehavior(communicationBridge, yoTime);

      communicationBridge.attachGlobalListenerToController(handPoseBehavior.getControllerGlobalObjectConsumer());

      fullRobotModel.updateFrames();

      // ReferenceFrame handFrame = fullRobotModel.getHandControlFrame(robotSideToTest);

      FramePose handPoseStart = new FramePose();
      handPoseStart.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));

      handPoseStart.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose handPoseTarget = new FramePose(handPoseStart);
      handPoseTarget.setZ(handPoseTarget.getZ() + 0.3);
      handPoseTarget.setOrientation(new double[] {0.0, 0.0, 0.6});

      RigidBodyTransform pose = new RigidBodyTransform();    // handFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      handPoseTarget.getPose(pose);

      double swingTrajectoryTime = 4.0;

      handPoseBehavior.setInput(Frame.WORLD, pose, robotSideToTest, swingTrajectoryTime);

      final double secondsToPauseEarly = 2.0;
      final double simulationRunTime = swingTrajectoryTime - secondsToPauseEarly;

      Thread behaviorThreadFirst = new Thread()
      {
         public void run()
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
               simStillRunning = timeSpentSimulating < (simulationRunTime - 0.5);

               handPoseBehavior.doControl();
            }

            handPoseBehavior.pause();

            if (DEBUG)
               System.out.println("testHandPosePause: behaviorThreadFirst done, pause command sent");
         }
      };

      behaviorThreadFirst.start();

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      fullRobotModel.updateFrames();
      FramePose handPoseAtPauseStart = new FramePose();
      handPoseAtPauseStart.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseAtPauseStart.changeFrame(ReferenceFrame.getWorldFrame());

      double timeToPause = 2.0;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToPause);

      fullRobotModel.updateFrames();
      FramePose handPoseAtPauseEnd = new FramePose();
      handPoseAtPauseEnd.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseAtPauseEnd.changeFrame(ReferenceFrame.getWorldFrame());

      if (DEBUG)
         System.out.println("testSimpleHandPoseMove: checking start and end of pause");

      assertPosesAreWithinThresholds(handPoseAtPauseStart, handPoseAtPauseEnd);

      handPoseBehavior.resume();


      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(swingTrajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);


      fullRobotModel.updateFrames();
      FramePose handPoseAtEnd = new FramePose();
      handPoseAtEnd.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseAtEnd.changeFrame(ReferenceFrame.getWorldFrame());

      if (DEBUG)
         System.out.println("testSimpleHandPoseMove: checking start and end");

      assertPosesAreWithinThresholds(handPoseAtEnd, handPoseTarget);

      assertTrue(success);
      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }


   private void assertPosesAreWithinThresholds(FramePose framePose1, FramePose framePose2)
   {
      double positionDistance = framePose1.getPositionDistance(framePose2);
      double orientationDistance = framePose1.getOrientationDistance(framePose2);

      if (DEBUG)
      {
         System.out.println("testSimpleHandPoseMove: positionDistance=" + positionDistance);
         System.out.println("testSimpleHandPoseMove: orientationDistance=" + orientationDistance);
      }

      assertEquals(0.0, positionDistance, POSITION_THRESHOLD);
      assertEquals(0.0, orientationDistance, ORIENTATION_THRESHOLD);
   }



}
