package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
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

   private final double POSITION_THRESHOLD = 0.007;
   private final double ORIENTATION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 2.0;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCHandPoseBehaviorTestControllerCommunicator");

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private final RobotSide robotSideToTest = RobotSide.LEFT;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private FullRobotModel fullRobotModel;

   private boolean returnValue = true;

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

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCHandPoseBehaviorTestJunkyCommunicator");

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

      communicationBridge.attachGlobalListenerToController(handPoseBehavior.getControllerGlobalPacketConsumer());

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

      communicationBridge.attachGlobalListenerToController(handPoseBehavior.getControllerGlobalPacketConsumer());

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

      final double swingTrajectoryTime = 4.0;

      handPoseBehavior.setInput(Frame.WORLD, pose, robotSideToTest, swingTrajectoryTime);

      final double secondsToPauseEarly = 2.0;
      final double simulationRunTime = swingTrajectoryTime - secondsToPauseEarly;

      setReturnValue(true);
      
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

               HandPoseStatus.Status status = handPoseBehavior.getStatus();

               if ((status != null) && (timeSpentSimulating > 0.5))
               {
                  if ((timeSpentSimulating > 0.5) && (timeSpentSimulating < (swingTrajectoryTime - 0.5)))
                  {
                     if (status != HandPoseStatus.Status.STARTED)
                     {
                        System.out.println("behaviorThreadFirst: status should be STARTED, = " + status + ", timeSpentSimulating=" + timeSpentSimulating);
                        setReturnValue(false);
                     }
                  }
                  else if (timeSpentSimulating > (swingTrajectoryTime + 0.5))
                  {
                     if (status != HandPoseStatus.Status.COMPLETED)
                     {
                        System.out.println("behaviorThreadFirst: status should be STARTED, = " + status + ", timeSpentSimulating=" + timeSpentSimulating);
                        setReturnValue(false);
                     }

                  }
               }
            }

            handPoseBehavior.pause();

            if (DEBUG)
               System.out.println("testHandPosePause: behaviorThreadFirst done, pause command sent");
         }
      };

      behaviorThreadFirst.start();

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      assertTrue(returnValue);
      
      fullRobotModel.updateFrames();
      FramePose handPoseAtPauseStart = new FramePose();
      handPoseAtPauseStart.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseAtPauseStart.changeFrame(ReferenceFrame.getWorldFrame());

      double timeToPause = 2.0;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToPause);

      handPoseBehavior.doControl();
      assertTrue(handPoseBehavior.isDone());
      
      fullRobotModel.updateFrames();
      FramePose handPoseAtPauseEnd = new FramePose();
      handPoseAtPauseEnd.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseAtPauseEnd.changeFrame(ReferenceFrame.getWorldFrame());

      
      
      if (DEBUG)
         System.out.println("testSimpleHandPoseMove: checking start and end of pause");

      assertPosesAreWithinThresholds(handPoseAtPauseStart, handPoseAtPauseEnd);

      handPoseBehavior.resume();

      setReturnValue(true);

      Thread behaviorThreadSecond = new Thread()
      {
         public void run()
         {
            double startTime = Double.NaN;
            boolean simStillRunning = true;
            boolean initalized = false;
            boolean ret = true;

            while (simStillRunning)
            {
               if (!initalized)
               {
                  startTime = yoTime.getDoubleValue();
                  initalized = true;
               }

               double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
               simStillRunning = timeSpentSimulating < (swingTrajectoryTime + 1.0);

               handPoseBehavior.doControl();

               HandPoseStatus.Status status = handPoseBehavior.getStatus();

               if ((status != null) && (timeSpentSimulating > 0.5))
               {
                  if ((timeSpentSimulating > 0.5) && (timeSpentSimulating < (swingTrajectoryTime - 0.5)))
                  {
                     if (status != HandPoseStatus.Status.STARTED)
                     {
                        System.out.println("behaviorThreadFirst: status should be STARTED, = " + status + ", timeSpentSimulating=" + timeSpentSimulating);
                        setReturnValue(false);
                     }
                     
                  }
                  else if (timeSpentSimulating > (swingTrajectoryTime + 0.5))
                  {
                     if (status != HandPoseStatus.Status.COMPLETED)
                     {
                        System.out.println("behaviorThreadFirst: status should be STARTED, = " + status + ", timeSpentSimulating=" + timeSpentSimulating);
                        setReturnValue(false);
                     }
                     
                  }
               }
            }
            
            

            if (DEBUG)
               System.out.println("testHandPosePause: behaviorThreadSecond done");
         }
      };

      behaviorThreadSecond.start();

      assertTrue(returnValue);
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(swingTrajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING);

      handPoseBehavior.doControl();
      HandPoseStatus.Status status = handPoseBehavior.getStatus();
      assertTrue(status.equals(HandPoseStatus.Status.COMPLETED));


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
   
   
   @Test
   public void testHandPoseMoveStop() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseBehavior handPoseBehavior = new HandPoseBehavior(communicationBridge, yoTime);

      communicationBridge.attachGlobalListenerToController(handPoseBehavior.getControllerGlobalPacketConsumer());

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

      final double simulationRunTime = swingTrajectoryTime - 2.0;

      Thread behaviorThreadFirst = new Thread()
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

      behaviorThreadFirst.start();

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      handPoseBehavior.stop();
      
      fullRobotModel.updateFrames();
      FramePose handPoseEnd = new FramePose();
      handPoseEnd.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseEnd.changeFrame(ReferenceFrame.getWorldFrame());

      final double simulationRunTime2 = 2.0;
      
      Thread behaviorThreadSecond = new Thread()
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
                  simStillRunning = timeSpentSimulating < simulationRunTime2;

                  handPoseBehavior.doControl();
               }
            }
         }
      };

      behaviorThreadSecond.start();
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime2);
      
      handPoseBehavior.doControl();
      HandPoseStatus.Status status = handPoseBehavior.getStatus();
      assertTrue(status.equals(HandPoseStatus.Status.COMPLETED));
      
      fullRobotModel.updateFrames();
      FramePose handPoseAfterResting = new FramePose();
      handPoseAfterResting.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      handPoseAfterResting.changeFrame(ReferenceFrame.getWorldFrame());

      assertPosesAreWithinThresholds(handPoseEnd, handPoseAfterResting);

      assertTrue(success);
      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   
   private void setReturnValue(boolean value)
   {
      returnValue = value;
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
