package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import java.util.Arrays;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
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
import us.ihmc.yoUtilities.humanoidRobot.footstep.footsepGenerator.SimplePathParameters;

public abstract class DRCFootstepListBehaviorTest implements MultiRobotTestInterface
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

   private WalkingControllerParameters walkingControllerParameters;
   
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

      walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      
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
   
   @Test(timeout=300000)
   public void testSimpleFootstepList() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final FootstepListBehavior footstepListBehavior = new FootstepListBehavior(communicationBridge);

      communicationBridge.attachGlobalListenerToController(footstepListBehavior.getControllerGlobalPacketConsumer());

      fullRobotModel.updateFrames();

      FootstepDataList footstepDataList = getFootstepDataList(walkingControllerParameters);
      footstepListBehavior.set(footstepDataList);
      
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

//      handPoseBehavior.setInput(Frame.WORLD, pose, robotSideToTest, swingTrajectoryTime);
//
//      final double simulationRunTime = swingTrajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;
//
//      Thread behaviorThread = new Thread()
//      {
//         public void run()
//         {
//            {
//               double startTime = Double.NaN;
//               boolean simStillRunning = true;
//               boolean initalized = false;
//
//               while (simStillRunning)
//               {
//                  if (!initalized)
//                  {
//                     startTime = yoTime.getDoubleValue();
//                     initalized = true;
//                  }
//
//                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
//                  simStillRunning = timeSpentSimulating < simulationRunTime;
//
//                  handPoseBehavior.doControl();
//               }
//            }
//         }
//      };
//
//      behaviorThread.start();
//
//      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);
//
//      fullRobotModel.updateFrames();
//      FramePose handPoseEnd = new FramePose();
//      handPoseEnd.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
//      handPoseEnd.changeFrame(ReferenceFrame.getWorldFrame());
//
//      assertPosesAreWithinThresholds(handPoseEnd, handPoseTarget);
//
//      assertTrue(success);
//      assertTrue(handPoseBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private static FootstepDataList getFootstepDataList(WalkingControllerParameters walkingControllerParameters)
   {
      SimplePathParameters simplePathParameters = new SimplePathParameters(walkingControllerParameters.getMaxStepLength(), walkingControllerParameters.getInPlaceWidth(), 0.0,
            Math.toRadians(20.0), Math.toRadians(10.0), 0.4); // 10 5 0.4
//      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);
//
//      for (RobotSide robotSide : RobotSide.values)
//      {
//         feet.put(robotSide, fullRobotModel.getFoot(robotSide));
//         soleFrames.put(robotSide, fullRobotModel.getSoleFrame(robotSide));
//      }
      
      return null;
   }



}
