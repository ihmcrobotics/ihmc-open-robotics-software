package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCValveEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCTurnValveBehaviorTest implements MultiRobotTestInterface
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
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private final double DESIRED_VALVE_CLOSE_PERCENTAGE = 8.0;

   private static final boolean DEBUG = false;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10, "DRCJunkyCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(createTestEnvironment(), networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel(), controllerCommunicator);
   }

   private static DRCValveEnvironment createTestEnvironment()
   {
      double valveX = 2.0 * TurnValveBehavior.howFarToStandBackFromValve;
      double valveY = TurnValveBehavior.howFarToStandToTheRightOfValve;
      double valveZ = 1.0;
      double valveYaw_degrees = 0.0 * 45.0;

      return new DRCValveEnvironment(valveX, valveY, valveZ, valveYaw_degrees);
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testWalkAndTurnValveNoScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(worldFrame, valveTransformToWorld);
      SysoutTool.println("Valve Pose = " + valvePose, DEBUG);
      SysoutTool.println("Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()), DEBUG);

      final TurnValveBehavior turnValveBehavior = createNewTurnValveBehavior();

      String scriptName = null;

      ScriptBehaviorInputPacket scriptBehaviorInput = new ScriptBehaviorInputPacket(scriptName, valveTransformToWorld);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(scriptBehaviorInput);
      assertTrue(turnValveBehavior.hasInputBeenSet());


      double initialValveClosePercentage = valveRobot.getClosePercentage();
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(turnValveBehavior);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      SysoutTool.println("Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage,
            DEBUG);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & turnValveBehavior.isDone();
      
      assertTrue(success);
      assertTrue(finalValveClosePercentage > initialValveClosePercentage);
      assertTrue(finalValveClosePercentage > DESIRED_VALVE_CLOSE_PERCENTAGE);

      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold

      BambooTools.reportTestFinishedMessage();
   }

   private TurnValveBehavior createNewTurnValveBehavior()
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getFullRobotModel();
      ReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();
      CapturePointUpdatable capturePointUpdatable = drcBehaviorTestHelper.getCapturePointUpdatable();
      BooleanYoVariable yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      BooleanYoVariable yoTippingDetected = capturePointUpdatable.getTippingDetectedBoolean();
      WalkingControllerParameters walkingControllerParams = getRobotModel().getWalkingControllerParameters();

      final TurnValveBehavior turnValveBehavior = new TurnValveBehavior(communicationBridge, fullRobotModel, referenceFrames, yoTime, yoDoubleSupport,
            yoTippingDetected, walkingControllerParams);
      return turnValveBehavior;
   }

   private FramePose getRobotPose(ReferenceFrames referenceFrames)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose ret = new FramePose();

      drcBehaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      ret.setToZero(midFeetFrame);
      ret.changeFrame(worldFrame);

      return ret;
   }

}
