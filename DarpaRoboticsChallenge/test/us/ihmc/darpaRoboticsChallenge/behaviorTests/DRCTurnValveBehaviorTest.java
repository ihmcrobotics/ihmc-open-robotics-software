package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCValveEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public abstract class DRCTurnValveBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
 
   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      KryoLocalPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCHandPoseBehaviorTestControllerCommunicator");

      testEnvironment = createTestEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel(), controllerCommunicator);
      
      
      Robot robot = drcBehaviorTestHelper.getRobot();
      yoTime = robot.getYoTime();

      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);
      
      CapturePointUpdatable capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, new YoGraphicsListRegistry(), robot.getRobotsYoVariableRegistry());
      yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      yoTippingDetected = capturePointUpdatable.getTippingDetectedBoolean();
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
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   
   private static DRCValveEnvironment createTestEnvironment()
   {
      double valveX = 2.0 * TurnValveBehavior.howFarToStandBackFromValve;
      double valveY = TurnValveBehavior.howFarToStandToTheRightOfValve;
      double valveZ = 1.0;
      double valveYaw_degrees = 0.0 * 45.0;

      return new DRCValveEnvironment(valveX, valveY, valveZ, valveYaw_degrees);
   }
   
   private DRCValveEnvironment testEnvironment;
   private DoubleYoVariable yoTime;
   
   private BooleanYoVariable yoDoubleSupport;
   private BooleanYoVariable yoTippingDetected;

   private final double DESIRED_VALVE_CLOSE_PERCENTAGE = 8.0;
   private static final boolean DEBUG = true;

   
	@AverageDuration
	@Test(timeout = 300000)
   public void testWalkAndTurnValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

       ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(worldFrame, valveTransformToWorld);
      SysoutTool.println("Valve Pose = " + valvePose, DEBUG);
      SysoutTool.println("Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()), DEBUG);

      double elapsedTimeToWalkAndTurnValve = 20.0;

      final TurnValveBehavior turnValveBehavior = new TurnValveBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getFullRobotModel(), drcBehaviorTestHelper.getReferenceFrames(), yoTime, yoDoubleSupport,
            yoTippingDetected, getRobotModel().getWalkingControllerParameters());
      
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      communicationBridge.attachGlobalListenerToController(turnValveBehavior.getControllerGlobalPacketConsumer());

//      String scriptName = "testTurnValveNoScript";
      String scriptName = null;

      ScriptBehaviorInputPacket scriptBehaviorInput = new ScriptBehaviorInputPacket(scriptName, valveTransformToWorld);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(scriptBehaviorInput);
      assertTrue(turnValveBehavior.hasInputBeenSet());

      double initialValveClosePercentage = valveRobot.getClosePercentage();
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(turnValveBehavior, elapsedTimeToWalkAndTurnValve);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      SysoutTool.println("Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage, DEBUG);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      assertTrue(success);

      success = success & turnValveBehavior.isDone();
      assertTrue(finalValveClosePercentage > initialValveClosePercentage);
      assertTrue(finalValveClosePercentage > DESIRED_VALVE_CLOSE_PERCENTAGE);
      
      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold
      
      
      BambooTools.reportTestFinishedMessage();
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
