package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.util.Arrays;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCValveEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.simulationconstructionset.Robot;
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
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCTurnValveBehaviorTest implements MultiRobotTestInterface
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
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   
   private final double DESIRED_VALVE_CLOSE_PERCENTAGE = 8.0;
   
   private static final boolean DEBUG = true;
 
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final double valveX = 2.0 * TurnValveBehavior.howFarToStandBackFromValve;
   private final double valveY = TurnValveBehavior.howFarToStandToTheRightOfValve;
   private final double valveZ = 1.0;
   private final double valveYaw_degrees = 0.0 * 45.0;

   private final DRCValveEnvironment testEnvironment = new DRCValveEnvironment(valveX, valveY, valveZ, valveYaw_degrees);
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel());

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

	@AverageDuration
	@Test(timeout = 300000)
   public void testWalkAndTurnValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(worldFrame, valveTransformToWorld);
      SysoutTool.println("Valve Pose = " + valvePose, DEBUG);
      SysoutTool.println("Robot Pose = " + getRobotPose(), DEBUG);

      double elapsedTimeToWalkAndTurnValve = 20.0;

      final TurnValveBehavior turnValveBehavior = new TurnValveBehavior(communicationBridge, fullRobotModel, referenceFrames, yoTime, yoDoubleSupport,
            yoTippingDetected, getRobotModel().getWalkingControllerParameters());
      communicationBridge.attachGlobalListenerToController(turnValveBehavior.getControllerGlobalPacketConsumer());

//      String scriptName = "testTurnValveNoScript";
      String scriptName = null;

      ScriptBehaviorInputPacket scriptBehaviorInput = new ScriptBehaviorInputPacket(scriptName, valveTransformToWorld);
      turnValveBehavior.initialize();
      turnValveBehavior.setInput(scriptBehaviorInput);

      double initialValveClosePercentage = valveRobot.getClosePercentage();
      success = executeBehavior(turnValveBehavior, elapsedTimeToWalkAndTurnValve);
      assertTrue(success);
      double finalValveClosePercentage = valveRobot.getClosePercentage();
      SysoutTool.println("Initial valve close percentage: " + initialValveClosePercentage + ".  Final valve close percentage: " + finalValveClosePercentage, DEBUG);

      assertTrue(turnValveBehavior.isDone());
      assertTrue(finalValveClosePercentage > initialValveClosePercentage);
      assertTrue(finalValveClosePercentage > DESIRED_VALVE_CLOSE_PERCENTAGE);
      
      //TODO: Keep track of max icp error and verify that it doesn't exceed a reasonable threshold
      
      BambooTools.reportTestFinishedMessage();
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
}
