package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDebrisEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspPieceOfDebrisBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.UnfinishedTest;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@UnfinishedTest
public abstract class DRCGraspPieceOfDebrisBehaviorTest implements MultiRobotTestInterface
{

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

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
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private static final boolean DEBUG = false;

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final double FINGER_JOINT_1_EXPECTED_RADIANS = 0.22;
   private final double FINGER_JOINT_2_EXPECTED_RADIANS = 0.22;
   private final double PALM_JOINT_EXPECTED_RADIANS = 0.04;

   private final double FINGER_JOINT_1_ERROR_MARGIN_RADIANS = 0.05;
   private final double FINGER_JOINT_2_ERROR_MARGIN_RADIANS = 0.05;
   private final double PALM_JOINT_ERROR_MARGIN_RADIANS = 0.02;

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();
   private final KryoLocalPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCGraspPieceOfDebrisBehaviorTestControllerCommunicator");

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private SDFFullRobotModel fullRobotModel;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            Vector3d additionalOffset = new Vector3d(0.0, 0.0, 0.0);
            double yaw = 0.0;
            OffsetAndYawRobotInitialSetup offsetAndYawRobotInitialSetup = new OffsetAndYawRobotInitialSetup(additionalOffset, yaw);
            return offsetAndYawRobotInitialSetup;
         }
      };

      testEnvironment.addStandingDebris(0.65, -0.3, 0.0);
      testEnvironment.createDebrisContactController();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null, startingLocation,
            simulationTestingParameters, false, getRobotModel(), controllerCommunicator);

      Robot robotToTest = drcBehaviorTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcBehaviorTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());
   }

   @AverageDuration(duration = 300.0)
   @Test(timeout = 900000)
   public void testGraspingDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(communicationBridge, getRobotModel().createFullRobotModel(),
            getRobotModel(), yoTime, true);
      communicationBridge.attachGlobalListenerToController(graspPieceOfDebrisBehavior.getControllerGlobalPacketConsumer());

      graspPieceOfDebrisBehavior.initialize();

      // from DebrisTaskBehaviorPanel.storeDebrisDataInList
      //here the rigidBodyTransform is of with respect to the center of the debris, whereas in the UI, it is with respect to the corner of the debris
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform debrisTransform = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransform);
      debrisTransform.applyTranslation(new Vector3d(-testEnvironment.getDebrisDepth() / 2.0, -testEnvironment.getDebrisWidth() / 2.0, -testEnvironment
            .getDebrisLength() / 2.0));

      FrameVector tempGraspVector = new FrameVector(ReferenceFrame.getWorldFrame());
      tempGraspVector.set(-1.0, 0.0, 0.0);
      tempGraspVector.applyTransform(debrisTransform);
      Vector3d graspVector = new Vector3d();
      tempGraspVector.get(graspVector);

      FramePoint tempGraspVectorPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      tempGraspVectorPosition.setZ(0.5);
      tempGraspVectorPosition.applyTransform(debrisTransform);
      Point3d graspVectorPosition = new Point3d();
      tempGraspVectorPosition.get(graspVectorPosition);

      graspPieceOfDebrisBehavior.setGraspPose(debrisTransform, graspVectorPosition, graspVector, RobotSide.RIGHT);

      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());

      double graspTime = 20.0;
      executeBehavior(graspPieceOfDebrisBehavior, graspTime);
      success = success & graspPieceOfDebrisBehavior.isDone();

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      DoubleYoVariable fingerJoint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_1");
      DoubleYoVariable fingerJoint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_1");
      DoubleYoVariable palmJoint = (DoubleYoVariable) robot.getVariable("q_r_palm_finger_1_joint");

      success = success & Math.abs(fingerJoint1.getDoubleValue() - FINGER_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_1_ERROR_MARGIN_RADIANS;
      success = success & Math.abs(fingerJoint2.getDoubleValue() - FINGER_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_2_ERROR_MARGIN_RADIANS;
      success = success & Math.abs(palmJoint.getDoubleValue() - PALM_JOINT_EXPECTED_RADIANS) < PALM_JOINT_ERROR_MARGIN_RADIANS;

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
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
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      SysoutTool.println("done simulating behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue(), DEBUG);

      return ret;
   }

}
