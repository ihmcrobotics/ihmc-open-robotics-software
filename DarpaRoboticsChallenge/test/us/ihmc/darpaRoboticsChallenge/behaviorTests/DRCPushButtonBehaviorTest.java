package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.packets.behaviors.ButtonData;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorButtonPacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCButtonEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.PushButtonBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableButtonRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCPushButtonBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static ArrayList<ButtonData> buttonData = new ArrayList<ButtonData>();
   private static int N = 0;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + "before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if(drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(createTestEnvironment(), getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   private static DRCButtonEnvironment createTestEnvironment()
   {
      ArrayList<Point3d> buttonLocations = new ArrayList<Point3d>();
      ArrayList<Vector3d> buttonPushDirections = new ArrayList<Vector3d>();

      // !! Five buttons: OutOfMemoryError on local machine!!
      // Define the locations of the buttons:

      buttonLocations.add(new Point3d(0.8, -0.3, 1.0));
      buttonLocations.add(new Point3d(0.7, -0.6, 1.0));
      buttonLocations.add(new Point3d(0.7, -0.6, 1.3));

      // Define the push direction in Space
      buttonPushDirections.add(new Vector3d(1.0, 0.0, 0.0));
      buttonPushDirections.add(new Vector3d(1.0, -1.0, 0.0));
      buttonPushDirections.add(new Vector3d(1.0, -1.0, 0.5));

      N = buttonLocations.size();

      for(int i = 0; i < N; i++)
      {
         buttonData.add(new ButtonData(buttonPushDirections.get(i), buttonLocations.get(i)));
      }


      return new DRCButtonEnvironment(buttonLocations, buttonPushDirections);
   }

   @EstimatedDuration(duration = 100.0)
   @Test(timeout = 500000)

   public void testPushButton() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ArrayList<ContactableButtonRobot> buttonRobots = new ArrayList<ContactableButtonRobot>();
      ArrayList<Boolean> initialButtonStates =  new ArrayList<Boolean>();

      for(Robot buttonRobot : testEnvironment.getEnvironmentRobots())
      {
         buttonRobots.add((ContactableButtonRobot) buttonRobot);
         initialButtonStates.add(((ContactableButtonRobot)buttonRobot).getButtonStatus());

      }

      final PushButtonBehavior pushButtonBehavior = createNewPushButtonBehavior();

      HumanoidBehaviorButtonPacket buttonPacket = new HumanoidBehaviorButtonPacket(buttonData);

      pushButtonBehavior.setInputs(buttonPacket);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(pushButtonBehavior);


      ArrayList<Boolean> finalButtonStates =  new ArrayList<Boolean>();

      for(Robot buttonRobot : testEnvironment.getEnvironmentRobots())
      {
         finalButtonStates.add(((ContactableButtonRobot)buttonRobot).getButtonStatus());

         // success for the button means status false since they are switched off in this configuration.
         success = success & !((ContactableButtonRobot) buttonRobot).getButtonStatus();
      }


      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & pushButtonBehavior.isDone();
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private PushButtonBehavior createNewPushButtonBehavior()
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();
      SideDependentList<WristForceSensorFilteredUpdatable> wristSensors = drcBehaviorTestHelper.getWristForceSensorUpdatableSideDependentList();

      final PushButtonBehavior pushButtonBehavior = new PushButtonBehavior(communicationBridge, referenceFrames, yoTime, wristSensors);

      return pushButtonBehavior;
   }

}
