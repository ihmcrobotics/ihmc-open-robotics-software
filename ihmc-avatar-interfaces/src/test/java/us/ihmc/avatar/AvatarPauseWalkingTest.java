package us.ihmc.avatar;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class AvatarPauseWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;
   private YoBoolean walkPaused;

   public abstract double getSwingTime();

   public abstract double getTransferTime();

   public abstract double getFinalTransferDuration();

   public abstract double getStepLength();

   public abstract double getStepWidth();

   public abstract double getTimeForPausing();

   public abstract double getTimeForResuming();

   public abstract int getNumberOfFootsteps();


   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      System.out.println("blah? " + simulationTestingParameters.getKeepSCSUp());
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
      robotModel = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testPauseWalking() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      sendFootstepCommand(0.0, getNumberOfFootsteps());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getTimeForPausing()));
      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getTimeForResuming()));
      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testPauseWalkingForward() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      sendFootstepCommand(getStepLength(), getNumberOfFootsteps());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getTimeForPausing()));
      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getTimeForResuming()));
      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testPauseWalkingInitialTransfer() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      sendFootstepCommand(0.0, getNumberOfFootsteps());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);

      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testPauseWalkingForwardInitialTransfer() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      sendFootstepCommand(getStepLength(), getNumberOfFootsteps());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   private void sendFootstepCommand(double stepLength, int numberOfFootsteps)
   {
      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(getSwingTime(), getTransferTime());
      RobotSide side = RobotSide.LEFT;
      Quaternion orientation = new Quaternion();
      for (int i = 1; i < numberOfFootsteps; i++)
      {
         addFootstep(new Point3D(i * stepLength, side.negateIfRightSide(getStepWidth() / 2.0), 0.0), orientation, side, footstepMessage);
         side = side.getOppositeSide();
      }
      addFootstep(new Point3D((numberOfFootsteps - 1) * stepLength, side.negateIfRightSide(getStepWidth() / 2.0), 0.0), orientation, side,
                  footstepMessage);
      footstepMessage.setFinalTransferDuration(getFinalTransferDuration());
      drcSimulationTestHelper.send(footstepMessage);
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLocation);
      footstepData.getOrientation().set(orient);
      footstepData.setRobotSide(robotSide.toByte());
      message.getFootstepDataList().add().set(footstepData);
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return new OffsetAndYawRobotInitialSetup(new Vector3D(0.0, 0.0, 0.0), 0.0);
         }
      };
      robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      YoVariableRegistry registry = drcSimulationTestHelper.getYoVariableRegistry();
      walkPaused = new YoBoolean("isWalkPaused", registry);
      ThreadTools.sleep(1000);
      setupCameraSideView();
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}
