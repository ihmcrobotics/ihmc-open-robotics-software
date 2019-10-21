package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndHandFingerTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final double epsilon = 0.05;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   protected HumanoidFloatingRootJointRobot controllerFullRobotModel;

   public abstract Object createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration);

   public void testClose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
         drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.CLOSE));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

      for (RobotSide robotSide : RobotSide.values)
         assertDesiredFingerJoint(robotSide, HandConfiguration.CLOSE, epsilon);
   }

   public void testCloseAndStopAndOpen() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
         drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.CLOSE));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

      for (RobotSide robotSide : RobotSide.values)
         drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.STOP));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      for (RobotSide robotSide : RobotSide.values)
         drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.OPEN));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

      for (RobotSide robotSide : RobotSide.values)
         assertDesiredFingerJoint(robotSide, HandConfiguration.OPEN, epsilon);
   }

   public void testCloseAndOpenFingers() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
         drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.CLOSE));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

      for (RobotSide robotSide : RobotSide.values)
         assertDesiredFingerJoint(robotSide, HandConfiguration.CLOSE, epsilon);

      for (RobotSide robotSide : RobotSide.values)
         drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.OPEN_FINGERS));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

      for (RobotSide robotSide : RobotSide.values)
         assertDesiredFingerJoint(robotSide, HandConfiguration.OPEN_FINGERS, epsilon);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
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

   public abstract void assertDesiredFingerJoint(RobotSide robotSide, HandConfiguration handConfiguration, double epsilon);
}
