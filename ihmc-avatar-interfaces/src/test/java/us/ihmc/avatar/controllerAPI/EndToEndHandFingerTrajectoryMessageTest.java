package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndHandFingerTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   protected HumanoidFloatingRootJointRobot controllerFullRobotModel;

   public abstract Packet<?> createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration);

   public void testCloseAndStopAndOpen() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(RobotSide.LEFT, HandConfiguration.CLOSE));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(RobotSide.LEFT, HandConfiguration.STOP));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(RobotSide.LEFT, HandConfiguration.OPEN));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);

      assertTrue(success);
   }

   public void testBasicGrip() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(RobotSide.LEFT, HandConfiguration.BASIC_GRIP));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      assertTrue(success);
   }

   public void testCloseAndOpenFingers() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(RobotSide.LEFT, HandConfiguration.CLOSE));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(RobotSide.LEFT, HandConfiguration.OPEN_FINGERS));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);

      assertTrue(success);
   }

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
}
