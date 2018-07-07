package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndHandFingerTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   protected HumanoidFloatingRootJointRobot controllerFullRobotModel;

   public abstract Packet<?> createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration);

   public void testClose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;
      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.CLOSE));
      double fingerJointQAtInitial = getTotalFingerJointQ(robotSide);
      
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      double fingerJointQAtFinal = getTotalFingerJointQ(robotSide);

      assertTrue(fingerJointQAtFinal > fingerJointQAtInitial);
   }
   
   public void testCloseAndStopAndOpen() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      controllerFullRobotModel = drcSimulationTestHelper.getRobot();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;
      
      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.CLOSE));
      double fingerJointQAtInitial = getTotalFingerJointQ(robotSide);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      
      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.STOP));
      double fingerJointQAtStop = getTotalFingerJointQ(robotSide);
      assertTrue(fingerJointQAtStop > fingerJointQAtInitial);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.OPEN));
      double fingerJointQAtOpen = getTotalFingerJointQ(robotSide);
      assertTrue(Math.abs(fingerJointQAtOpen - fingerJointQAtStop) < 1.0);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      double fingerJointQAtFinal = getTotalFingerJointQ(robotSide);
      assertTrue(fingerJointQAtFinal < fingerJointQAtStop);
      

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

      RobotSide robotSide = RobotSide.RIGHT;
      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.BASIC_GRIP));
      double fingerJointQAtInitial = getTotalFingerJointQ(robotSide);
      
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      double fingerJointQAtFinal = getTotalFingerJointQ(robotSide);

      assertTrue(fingerJointQAtFinal > fingerJointQAtInitial);
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

      RobotSide robotSide = RobotSide.RIGHT;
      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.CLOSE));
      double fingerJointQAtInitial = getTotalFingerJointQ(robotSide);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      drcSimulationTestHelper.publishToController(createTrajectoryMessage(robotSide, HandConfiguration.OPEN_FINGERS));
      double fingerJointQAtOpen = getTotalFingerJointQ(robotSide);

      assertTrue(fingerJointQAtOpen > fingerJointQAtInitial);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      double fingerJointQAtFinal = getTotalFingerJointQ(robotSide);

      assertTrue(fingerJointQAtFinal < fingerJointQAtOpen);
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
   
   private double getTotalFingerJointQ(RobotSide robotSide)
   {
      double ret = 0.0;

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      HumanoidJointNameMap jointNameMap = (HumanoidJointNameMap) drcSimulationTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames();
      Joint wristJoint = drcSimulationTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQYoVariable().getDoubleValue();
         ret += q;
         if (true)
         {
            PrintTools.debug(this, fingerJoint.getName() + " q : " + q);
         }
      }

      return ret;
   }
}
