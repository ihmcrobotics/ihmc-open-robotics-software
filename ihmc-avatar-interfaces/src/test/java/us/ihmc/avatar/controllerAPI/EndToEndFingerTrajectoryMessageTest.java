package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.FingerTrajectoryMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndFingerTrajectoryMessageTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testFingerTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      // define trajectories
      RobotSide robotSide = RobotSide.RIGHT;
      double[] trajectoryTimes = new double[] {2.0, 3.0, 4.0};
      double[] delayTimes = new double[] {1.0, 1.5, 2.0};
      double[] desiredPositions = new double[] {2.0, 2.5, 3.0};

      // messages
      FingerTrajectoryMessage fingerTrajectoryMessage = HumanoidMessageTools.createFingerTrajectoryMessage(robotSide, trajectoryTimes, delayTimes,
                                                                                                           desiredPositions);

      // publish
      drcSimulationTestHelper.publishToController(fingerTrajectoryMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);

      // define trajectories
      trajectoryTimes = new double[] {2.0, 3.0, 4.0};
      delayTimes = new double[] {1.0, 1.5, 2.0};
      desiredPositions = new double[] {-1.0, -1.0, -1.0};

      // messages
      fingerTrajectoryMessage = HumanoidMessageTools.createFingerTrajectoryMessage(robotSide, trajectoryTimes, delayTimes, desiredPositions);

      drcSimulationTestHelper.publishToController(fingerTrajectoryMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);

      // define trajectories
      trajectoryTimes = new double[] {2.0, 3.0, 4.0};
      delayTimes = new double[] {1.0, 1.5, 2.0};
      desiredPositions = new double[] {1.0, 1.0, 1.0};

      // messages
      fingerTrajectoryMessage = HumanoidMessageTools.createFingerTrajectoryMessage(robotSide, trajectoryTimes, delayTimes, desiredPositions);

      // publish
      drcSimulationTestHelper.publishToController(fingerTrajectoryMessage);

      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      HumanoidJointNameMap jointNameMap = (HumanoidJointNameMap) controllerFullRobotModel.getRobotSpecificJointNames();
      Joint wristJoint = drcSimulationTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQYoVariable().getDoubleValue();
         PrintTools.debug(this, fingerJoint.getName() + " q : " + q + " " + fingerJoint.getqDesired());
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + 5.0);
      assertTrue(success);
   }

   public void testFingerCloseMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;
      double trajectoryTime = 2.5;

      HandDesiredConfigurationMessage handDesiredConfigurationMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                                   HandConfiguration.CLOSE);

      drcSimulationTestHelper.publishToController(handDesiredConfigurationMessage);

      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      HumanoidJointNameMap jointNameMap = (HumanoidJointNameMap) controllerFullRobotModel.getRobotSpecificJointNames();
      Joint wristJoint = drcSimulationTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQYoVariable().getDoubleValue();
         PrintTools.debug(this, fingerJoint.getName() + " q : " + q + " " + fingerJoint.getqDesired());
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
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
      //if (simulationTestingParameters.getKeepSCSUp())
      if (true)
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