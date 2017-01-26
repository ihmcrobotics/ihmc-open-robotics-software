package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadUserControlModeState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage.NeckControlMode;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndNeckDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 14.8)
   @Test(timeout = 74000)
   public void testSimpleCommands() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RigidBody chest = fullRobotModel.getChest();
      RigidBody head = fullRobotModel.getHead();
      OneDoFJoint[] neckJoints = ScrewTools.createOneDoFJointPath(chest, head);
      double[] neckDesiredJointAccelerations = RandomTools.generateRandomDoubleArray(random, neckJoints.length, 0.1);
      NeckDesiredAccelerationsMessage neckDesiredAccelerationsMessage = new NeckDesiredAccelerationsMessage(NeckControlMode.USER_CONTROL_MODE,
            neckDesiredJointAccelerations);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertEquals(HeadControlMode.TASKSPACE, EndToEndHeadTrajectoryMessageTest.findControllerState(scs));

      drcSimulationTestHelper.send(neckDesiredAccelerationsMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      assertTrue(success);

      assertEquals(HeadControlMode.USER_CONTROL_MODE, EndToEndHeadTrajectoryMessageTest.findControllerState(scs));
      double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(neckJoints, scs);
      assertArrayEquals(neckDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
      double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(neckJoints, scs);
      assertArrayEquals(neckDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

      neckDesiredAccelerationsMessage = new NeckDesiredAccelerationsMessage(NeckControlMode.IHMC_CONTROL_MODE, null);

      drcSimulationTestHelper.send(neckDesiredAccelerationsMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      assertTrue(success);

      assertEquals(HeadControlMode.TASKSPACE, EndToEndHeadTrajectoryMessageTest.findControllerState(scs));

      neckDesiredAccelerationsMessage = new NeckDesiredAccelerationsMessage(NeckControlMode.USER_CONTROL_MODE, neckDesiredJointAccelerations);

      drcSimulationTestHelper.send(neckDesiredAccelerationsMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(HeadUserControlModeState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
      assertTrue(success);

      assertEquals(HeadControlMode.USER_CONTROL_MODE, EndToEndHeadTrajectoryMessageTest.findControllerState(scs));

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.07);
      assertTrue(success);

      assertEquals(HeadControlMode.TASKSPACE, EndToEndHeadTrajectoryMessageTest.findControllerState(scs));
   }

   public static double[] findQPOutputJointAccelerations(OneDoFJoint[] neckJoints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[neckJoints.length];
      for (int i = 0; i < neckJoints.length; i++)
      {
         qdd_ds[i] = scs.getVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + neckJoints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public static double[] findControllerDesiredJointAccelerations(OneDoFJoint[] neckJoints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[neckJoints.length];
      String nameSpace = HeadUserControlModeState.class.getSimpleName();

      for (int i = 0; i < neckJoints.length; i++)
      {
         qdd_ds[i] = scs.getVariable(nameSpace, "qdd_d_user_" + neckJoints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
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
