package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertArrayEquals;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.NeckDesiredAccelerationsMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyUserControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class EndToEndNeckDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Test
   public void testSimpleCommands() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics head = fullRobotModel.getHead();
      String headName = head.getName();
      OneDoFJointBasics[] neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);

      // move joints to mid range
      double[] desiredJointPositions = new double[neckJoints.length];
      double[] desiredJointVelcoties = new double[neckJoints.length];
      for (int i = 0; i < neckJoints.length; i++)
      {
         OneDoFJointBasics joint = neckJoints[i];
         desiredJointPositions[i] = (joint.getJointLimitLower() + joint.getJointLimitUpper()) / 2.0;
         desiredJointVelcoties[i] = 0.0;
      }
      NeckTrajectoryMessage neckTrajectoryMessage = HumanoidMessageTools.createNeckTrajectoryMessage(0.5, desiredJointPositions);
      drcSimulationTestHelper.publishToController(neckTrajectoryMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.55);
      assertTrue(success);

      double[] neckDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, neckJoints.length, 0.1);
      NeckDesiredAccelerationsMessage neckDesiredAccelerationsMessage = HumanoidMessageTools.createNeckDesiredAccelerationsMessage(neckDesiredJointAccelerations);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertEquals(RigidBodyControlMode.JOINTSPACE, findControllerState(headName, scs));

      drcSimulationTestHelper.publishToController(neckDesiredAccelerationsMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(RigidBodyUserControlState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
      assertTrue(success);

      assertEquals(RigidBodyControlMode.USER, findControllerState(headName, scs));
      double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(neckJoints, headName, scs);
      assertArrayEquals(neckDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
      double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(neckJoints, scs);
      assertArrayEquals(neckDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.07);
      assertTrue(success);

      assertEquals(RigidBodyControlMode.JOINTSPACE, findControllerState(headName, scs));
   }

   @SuppressWarnings("unchecked")
   public static RigidBodyControlMode findControllerState(String bodyName, SimulationConstructionSet scs)
   {
      String headOrientatManagerName = bodyName + "Manager";
      String headControlStateName = headOrientatManagerName + "CurrentState";
      return ((YoEnum<RigidBodyControlMode>) scs.findVariable(headOrientatManagerName, headControlStateName)).getEnumValue();
   }

   public static double[] findQPOutputJointAccelerations(OneDoFJointBasics[] neckJoints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[neckJoints.length];
      for (int i = 0; i < neckJoints.length; i++)
      {
         qdd_ds[i] = scs.findVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + neckJoints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public static double[] findControllerDesiredJointAccelerations(OneDoFJointBasics[] neckJoints, String bodyName, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[neckJoints.length];
      String namespace = bodyName + "UserControlModule";

      for (int i = 0; i < neckJoints.length; i++)
      {
         String variable = bodyName + "UserMode_" + neckJoints[i].getName() + "_qdd_d";
         qdd_ds[i] = scs.findVariable(namespace, variable).getValueAsDouble();
      }
      return qdd_ds;
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
}
