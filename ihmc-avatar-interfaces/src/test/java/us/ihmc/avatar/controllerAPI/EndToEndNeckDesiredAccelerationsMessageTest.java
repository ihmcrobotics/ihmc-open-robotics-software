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
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
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
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class EndToEndNeckDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSimpleCommands() throws Exception
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.1);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

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
      simulationTestHelper.publishToController(neckTrajectoryMessage);
      success = simulationTestHelper.simulateNow(0.55);
      assertTrue(success);

      double[] neckDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, neckJoints.length, 0.1);
      NeckDesiredAccelerationsMessage neckDesiredAccelerationsMessage = HumanoidMessageTools.createNeckDesiredAccelerationsMessage(neckDesiredJointAccelerations);

      assertEquals(RigidBodyControlMode.JOINTSPACE, findControllerState(headName, simulationTestHelper));

      simulationTestHelper.publishToController(neckDesiredAccelerationsMessage);
      success = simulationTestHelper.simulateNow(RigidBodyUserControlState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
      assertTrue(success);

      assertEquals(RigidBodyControlMode.USER, findControllerState(headName, simulationTestHelper));
      double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(neckJoints, headName, simulationTestHelper);
      assertArrayEquals(neckDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
      double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(neckJoints, simulationTestHelper);
      assertArrayEquals(neckDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

      success = simulationTestHelper.simulateNow(0.07);
      assertTrue(success);

      assertEquals(RigidBodyControlMode.JOINTSPACE, findControllerState(headName, simulationTestHelper));
   }

   @SuppressWarnings("unchecked")
   public static RigidBodyControlMode findControllerState(String bodyName, YoVariableHolder yoVariableHolder)
   {
      String headOrientatManagerName = bodyName + "Manager";
      String headControlStateName = headOrientatManagerName + "CurrentState";
      return ((YoEnum<RigidBodyControlMode>) yoVariableHolder.findVariable(headOrientatManagerName, headControlStateName)).getEnumValue();
   }

   public static double[] findQPOutputJointAccelerations(OneDoFJointBasics[] neckJoints, YoVariableHolder yoVariableHolder)
   {
      double[] qdd_ds = new double[neckJoints.length];
      for (int i = 0; i < neckJoints.length; i++)
      {
         qdd_ds[i] = yoVariableHolder.findVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + neckJoints[i].getName())
                                     .getValueAsDouble();
      }
      return qdd_ds;
   }

   public static double[] findControllerDesiredJointAccelerations(OneDoFJointBasics[] neckJoints, String bodyName, YoVariableHolder yoVariableHolder)
   {
      double[] qdd_ds = new double[neckJoints.length];
      String namespace = bodyName + "UserControlModule";

      for (int i = 0; i < neckJoints.length; i++)
      {
         String variable = bodyName + "UserMode_" + neckJoints[i].getName() + "_qdd_d";
         qdd_ds[i] = yoVariableHolder.findVariable(namespace, variable).getValueAsDouble();
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
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
