package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertArrayEquals;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmDesiredAccelerationsMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableHolder;

public abstract class EndToEndArmDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSimpleCommands() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         String handName = fullRobotModel.getHand(robotSide).getName();

         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         double[] armDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, armJoints.length, 0.1);
         ArmDesiredAccelerationsMessage armDesiredAccelerationsMessage = HumanoidMessageTools.createArmDesiredAccelerationsMessage(robotSide,
                                                                                                                                   armDesiredJointAccelerations);

         assertEquals(RigidBodyControlMode.JOINTSPACE, EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper));
         simulationTestHelper.publishToController(armDesiredAccelerationsMessage);

         success = simulationTestHelper.simulateAndWait(RigidBodyUserControlState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
         assertTrue(success);

         assertEquals(RigidBodyControlMode.USER, EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper));
         double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(hand.getName(), robotSide, armJoints, simulationTestHelper);
         assertArrayEquals(armDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
         double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(armJoints, simulationTestHelper);
         assertArrayEquals(armDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

         success = simulationTestHelper.simulateAndWait(0.07);
         assertTrue(success);

         assertEquals(RigidBodyControlMode.JOINTSPACE, EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper));
      }
   }

   public static double[] findQPOutputJointAccelerations(OneDoFJointBasics[] armJoints, YoVariableHolder scs)
   {
      double[] qdd_ds = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         qdd_ds[i] = scs.findVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + armJoints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public static double[] findControllerDesiredJointAccelerations(String bodyName, RobotSide robotSide, OneDoFJointBasics[] armJoints, YoVariableHolder scs)
   {
      double[] qdd_ds = new double[armJoints.length];
      String namespace = bodyName + "UserControlModule";

      for (int i = 0; i < armJoints.length; i++)
      {
         String variable = bodyName + "UserMode_" + armJoints[i].getName() + "_qdd_d";
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
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
