package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertArrayEquals;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmDesiredAccelerationsMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
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
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Tag("controller-api")
public abstract class EndToEndArmDesiredAccelerationsMessageTest implements MultiRobotTestInterface
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
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         String handName = fullRobotModel.getHand(robotSide).getName();

         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         double[] armDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, armJoints.length, 0.1);
         ArmDesiredAccelerationsMessage armDesiredAccelerationsMessage = HumanoidMessageTools.createArmDesiredAccelerationsMessage(robotSide, armDesiredJointAccelerations);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         assertEquals(RigidBodyControlMode.JOINTSPACE, EndToEndTestTools.findRigidBodyControlManagerState(handName, scs));
         drcSimulationTestHelper.publishToController(armDesiredAccelerationsMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(RigidBodyUserControlState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
         assertTrue(success);

         assertEquals(RigidBodyControlMode.USER, EndToEndTestTools.findRigidBodyControlManagerState(handName, scs));
         double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(hand.getName(), robotSide, armJoints, scs);
         assertArrayEquals(armDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
         double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(armJoints, scs);
         assertArrayEquals(armDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.07);
         assertTrue(success);

         assertEquals(RigidBodyControlMode.JOINTSPACE, EndToEndTestTools.findRigidBodyControlManagerState(handName, scs));
      }
   }

   public static double[] findQPOutputJointAccelerations(OneDoFJointBasics[] armJoints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         qdd_ds[i] = scs.getVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + armJoints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public static double[] findControllerDesiredJointAccelerations(String bodyName, RobotSide robotSide, OneDoFJointBasics[] armJoints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[armJoints.length];
      String nameSpace = bodyName + "UserControlModule";

      for (int i = 0; i < armJoints.length; i++)
      {
         String variable = bodyName + "UserMode_" + armJoints[i].getName() + "_qdd_d";
         qdd_ds[i] = scs.getVariable(nameSpace, variable).getValueAsDouble();
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
