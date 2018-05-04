package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.ArmDesiredAccelerationsMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.HandUserControlModeState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndArmDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

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
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         String handName = fullRobotModel.getHand(robotSide).getName();

         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         double[] armDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, armJoints.length, 0.1);
         ArmDesiredAccelerationsMessage armDesiredAccelerationsMessage = HumanoidMessageTools.createArmDesiredAccelerationsMessage(robotSide, armDesiredJointAccelerations);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         assertEquals(RigidBodyControlMode.JOINTSPACE, EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs));
         drcSimulationTestHelper.send(armDesiredAccelerationsMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(HandUserControlModeState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
         assertTrue(success);

         assertEquals(RigidBodyControlMode.USER, EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs));
         double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(hand.getName(), robotSide, armJoints, scs);
         assertArrayEquals(armDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
         double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(armJoints, scs);
         assertArrayEquals(armDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.07);
         assertTrue(success);

         assertEquals(RigidBodyControlMode.JOINTSPACE, EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs));
      }
   }

   public static double[] findQPOutputJointAccelerations(OneDoFJoint[] armJoints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         qdd_ds[i] = scs.getVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + armJoints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public static double[] findControllerDesiredJointAccelerations(String bodyName, RobotSide robotSide, OneDoFJoint[] armJoints, SimulationConstructionSet scs)
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
