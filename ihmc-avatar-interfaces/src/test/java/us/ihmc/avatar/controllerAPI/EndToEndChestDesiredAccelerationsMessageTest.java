package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertArrayEquals;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.SpineDesiredAccelerationsMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
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
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableHolder;

public abstract class EndToEndChestDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSimpleCommands() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                         simulationTestingParameters);
      factory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = factory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();
      OneDoFJointBasics[] spineJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, chest);
      double[] chestDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, spineJoints.length, 0.1);
      SpineDesiredAccelerationsMessage desiredAccelerationsMessage = HumanoidMessageTools.createSpineDesiredAccelerationsMessage(chestDesiredJointAccelerations);

      RigidBodyControlMode defaultControlState = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlState == null)
         defaultControlState = RigidBodyControlMode.JOINTSPACE;

      assertEquals(defaultControlState, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), simulationTestHelper.getControllerRegistry()));

      simulationTestHelper.publishToController(desiredAccelerationsMessage);
      success = simulationTestHelper.simulateAndWait(RigidBodyUserControlState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
      assertTrue(success);

      assertEquals(RigidBodyControlMode.USER,
                   EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), simulationTestHelper.getControllerRegistry()));
      double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(spineJoints, simulationTestHelper.getControllerRegistry());
      assertArrayEquals(chestDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
      double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(spineJoints, simulationTestHelper.getControllerRegistry());
      assertArrayEquals(chestDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

      success = simulationTestHelper.simulateAndWait(0.07);
      assertTrue(success);

      assertEquals(defaultControlState, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), simulationTestHelper.getControllerRegistry()));
   }

   public double[] findQPOutputJointAccelerations(OneDoFJointBasics[] joints, YoVariableHolder scs)
   {
      double[] qdd_ds = new double[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         qdd_ds[i] = scs.findVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + joints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public double[] findControllerDesiredJointAccelerations(OneDoFJointBasics[] joints, YoVariableHolder scs)
   {
      double[] qdd_ds = new double[joints.length];
      String prefix = "utorsoUserMode";
      for (int i = 0; i < joints.length; i++)
      {
         String name = prefix + "_" + joints[i].getName() + "_qdd_d";
         qdd_ds[i] = scs.findVariable(name).getValueAsDouble();
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
