package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.NeckTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

@Tag("controller-api-2")
public abstract class EndToEndNeckTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Test
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 0.5;
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics head = fullRobotModel.getHead();
      OneDoFJointBasics[] neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);
      int numberOfJoints = neckJoints.length;
      double[] desiredJointPositions = new double[numberOfJoints];
      double[] desiredJointVelcoties = new double[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJointBasics joint = neckJoints[i];
         desiredJointPositions[i] = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
      }

      NeckTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createNeckTrajectoryMessage(trajectoryTime, desiredJointPositions);

      if (DEBUG)
      {
         for (int i = 0; i < numberOfJoints; i++)
         {
            OneDoFJointBasics neckJoint = neckJoints[i];
            System.out.println(neckJoint.getName() + ": q = " + neckJoint.getQ());
         }
      }

      drcSimulationTestHelper.publishToController(armTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      assertSingleWaypointExecuted(neckJoints, desiredJointPositions, desiredJointVelcoties, epsilon, scs);
   }

   public static void assertSingleWaypointExecuted(OneDoFJointBasics[] neckJoints, double[] desiredJointPositions, double[] desiredJointVelcoties, double epsilon,
         SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointPositions = findControllerDesiredPositions(neckJoints, scs);
      double[] controllerDesiredJointVelocities = findControllerDesiredVelocities(neckJoints, scs);

      assertArrayEquals(desiredJointPositions, controllerDesiredJointPositions, epsilon);
      assertArrayEquals(desiredJointVelcoties, controllerDesiredJointVelocities, epsilon);

      if (DEBUG)
      {
         for (int i = 0; i < neckJoints.length; i++)
         {
            OneDoFJointBasics joint = neckJoints[i];
            double q_err = desiredJointPositions[i] - joint.getQ();
            System.out.println(joint.getName() + ": q_err = " + q_err + ", controller q_d = " + controllerDesiredJointPositions[i] + ", message q_d = "
                  + desiredJointPositions[i] + ", q = " + joint.getQ());
         }

         for (int i = 0; i < neckJoints.length; i++)
         {
            OneDoFJointBasics joint = neckJoints[i];
            System.out.println(joint.getName() + ": controller qd_d = " + controllerDesiredJointVelocities[i]);
         }
      }
   }

   public static double[] findControllerDesiredPositions(OneDoFJointBasics[] neckJoints, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointPositions = new double[neckJoints.length];
      for (int i = 0; i < neckJoints.length; i++)
      {
         String jointName = neckJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentValue";
         YoDouble q_d = (YoDouble) scs.getVariable(subTrajectoryName, variableName);
         controllerDesiredJointPositions[i] = q_d.getDoubleValue();
      }
      return controllerDesiredJointPositions;
   }

   public static double[] findControllerDesiredVelocities(OneDoFJointBasics[] neckJoints, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointVelocities = new double[neckJoints.length];
      for (int i = 0; i < neckJoints.length; i++)
      {
         String jointName = neckJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentVelocity";
         YoDouble qd_d = (YoDouble) scs.getVariable(subTrajectoryName, variableName);
         controllerDesiredJointVelocities[i] = qd_d.getDoubleValue();
      }
      return controllerDesiredJointVelocities;
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
