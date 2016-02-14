package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndArmTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage();

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), "", selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 0.5;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);
         double[] desiredJointPositions = new double[numberOfJoints];
         double[] desiredJointVelcoties = new double[numberOfJoints];

         for (int i = 0; i < numberOfJoints; i++)
         {
            OneDoFJoint joint = armJoints[i];
            desiredJointPositions[i] = RandomTools.generateRandomDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
         }

         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);

         if (DEBUG)
         {
            for (int i = 0; i < numberOfJoints; i++)
            {
               OneDoFJoint armJoint = armJoints[i];
               System.out.println(armJoint.getName() + ": q = " + armJoint.getQ());
            }
         }

         drcSimulationTestHelper.send(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         assertSingleWaypointExecuted(robotSide, armJoints, numberOfJoints, desiredJointPositions, desiredJointVelcoties, epsilon, scs);
      }
   }

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage();

      Random random = new Random(564654L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), "", selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 5.0;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);
         double[] desiredJointPositions = new double[numberOfJoints];

         for (int i = 0; i < numberOfJoints; i++)
         {
            OneDoFJoint joint = armJoints[i];
            desiredJointPositions[i] = RandomTools.generateRandomDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
         }

         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);

         drcSimulationTestHelper.send(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         double timeStopSent = scs.getRobots()[0].getYoTime().getDoubleValue();
         double[] actualJointPositions = new double[numberOfJoints];
         double[] zeroVelocities = new double[numberOfJoints];
         for (int i = 0; i < numberOfJoints; i++)
         {
            actualJointPositions[i] = armJoints[i].getQ();
         }
         
         drcSimulationTestHelper.send(new StopAllTrajectoryMessage());

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
         assertTrue(success);


         HandControlState controllerState = findControllerState(robotSide, scs);
         double switchTime = findControllerSwitchTime(robotSide, scs);
         double[] controllerDesiredJointPositions = findControllerDesiredPositions(robotSide, armJoints, numberOfJoints, scs);
         double[] controllerDesiredJointVelocities = findControllerDesiredVelocities(robotSide, armJoints, numberOfJoints, scs);

         assertEquals(HandControlState.JOINT_SPACE, controllerState);
         assertEquals(timeStopSent, switchTime, getRobotModel().getControllerDT());
         assertArrayEquals(actualJointPositions, controllerDesiredJointPositions, 0.01);
         assertArrayEquals(zeroVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public static void assertSingleWaypointExecuted(RobotSide robotSide, OneDoFJoint[] armJoints, int numberOfJoints, double[] desiredJointPositions,
         double[] desiredJointVelcoties, double epsilon, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointPositions = findControllerDesiredPositions(robotSide, armJoints, numberOfJoints, scs);
      double[] controllerDesiredJointVelocities = findControllerDesiredVelocities(robotSide, armJoints, numberOfJoints, scs);

      assertArrayEquals(desiredJointPositions, controllerDesiredJointPositions, epsilon);
      assertArrayEquals(desiredJointVelcoties, controllerDesiredJointVelocities, epsilon);

      if (DEBUG)
      {
         for (int i = 0; i < numberOfJoints; i++)
         {
            OneDoFJoint joint = armJoints[i];
            double q_err = desiredJointPositions[i] - joint.getQ();
            System.out.println(joint.getName() + ": q_err = " + q_err + ", controller q_d = " + controllerDesiredJointPositions[i] + ", message q_d = "
                  + desiredJointPositions[i] + ", q = " + joint.getQ());
         }

         for (int i = 0; i < numberOfJoints; i++)
         {
            OneDoFJoint joint = armJoints[i];
            System.out.println(joint.getName() + ": controller qd_d = " + controllerDesiredJointVelocities[i]);
         }
      }
   }

   @SuppressWarnings("unchecked")
   public static HandControlState findControllerState(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String handControlModuleName = robotSide.getCamelCaseNameForStartOfExpression() + HandControlModule.class.getSimpleName();
      return ((EnumYoVariable<HandControlState>) scs.getVariable(handControlModuleName, handControlModuleName)).getEnumValue();
   }

   public static double findControllerSwitchTime(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String handControlModuleName = robotSide.getCamelCaseNameForStartOfExpression() + HandControlModule.class.getSimpleName();
      return scs.getVariable(handControlModuleName, handControlModuleName + "SwitchTime").getValueAsDouble();
   }

   public static double[] findControllerDesiredPositions(RobotSide robotSide, OneDoFJoint[] armJoints, int numberOfJoints, SimulationConstructionSet scs)
   {
      String handControlModuleName = robotSide.getCamelCaseNameForStartOfExpression() + HandControlModule.class.getSimpleName();

      double[] controllerDesiredJointPositions = new double[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJoint joint = armJoints[i];
         DoubleYoVariable q_d_HCM = (DoubleYoVariable) scs.getVariable(handControlModuleName, "q_d_" + joint.getName() + HandControlModule.class.getSimpleName());
         controllerDesiredJointPositions[i] = q_d_HCM.getDoubleValue();
      }
      return controllerDesiredJointPositions;
   }

   public static double[] findControllerDesiredVelocities(RobotSide robotSide, OneDoFJoint[] armJoints, int numberOfJoints, SimulationConstructionSet scs)
   {
      String handControlModuleName = robotSide.getCamelCaseNameForStartOfExpression() + HandControlModule.class.getSimpleName();

      double[] controllerDesiredJointVelocities = new double[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJoint joint = armJoints[i];
         DoubleYoVariable qd_d_HCM = (DoubleYoVariable) scs.getVariable(handControlModuleName, "qd_d_" + joint.getName() + HandControlModule.class.getSimpleName());
         controllerDesiredJointVelocities[i] = qd_d_HCM.getDoubleValue();
      }
      return controllerDesiredJointVelocities;
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
