package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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
   private boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testArmTrajectoryMessageWithSingleWaypoint() throws Exception
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

         String handControlModuleName = robotSide.getCamelCaseNameForStartOfExpression() + HandControlModule.class.getSimpleName();

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         double[] controllerDesiredJointPositions = new double[numberOfJoints];
         double[] controllerDesiredJointVelocities = new double[numberOfJoints];

         for (int i = 0; i < numberOfJoints; i++)
         {
            OneDoFJoint joint = armJoints[i];
            DoubleYoVariable q_d_HCM = (DoubleYoVariable) scs.getVariable(handControlModuleName, "q_d_" + joint.getName() + "HandControlModule");
            DoubleYoVariable qd_d_HCM = (DoubleYoVariable) scs.getVariable(handControlModuleName, "qd_d_" + joint.getName() + "HandControlModule");
            controllerDesiredJointPositions[i] = q_d_HCM.getDoubleValue();
            controllerDesiredJointVelocities[i] = qd_d_HCM.getDoubleValue();
         }

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
