package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.robotics.Assert.assertArrayEquals;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.NeckTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class EndToEndNeckTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

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

      simulationTestHelper.publishToController(armTrajectoryMessage);

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      assertSingleWaypointExecuted(neckJoints, desiredJointPositions, desiredJointVelcoties, epsilon, simulationTestHelper);
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(54651);

      YoRegistry testRegistry = new YoRegistry("testStreaming");

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getRootRegistry().addChild(testRegistry);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = simulationTestHelper.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics head = fullRobotModel.getHead();
      OneDoFJointBasics[] neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);

      YoDouble[] initialNeckJointAngles = new YoDouble[neckJoints.length];
      YoDouble[] finalNeckJointAngles = new YoDouble[neckJoints.length];
      YoDouble[] desiredNeckJointAngles = new YoDouble[neckJoints.length];
      YoDouble[] desiredNeckJointVelocities = new YoDouble[neckJoints.length];

      for (int i = 0; i < neckJoints.length; i++)
      {
         OneDoFJointBasics neckJoint = neckJoints[i];
         YoDouble qInitial = new YoDouble("test_q_initial_" + neckJoint.getName(), testRegistry);
         YoDouble qFinal = new YoDouble("test_q_final_" + neckJoint.getName(), testRegistry);
         YoDouble qDesired = new YoDouble("test_q_des_" + neckJoint.getName(), testRegistry);
         YoDouble qDDesired = new YoDouble("test_qd_des_" + neckJoint.getName(), testRegistry);
         qInitial.set(neckJoint.getQ());
         qFinal.set(RandomNumbers.nextDouble(random, neckJoint.getJointLimitLower(), neckJoint.getJointLimitUpper()));
         initialNeckJointAngles[i] = qInitial;
         finalNeckJointAngles[i] = qFinal;
         desiredNeckJointAngles[i] = qDesired;
         desiredNeckJointVelocities[i] = qDDesired;
      }

      simulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         private boolean everyOtherTick = false;

         @Override
         public void doControl()
         {
            everyOtherTick = !everyOtherTick;

            if (!everyOtherTick)
               return;

            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            double[] qDesireds = new double[initialNeckJointAngles.length];
            double[] qDDesireds = new double[initialNeckJointAngles.length];

            for (int i = 0; i < initialNeckJointAngles.length; i++)
            {
               double qDes = EuclidCoreTools.interpolate(initialNeckJointAngles[i].getValue(), finalNeckJointAngles[i].getValue(), alpha);
               double qDDes;
               if (alpha <= 0.0 || alpha >= 1.0)
                  qDDes = 0.0;
               else
                  qDDes = (finalNeckJointAngles[i].getValue() - initialNeckJointAngles[i].getValue()) / trajectoryTime.getValue();
               desiredNeckJointAngles[i].set(qDes);
               desiredNeckJointVelocities[i].set(qDDes);
               qDesireds[i] = qDes;
               qDDesireds[i] = qDDes;
            }

            NeckTrajectoryMessage message = HumanoidMessageTools.createNeckTrajectoryMessage(0.0, qDesireds, qDDesireds, null);
            message.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
            message.getJointspaceTrajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
            simulationTestHelper.publishToController(message);
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return null;
         }

         @Override
         public String getDescription()
         {
            return RobotController.super.getDescription();
         }

         @Override
         public String getName()
         {
            return RobotController.super.getName();
         }
      });

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue());
      assertTrue(success);

      double desiredEpsilon = 5.0e-3;
      double trackingEpsilon = 5.0e-2;

      double[] controllerDesiredPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(neckJoints, simulationTestHelper);
      double[] controllerDesiredVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(neckJoints, simulationTestHelper);

      for (int i = 0; i < neckJoints.length; i++)
      {
         double qDDes = desiredNeckJointVelocities[i].getValue();
         double qDes = desiredNeckJointAngles[i].getValue() - getRobotModel().getControllerDT() * qDDes; // Hack to approx the previous desired. The last computed desired has not been processed yet.

         assertEquals(qDes,
                      controllerDesiredPositions[i],
                      desiredEpsilon,
                      "Desired position mismatch for joint " + neckJoints[i].getName() + " diff: " + Math.abs(qDes - controllerDesiredPositions[i]));
         assertEquals(qDDes,
                      controllerDesiredVelocities[i],
                      desiredEpsilon,
                      "Desired velocity mismatch for joint " + neckJoints[i].getName() + " diff: " + Math.abs(qDDes - controllerDesiredVelocities[i]));
         assertEquals(controllerDesiredPositions[i],
                      neckJoints[i].getQ(),
                      trackingEpsilon,
                      "Poor position tracking for joint " + neckJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredPositions[i] - neckJoints[i].getQ()));
         assertEquals(controllerDesiredVelocities[i],
                      neckJoints[i].getQd(),
                      trackingEpsilon,
                      "Poor velocity tracking for joint " + neckJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredVelocities[i] - neckJoints[i].getQd()));
      }

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue() + 1.5);

      assertTrue(success);

      desiredEpsilon = 1.0e-7;
      trackingEpsilon = 5.0e-3;

      controllerDesiredPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(neckJoints, simulationTestHelper);
      controllerDesiredVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(neckJoints, simulationTestHelper);

      for (int i = 0; i < neckJoints.length; i++)
      {
         double qDes = desiredNeckJointAngles[i].getValue();
         double qDDes = desiredNeckJointVelocities[i].getValue();

         assertEquals(qDes,
                      controllerDesiredPositions[i],
                      desiredEpsilon,
                      "Desired position mismatch for joint " + neckJoints[i].getName() + " diff: " + Math.abs(qDes - controllerDesiredPositions[i]));
         assertEquals(qDDes,
                      controllerDesiredVelocities[i],
                      desiredEpsilon,
                      "Desired velocity mismatch for joint " + neckJoints[i].getName() + " diff: " + Math.abs(qDDes - controllerDesiredVelocities[i]));
         assertEquals(controllerDesiredPositions[i],
                      neckJoints[i].getQ(),
                      trackingEpsilon,
                      "Poor position tracking for joint " + neckJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredPositions[i] - neckJoints[i].getQ()));
         assertEquals(controllerDesiredVelocities[i],
                      neckJoints[i].getQd(),
                      trackingEpsilon,
                      "Poor velocity tracking for joint " + neckJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredVelocities[i] - neckJoints[i].getQd()));
      }
   }

   public static void assertSingleWaypointExecuted(OneDoFJointBasics[] neckJoints,
                                                   double[] desiredJointPositions,
                                                   double[] desiredJointVelcoties,
                                                   double epsilon,
                                                   YoVariableHolder yoVariableHolder)
   {
      double[] controllerDesiredJointPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(neckJoints, yoVariableHolder);
      double[] controllerDesiredJointVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(neckJoints, yoVariableHolder);

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
