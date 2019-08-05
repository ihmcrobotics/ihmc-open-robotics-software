package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.robotics.Assert.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

@Tag("controller-api-2")
public abstract class EndToEndSpineJointTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final double DESIRED_EPSILON = 1.0E-10;
   private static final double DESIRED_QUAT_EPSILON = 0.01;
   private static final double MAX_SPEED_FOR_CONTINOUS = 10.0;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private final Random random = new Random(1991L);
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RigidBodyBasics pelvis;
   private RigidBodyBasics chest;
   private OneDoFJointBasics[] spineJoints;
   private int numberOfJoints;
   private ControllerSpy controllerSpy;

   /**
    * This tests the execution of a single spine waypoint.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double trajectoryTime = 1.0;
      SpineTrajectoryMessage message = createRandomSpineMessage(trajectoryTime, random);

      executeMessage(message);

      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   /**
    * This tests that the switching between jointspace and taskspace control modes works properly. It
    * does not test that the desired joint positions are continuous over the state switches anymore.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testSwitchingBetweenControlModes() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double trajectoryTime = 1.0;
      SpineTrajectoryMessage message1 = createRandomSpineMessage(trajectoryTime, random);
      ChestTrajectoryMessage message2 = createRandomChestMessage(trajectoryTime, random);
      SpineTrajectoryMessage message3 = createRandomSpineMessage(trajectoryTime, random);

      executeMessage(message1);
      executeMessage(message2, chest);
      executeMessage(message3);

      assertControlWasConsistent(controllerSpy);
   }

   /**
    * This tests that the joint desireds are continuous when sending multiple joint space messages.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testDesiredsAreContinuous() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double trajectoryTime = 1.0;
      for (int i = 0; i < 10; i++)
      {
         SpineTrajectoryMessage message = createRandomSpineMessage(trajectoryTime, random);
         executeMessage(message);
      }

      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   /**
    * This tests a trajectory with multiple waypoints. This will execute a spine yaw sine wave.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testMultipleWaypoints() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double amplitude = 0.2;
      double frequency = 0.25;
      double totalTime = 10.0;
      int waypoints = 20;

      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         message.getJointspaceTrajectory().getJointTrajectoryMessages().add();

      for (int waypoint = 0; waypoint < waypoints; waypoint++)
      {
         double timeAtWaypoint = totalTime * waypoint / waypoints;
         double desiredPosition = amplitude * Math.sin(2.0 * Math.PI * frequency * timeAtWaypoint);
         double desiredVelocity = 2.0 * Math.PI * frequency * amplitude * Math.cos(2.0 * Math.PI * frequency * timeAtWaypoint);

         if (waypoint == 0 || waypoint == waypoints - 1)
            desiredVelocity = 0.0;

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            OneDoFJointTrajectoryMessage jointTrajectoryMessage = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx);
            if (jointIdx == 0)
               jointTrajectoryMessage.getTrajectoryPoints().add()
                                     .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(timeAtWaypoint, desiredPosition, desiredVelocity));
            else
               jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(timeAtWaypoint, 0.0, 0.0));
         }
      }

      executeMessage(message);

      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   /**
    * This tests a trajectory a lot of waypoints. The message does not do anything except testing that
    * the controller does not blow up.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testLongMessage() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      int waypoints = 100;
      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = message.getJointspaceTrajectory().getJointTrajectoryMessages().add();

         for (int waypoint = 0; waypoint < waypoints; waypoint++)
         {
            jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(0.1 * waypoint, 0.0, 0.0));
         }
      }
      executeMessage(message);

      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   /**
    * Tests that messages queue properly and the body manager has the correct number of waypoints after
    * queuing.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testMessageQueuing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      int numberOfMessages = 10;
      int numberOfPoints = 5;

      // same wave for all back joints
      double amplitude = 0.1;
      double frequency = 0.25;

      double timePerWaypoint = 0.05;
      double totalTime = timePerWaypoint;

      // create messages
      SpineTrajectoryMessage[] messages = new SpineTrajectoryMessage[numberOfMessages];
      for (int msgIdx = 0; msgIdx < numberOfMessages; msgIdx++)
      {
         SpineTrajectoryMessage message = new SpineTrajectoryMessage();
         double timeInMessage = timePerWaypoint;

         if (msgIdx == 0)
         {
            // To make sure an initial point at the current desired is queued offset the very first trajectory point.
            timeInMessage += RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
            totalTime += RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         }

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            message.getJointspaceTrajectory().getJointTrajectoryMessages().add();

         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            double desiredPosition = amplitude * Math.sin(2.0 * Math.PI * frequency * totalTime);
            double desiredVelocity = 2.0 * Math.PI * frequency * amplitude * Math.cos(2.0 * Math.PI * frequency * totalTime);

            if (msgIdx == 0 && pointIdx == 0)
               desiredVelocity = 0.0;
            if (msgIdx == numberOfMessages - 1 && pointIdx == numberOfPoints - 1)
               desiredVelocity = 0.0;

            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            {
               message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx).getTrajectoryPoints().add()
                      .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(timeInMessage, desiredPosition, desiredVelocity));
            }

            totalTime += timePerWaypoint;
            timeInMessage += timePerWaypoint;
         }

         message.getJointspaceTrajectory().getQueueingProperties().setMessageId(msgIdx + 1);
         if (msgIdx != 0)
         {
            message.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            message.getJointspaceTrajectory().getQueueingProperties().setPreviousMessageId(msgIdx);
         }

         messages[msgIdx] = message;
      }

      // send messages
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      double controllerDT = getRobotModel().getControllerDT();
      for (int msgIdx = 0; msgIdx < numberOfMessages; msgIdx++)
      {
         drcSimulationTestHelper.publishToController(messages[msgIdx]);
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * controllerDT);

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager((msgIdx + 1) * numberOfPoints + 1, chest.getName(),
                                                                              spineJoints[jointIdx].getName(), scs);
      }

      int expectedPointsInGenerator = Math.min(numberOfPoints + 1, RigidBodyJointspaceControlState.maxPointsInGenerator);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         EndToEndTestTools.assertNumberOfWaypointsInJointspaceManagerGenerator(expectedPointsInGenerator, chest.getName(), spineJoints[jointIdx].getName(),
                                                                               scs);

      int expectedPointsInQueue = numberOfMessages * numberOfPoints - expectedPointsInGenerator + 1;
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         EndToEndTestTools.assertNumberOfWaypointsInJointspaceManagerQueue(expectedPointsInQueue, chest.getName(), spineJoints[jointIdx].getName(), scs);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(totalTime + 1.0);
      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   /**
    * Tests that messages queue properly and the body manager has the correct number of waypoints after
    * queuing.
    * 
    * @throws SimulationExceededMaximumTimeException
    */
   @Test
   public void testMessageWithDifferentTrajectoryLengthsPerJoint() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      Random random = new Random(845278L);
      double maxTime = 5.0;

      int[] numberOfPoints = new int[numberOfJoints];
      double[] trajectoryTime = new double[numberOfJoints];
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         numberOfPoints[jointIdx] = random.nextInt(10);
         trajectoryTime[jointIdx] = random.nextDouble() * maxTime;
      }

      SpineTrajectoryMessage message = new SpineTrajectoryMessage();
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         int numberOfPoinsForJoint = numberOfPoints[jointIdx];
         double timePerPoint = trajectoryTime[jointIdx] / numberOfPoinsForJoint;
         double time = timePerPoint;

         OneDoFJointTrajectoryMessage jointTrajectoryMessage = new OneDoFJointTrajectoryMessage();
         for (int pointIdx = 0; pointIdx < numberOfPoinsForJoint; pointIdx++)
         {
            double position = getRandomJointAngleInRange(random, spineJoints[jointIdx]);
            jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(time, position, 0.0));
            time += timePerPoint;
         }
         message.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(jointTrajectoryMessage);
      }

      // send message
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.publishToController(message);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * controllerDT);

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = spineJoints[jointIdx];
         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(numberOfPoints[jointIdx] + 1, chest.getName(), joint.getName(), scs);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(maxTime);

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         int maxPointsInGenerator = RigidBodyJointspaceControlState.maxPointsInGenerator;
         int totalPointsForJoint = numberOfPoints[jointIdx] + 1;

         if (totalPointsForJoint <= maxPointsInGenerator)
         {
            EndToEndTestTools.assertNumberOfWaypointsInJointspaceManagerGenerator(totalPointsForJoint, chest.getName(), spineJoints[jointIdx].getName(), scs);
         }
         else
         {
            int pointsInLastTrajectory = totalPointsForJoint - maxPointsInGenerator; // fist set in generator
            while (pointsInLastTrajectory > (maxPointsInGenerator - 1))
               pointsInLastTrajectory -= (maxPointsInGenerator - 1); // keep filling the generator
            pointsInLastTrajectory++;
            EndToEndTestTools.assertNumberOfWaypointsInJointspaceManagerGenerator(pointsInLastTrajectory, chest.getName(), spineJoints[jointIdx].getName(),
                                                                                  scs);
         }
      }

      assertDesiredsMatchAfterExecution(message, spineJoints, scs);
      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(54651);

      YoVariableRegistry testRegistry = new YoVariableRegistry("testStreaming");

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoVariableRegistry(testRegistry);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox()
                                               .getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();
      OneDoFJointBasics[] spineJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, chest);

      YoDouble[] initialSpineJointAngles = new YoDouble[spineJoints.length];
      YoDouble[] finalSpineJointAngles = new YoDouble[spineJoints.length];
      YoDouble[] desiredSpineJointAngles = new YoDouble[spineJoints.length];
      YoDouble[] desiredSpineJointVelocities = new YoDouble[spineJoints.length];

      for (int i = 0; i < spineJoints.length; i++)
      {
         OneDoFJointBasics spineJoint = spineJoints[i];
         YoDouble qInitial = new YoDouble("test_q_initial_" + spineJoint.getName(), testRegistry);
         YoDouble qFinal = new YoDouble("test_q_final_" + spineJoint.getName(), testRegistry);
         YoDouble qDesired = new YoDouble("test_q_des_" + spineJoint.getName(), testRegistry);
         YoDouble qDDesired = new YoDouble("test_qd_des_" + spineJoint.getName(), testRegistry);
         qInitial.set(spineJoint.getQ());
         qFinal.set(RandomNumbers.nextDouble(random, spineJoint.getJointLimitLower(), spineJoint.getJointLimitUpper()));
         initialSpineJointAngles[i] = qInitial;
         finalSpineJointAngles[i] = qFinal;
         desiredSpineJointAngles[i] = qDesired;
         desiredSpineJointVelocities[i] = qDDesired;
      }

      drcSimulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         @Override
         public void doControl()
         {
            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            double[] qDesireds = new double[initialSpineJointAngles.length];
            double[] qDDesireds = new double[initialSpineJointAngles.length];

            for (int i = 0; i < initialSpineJointAngles.length; i++)
            {
               double qDes = EuclidCoreTools.interpolate(initialSpineJointAngles[i].getValue(), finalSpineJointAngles[i].getValue(), alpha);
               double qDDes;
               if (alpha <= 0.0 || alpha >= 1.0)
                  qDDes = 0.0;
               else
                  qDDes = (finalSpineJointAngles[i].getValue() - initialSpineJointAngles[i].getValue()) / trajectoryTime.getValue();
               desiredSpineJointAngles[i].set(qDes);
               desiredSpineJointVelocities[i].set(qDDes);
               qDesireds[i] = qDes;
               qDDesireds[i] = qDDes;
            }

            drcSimulationTestHelper.publishToController(HumanoidMessageTools.createSpineTrajectoryMessage(0.0, qDesireds, qDDesireds, null));
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
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

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 * trajectoryTime.getValue());
      assertTrue(success);

      double desiredEpsilon = 5.0e-3;
      double trackingEpsilon = 5.0e-2;

      double[] controllerDesiredPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(spineJoints, scs);
      double[] controllerDesiredVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(spineJoints, scs);

      for (int i = 0; i < spineJoints.length; i++)
      {
         double qDDes = desiredSpineJointVelocities[i].getValue();
         double qDes = desiredSpineJointAngles[i].getValue() - getRobotModel().getControllerDT() * qDDes; // Hack to approx the previous desired. The last computed desired has not been processed yet.

         assertEquals(qDes,
                      controllerDesiredPositions[i],
                      desiredEpsilon,
                      "Desired position mismatch for joint " + spineJoints[i].getName() + " diff: " + Math.abs(qDes - controllerDesiredPositions[i]));
         assertEquals(qDDes,
                      controllerDesiredVelocities[i],
                      desiredEpsilon,
                      "Desired velocity mismatch for joint " + spineJoints[i].getName() + " diff: " + Math.abs(qDDes - controllerDesiredVelocities[i]));
         assertEquals(controllerDesiredPositions[i],
                      spineJoints[i].getQ(),
                      trackingEpsilon,
                      "Poor position tracking for joint " + spineJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredPositions[i] - spineJoints[i].getQ()));
         assertEquals(controllerDesiredVelocities[i],
                      spineJoints[i].getQd(),
                      trackingEpsilon,
                      "Poor velocity tracking for joint " + spineJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredVelocities[i] - spineJoints[i].getQd()));
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 * trajectoryTime.getValue() + 1.5);

      assertTrue(success);

      desiredEpsilon = 1.0e-7;
      trackingEpsilon = 5.0e-3;

      controllerDesiredPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(spineJoints, scs);
      controllerDesiredVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(spineJoints, scs);

      for (int i = 0; i < spineJoints.length; i++)
      {
         double qDes = desiredSpineJointAngles[i].getValue();
         double qDDes = desiredSpineJointVelocities[i].getValue();

         assertEquals(qDes,
                      controllerDesiredPositions[i],
                      desiredEpsilon,
                      "Desired position mismatch for joint " + spineJoints[i].getName() + " diff: " + Math.abs(qDes - controllerDesiredPositions[i]));
         assertEquals(qDDes,
                      controllerDesiredVelocities[i],
                      desiredEpsilon,
                      "Desired velocity mismatch for joint " + spineJoints[i].getName() + " diff: " + Math.abs(qDDes - controllerDesiredVelocities[i]));
         assertEquals(controllerDesiredPositions[i],
                      spineJoints[i].getQ(),
                      trackingEpsilon,
                      "Poor position tracking for joint " + spineJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredPositions[i] - spineJoints[i].getQ()));
         assertEquals(controllerDesiredVelocities[i],
                      spineJoints[i].getQd(),
                      trackingEpsilon,
                      "Poor velocity tracking for joint " + spineJoints[i].getName() + " err: "
                            + Math.abs(controllerDesiredVelocities[i] - spineJoints[i].getQd()));
      }
   }

   private SpineTrajectoryMessage createRandomSpineMessage(double trajectoryTime, Random random)
   {
      double[] jointDesireds = new double[numberOfJoints];
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointBasics joint = spineJoints[jointIdx];
         double desired = getRandomJointAngleInRange(random, joint);
         jointDesireds[jointIdx] = desired;
      }
      return HumanoidMessageTools.createSpineTrajectoryMessage(trajectoryTime, jointDesireds);
   }

   private double getRandomJointAngleInRange(Random random, OneDoFJointBasics joint)
   {
      double jointLimitUpper = joint.getJointLimitUpper();
      double jointLimitLower = joint.getJointLimitLower();
      return RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper);
   }

   private ChestTrajectoryMessage createRandomChestMessage(double trajectoryTime, Random random)
   {
      OneDoFJointBasics[] spineClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);
      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);
      RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      return HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(),
                                                               ReferenceFrame.getWorldFrame());
   }

   private static void assertControlWasConsistent(ControllerSpy controllerSpy)
   {
      assertFalse("Joint and Taskspace control was inconsistent.", controllerSpy.wasControlInconsistent());
   }

   private static void assertDesiredsContinous(ControllerSpy controllerSpy)
   {
      double maxSpeed = controllerSpy.getMaxSpeed();
      String errorMessage = "The maximum speed along the trajectory was " + maxSpeed + " this was probably caused by a discontinous desired value.";
      assertTrue(errorMessage, maxSpeed < MAX_SPEED_FOR_CONTINOUS);
   }

   private void executeMessage(SpineTrajectoryMessage message) throws SimulationExceededMaximumTimeException
   {
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.publishToController(message);

      double trajectoryTime = 0.0;
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointTrajectoryMessage jointTrajectory = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx);
         double jointTrajectoryTime = jointTrajectory.getTrajectoryPoints().getLast().getTime();
         if (jointTrajectoryTime > trajectoryTime)
            trajectoryTime = jointTrajectoryTime;
      }
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 5.0 * controllerDT));
      assertDesiredsMatchAfterExecution(message, spineJoints, drcSimulationTestHelper.getSimulationConstructionSet());
   }

   private static void assertDesiredsMatchAfterExecution(SpineTrajectoryMessage message, OneDoFJointBasics[] spineJoints, SimulationConstructionSet scs)
   {
      for (int jointIdx = 0; jointIdx < spineJoints.length; jointIdx++)
      {
         OneDoFJointTrajectoryMessage jointTrajectory = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx);
         double desired = jointTrajectory.getTrajectoryPoints().getLast().getPosition();
         OneDoFJointBasics joint = spineJoints[jointIdx];
         EndToEndTestTools.assertOneDoFJointFeedbackControllerDesiredPosition(joint.getName(), desired, DESIRED_EPSILON, scs);
      }
   }

   private void executeMessage(ChestTrajectoryMessage message, RigidBodyBasics chest) throws SimulationExceededMaximumTimeException
   {
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.publishToController(message);

      double trajectoryTime = message.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime();
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 5.0 * controllerDT));

      Quaternion desired = new Quaternion(message.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getOrientation());
      assertChestDesired(drcSimulationTestHelper.getSimulationConstructionSet(), desired, chest);
   }

   private static void assertChestDesired(SimulationConstructionSet scs, Quaternion desired, RigidBodyBasics chest)
   {
      QuaternionReadOnly controllerDesired = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs);
      EuclidCoreTestTools.assertQuaternionEquals(desired, controllerDesired, DESIRED_QUAT_EPSILON);
   }

   private static YoBoolean findOrientationControlEnabled(SimulationConstructionSet scs, RigidBodyBasics body)
   {
      String bodyName = body.getName();
      String namespace = bodyName + "OrientationFBController";
      String variable = bodyName + "IsOrientationFBControllerEnabled";
      return EndToEndTestTools.findYoBoolean(namespace, variable, scs);
   }

   private static YoBoolean findJointControlEnabled(SimulationConstructionSet scs, OneDoFJointBasics joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "control_enabled_" + jointName;
      return EndToEndTestTools.findYoBoolean(namespace, variable, scs);
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      ThreadTools.sleep(1000);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      chest = fullRobotModel.getChest();
      spineJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, chest);
      numberOfJoints = spineJoints.length;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      controllerSpy = new ControllerSpy(spineJoints, scs, getRobotModel().getControllerDT());
      drcSimulationTestHelper.addRobotControllerOnControllerThread(controllerSpy);
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

   private class ControllerSpy extends SimpleRobotController
   {
      private final double controllerDT;
      private final OneDoFJointBasics[] spineJoints;
      private final OneDoFJointBasics[] spineJointClones;
      private final RigidBodyBasics chestClone;

      private final Map<OneDoFJointBasics, YoBoolean> jointControlEnabled = new HashMap<>();
      private final Map<OneDoFJointBasics, YoDouble> jointDesiredsMap = new HashMap<>();

      private final YoBoolean orientationControlEnabled;
      private final QuaternionReadOnly desiredOrientation;

      private final YoFrameQuaternion currentDesiredOrientation = new YoFrameQuaternion("CurrentDesired", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameQuaternion previousDesiredOrientation = new YoFrameQuaternion("PreviousDesired", ReferenceFrame.getWorldFrame(), registry);

      private final YoBoolean inconsistentControl = new YoBoolean("InconsistentControl", registry);
      private final YoDouble maxSpeed = new YoDouble("maxSpeed", registry);

      private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

      public ControllerSpy(OneDoFJointBasics[] spineJoints, SimulationConstructionSet scs, double controllerDT)
      {
         this.spineJoints = spineJoints;
         this.controllerDT = controllerDT;
         spineJointClones = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(spineJoints);
         chestClone = spineJointClones[spineJointClones.length - 1].getSuccessor();

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            OneDoFJointBasics joint = spineJoints[jointIdx];
            jointDesiredsMap.put(joint, EndToEndTestTools.findOneDoFJointFeedbackControllerDesiredPosition(joint.getName(), scs));
            jointControlEnabled.put(joint, findJointControlEnabled(scs, joint));
         }
         orientationControlEnabled = findOrientationControlEnabled(scs, chest);
         desiredOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs);
         inconsistentControl.set(false);
         maxSpeed.set(0.0);
      }

      @Override
      public void doControl()
      {
         if (spineJoints.length == 0)
            return;

         boolean jointControl = jointControlEnabled.get(spineJoints[0]).getBooleanValue();
         for (int jointIdx = 1; jointIdx < numberOfJoints; jointIdx++)
         {
            boolean thisJointControl = jointControlEnabled.get(spineJoints[jointIdx]).getBooleanValue();
            if (thisJointControl != jointControl)
               inconsistentControl.set(true);
         }
         if (jointControl && orientationControlEnabled.getBooleanValue())
            inconsistentControl.set(true);
         if (!jointControl && !orientationControlEnabled.getBooleanValue())
            inconsistentControl.set(true);

         if (jointControl)
         {
            DenseMatrix64F jointPositions = new DenseMatrix64F(spineJoints.length, 1);
            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
               jointPositions.set(jointIdx, jointDesiredsMap.get(spineJoints[jointIdx]).getDoubleValue());
            MultiBodySystemTools.insertJointsState(spineJointClones, JointStateType.CONFIGURATION, jointPositions);
            FrameQuaternion chestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
            chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
            currentDesiredOrientation.set(chestOrientation);
         }
         else
         {
            currentDesiredOrientation.set(desiredOrientation);
         }

         if (!currentDesiredOrientation.containsNaN() && !previousDesiredOrientation.containsNaN())
         {
            Vector4D derivative = new Vector4D();
            quaternionCalculus.computeQDotByFiniteDifferenceCentral(previousDesiredOrientation, currentDesiredOrientation, controllerDT, derivative);
            Vector3D angularVelocity = new Vector3D();
            quaternionCalculus.computeAngularVelocityInWorldFrame(currentDesiredOrientation, derivative, angularVelocity);
            double speed = angularVelocity.length();
            if (speed > maxSpeed.getDoubleValue())
               maxSpeed.set(speed);
         }

         previousDesiredOrientation.set(currentDesiredOrientation);
      }

      public double getMaxSpeed()
      {
         return maxSpeed.getDoubleValue();
      }

      public boolean wasControlInconsistent()
      {
         return inconsistentControl.getBooleanValue();
      }
   }

}
