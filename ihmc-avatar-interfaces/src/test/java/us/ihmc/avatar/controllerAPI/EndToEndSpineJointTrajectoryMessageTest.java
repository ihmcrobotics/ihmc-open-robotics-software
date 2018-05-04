package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class EndToEndSpineJointTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final double DESIRED_EPSILON = 1.0E-10;
   private static final double DESIRED_QUAT_EPSILON = 0.01;
   private static final double MAX_SPEED_FOR_CONTINOUS = 10.0;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private final Random random = new Random(1991L);
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RigidBody pelvis;
   private RigidBody chest;
   private OneDoFJoint[] spineJoints;
   private int numberOfJoints;
   private ControllerSpy controllerSpy;

   /**
    * This tests the execution of a single spine waypoint.
    * @throws SimulationExceededMaximumTimeException
    */
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
    * This tests that the switching between jointspace and taskspace control modes works properly.
    * It does not test that the desired joint positions are continuous over the state switches anymore.
    * @throws SimulationExceededMaximumTimeException
    */
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
    * @throws SimulationExceededMaximumTimeException
    */
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
    * @throws SimulationExceededMaximumTimeException
    */
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
               jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(timeAtWaypoint, desiredPosition, desiredVelocity));
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
    * @throws SimulationExceededMaximumTimeException
    */
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
    * @throws SimulationExceededMaximumTimeException
    */
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
                                                                                                  .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(timeInMessage,
                                                                                                                                                           desiredPosition,
                                                                                                                                                           desiredVelocity));
            }

            totalTime += timePerWaypoint;
            timeInMessage += timePerWaypoint;
         }

         message.getJointspaceTrajectory().getQueueingProperties().setMessageId(msgIdx + 1);
         if (msgIdx != 0)
         {
            message.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            message.getJointspaceTrajectory().getQueueingProperties().setPreviousMessageId((long) msgIdx);
         }

         messages[msgIdx] = message;
      }

      // send messages
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      double controllerDT = getRobotModel().getControllerDT();
      for (int msgIdx = 0; msgIdx < numberOfMessages; msgIdx++)
      {
         drcSimulationTestHelper.send(messages[msgIdx]);
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * controllerDT);

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            assertNumberOfTrajectoryPoints((msgIdx + 1) * numberOfPoints + 1, spineJoints[jointIdx], scs);
      }

      int expectedPointsInGenerator = Math.min(numberOfPoints + 1, RigidBodyJointspaceControlState.maxPointsInGenerator);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         assertNumberOfTrajectoryPointsInGenerator(expectedPointsInGenerator, spineJoints[jointIdx], scs);

      int expectedPointsInQueue = numberOfMessages * numberOfPoints - expectedPointsInGenerator + 1;
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         assertNumberOfTrajectoryPointsInQueue(expectedPointsInQueue, spineJoints[jointIdx], scs);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(totalTime + 1.0);
      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   /**
    * Tests that messages queue properly and the body manager has the correct number of waypoints after
    * queuing.
    * @throws SimulationExceededMaximumTimeException
    */
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
      drcSimulationTestHelper.send(message);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * controllerDT);

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = spineJoints[jointIdx];
         assertNumberOfTrajectoryPoints(numberOfPoints[jointIdx] + 1, joint, scs);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(maxTime);

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         int maxPointsInGenerator = RigidBodyJointspaceControlState.maxPointsInGenerator;
         int totalPointsForJoint = numberOfPoints[jointIdx] + 1;

         if (totalPointsForJoint <= maxPointsInGenerator)
         {
            assertNumberOfTrajectoryPointsInGenerator(totalPointsForJoint, spineJoints[jointIdx], scs);
         }
         else
         {
            int pointsInLastTrajectory = totalPointsForJoint - maxPointsInGenerator; // fist set in generator
            while (pointsInLastTrajectory > (maxPointsInGenerator - 1))
               pointsInLastTrajectory -= (maxPointsInGenerator - 1); // keep filling the generator
            pointsInLastTrajectory++;
            assertNumberOfTrajectoryPointsInGenerator(pointsInLastTrajectory, spineJoints[jointIdx], scs);
         }
      }

      assertDesiredsMatchAfterExecution(message, spineJoints, scs);
      assertControlWasConsistent(controllerSpy);
      assertDesiredsContinous(controllerSpy);
   }

   private static void assertNumberOfTrajectoryPoints(int points, OneDoFJoint joint, SimulationConstructionSet scs)
   {
      String bodyName = "utorso";
      String prefix = bodyName + "Jointspace";
      String jointName = joint.getName();
      YoInteger numberOfPoints = getIntegerYoVariable(scs, prefix + "_" + jointName + "_numberOfPoints", bodyName + RigidBodyJointControlHelper.shortName);
      assertEquals("Unexpected number of trajectory points for " + jointName, points, numberOfPoints.getIntegerValue());
   }

   private static void assertNumberOfTrajectoryPointsInGenerator(int points, OneDoFJoint joint, SimulationConstructionSet scs)
   {
      String bodyName = "utorso";
      String prefix = bodyName + "Jointspace";
      String jointName = joint.getName();
      YoInteger numberOfPoints = getIntegerYoVariable(scs, prefix + "_" + jointName + "_numberOfPointsInGenerator", bodyName + RigidBodyJointControlHelper.shortName);
      assertEquals("Unexpected number of trajectory points for " + jointName, points, numberOfPoints.getIntegerValue());
   }

   private static void assertNumberOfTrajectoryPointsInQueue(int points, OneDoFJoint joint, SimulationConstructionSet scs)
   {
      String bodyName = "utorso";
      String prefix = bodyName + "Jointspace";
      String jointName = joint.getName();
      YoInteger numberOfPoints = getIntegerYoVariable(scs, prefix + "_" + jointName + "_numberOfPointsInQueue", bodyName + RigidBodyJointControlHelper.shortName);
      assertEquals("Unexpected number of trajectory points for " + jointName, points, numberOfPoints.getIntegerValue());
   }

   private SpineTrajectoryMessage createRandomSpineMessage(double trajectoryTime, Random random)
   {
      double[] jointDesireds = new double[numberOfJoints];
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = spineJoints[jointIdx];
         double desired = getRandomJointAngleInRange(random, joint);
         jointDesireds[jointIdx] = desired;
      }
      return HumanoidMessageTools.createSpineTrajectoryMessage(trajectoryTime, jointDesireds);
   }

   private double getRandomJointAngleInRange(Random random, OneDoFJoint joint)
   {
      double jointLimitUpper = joint.getJointLimitUpper();
      double jointLimitLower = joint.getJointLimitLower();
      return RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper);
   }

   private ChestTrajectoryMessage createRandomChestMessage(double trajectoryTime, Random random)
   {
      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(controllerFullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      return HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
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
      drcSimulationTestHelper.send(message);

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

   private static void assertDesiredsMatchAfterExecution(SpineTrajectoryMessage message, OneDoFJoint[] spineJoints, SimulationConstructionSet scs)
   {
      for (int jointIdx = 0; jointIdx < spineJoints.length; jointIdx++)
      {
         OneDoFJointTrajectoryMessage jointTrajectory = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx);
         double desired = jointTrajectory.getTrajectoryPoints().getLast().getPosition();
         OneDoFJoint joint = spineJoints[jointIdx];
         assertJointDesired(scs, joint, desired);
      }
   }

   private void executeMessage(ChestTrajectoryMessage message, RigidBody chest) throws SimulationExceededMaximumTimeException
   {
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.send(message);

      double trajectoryTime = message.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime();
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 5.0 * controllerDT));

      Quaternion desired = new Quaternion(message.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getOrientation());
      assertChestDesired(drcSimulationTestHelper.getSimulationConstructionSet(), desired, chest);
   }

   private static void assertChestDesired(SimulationConstructionSet scs, Quaternion desired, RigidBody chest)
   {
      Quaternion controllerDesired = EndToEndChestTrajectoryMessageTest.findControllerDesiredOrientation(scs, chest);
      EuclidCoreTestTools.assertQuaternionEquals(desired, controllerDesired, DESIRED_QUAT_EPSILON);
   }

   private static void assertJointDesired(SimulationConstructionSet scs, OneDoFJoint joint, double desired)
   {
      YoDouble scsDesired = findJointDesired(scs, joint);
      assertEquals(desired, scsDesired.getDoubleValue(), DESIRED_EPSILON);
   }

   private static YoDouble findJointDesired(SimulationConstructionSet scs, OneDoFJoint joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "q_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace);
   }

   private static YoBoolean findOrientationControlEnabled(SimulationConstructionSet scs, RigidBody body)
   {
      String bodyName = body.getName();
      String namespace = bodyName + "SpatialFBController";
      String variable = bodyName + "IsSpatialFBControllerEnabled";
      return getBooleanYoVariable(scs, variable, namespace);
   }

   private static YoBoolean findJointControlEnabled(SimulationConstructionSet scs, OneDoFJoint joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "control_enabled_" + jointName;
      return getBooleanYoVariable(scs, variable, namespace);
   }

   private static YoFrameQuaternion findOrientationDesired(SimulationConstructionSet scs, RigidBody body)
   {
      String bodyName = body.getName();
      String namespace = "FeedbackControllerToolbox";
      YoDouble qx = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQx", namespace);
      YoDouble qy = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQy", namespace);
      YoDouble qz = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQz", namespace);
      YoDouble qs = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQs", namespace);
      return new YoFrameQuaternion(qx, qy, qz, qs, ReferenceFrame.getWorldFrame());
   }

   private static YoBoolean getBooleanYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoBoolean.class);
   }

   private static YoInteger getIntegerYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoInteger.class);
   }

   private static YoDouble getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoDouble.class);
   }

   private static <T extends YoVariable<T>> T getYoVariable(SimulationConstructionSet scs, String name, String namespace, Class<T> clazz)
   {
      YoVariable<?> uncheckedVariable = scs.getVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
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
      spineJoints = ScrewTools.createOneDoFJointPath(pelvis, chest);
      numberOfJoints = spineJoints.length;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      controllerSpy = new ControllerSpy(spineJoints, scs, getRobotModel().getControllerDT());
      drcSimulationTestHelper.addRobotControllerOnControllerThread(controllerSpy);
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

   private class ControllerSpy extends SimpleRobotController
   {
      private final double controllerDT;
      private final OneDoFJoint[] spineJoints;
      private final OneDoFJoint[] spineJointClones;
      private final RigidBody chestClone;

      private final Map<OneDoFJoint, YoBoolean> jointControlEnabled = new HashMap<>();
      private final Map<OneDoFJoint, YoDouble> jointDesiredsMap = new HashMap<>();

      private final YoBoolean orientationControlEnabled;
      private final YoFrameQuaternion desiredOrientation;

      private final YoFrameQuaternion currentDesiredOrientation = new YoFrameQuaternion("CurrentDesired", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameQuaternion previousDesiredOrientation = new YoFrameQuaternion("PreviousDesired", ReferenceFrame.getWorldFrame(), registry);

      private final YoBoolean inconsistentControl = new YoBoolean("InconsistentControl", registry);
      private final YoDouble maxSpeed = new YoDouble("maxSpeed", registry);

      private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

      public ControllerSpy(OneDoFJoint[] spineJoints, SimulationConstructionSet scs, double controllerDT)
      {
         this.spineJoints = spineJoints;
         this.controllerDT = controllerDT;
         spineJointClones = ScrewTools.cloneOneDoFJointPath(spineJoints);
         chestClone = spineJointClones[spineJointClones.length - 1].getSuccessor();

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            OneDoFJoint joint = spineJoints[jointIdx];
            jointDesiredsMap.put(joint, findJointDesired(scs, joint));
            jointControlEnabled.put(joint, findJointControlEnabled(scs, joint));
         }
         orientationControlEnabled = findOrientationControlEnabled(scs, chest);
         desiredOrientation = findOrientationDesired(scs, chest);
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
            ScrewTools.setJointPositions(spineJointClones, jointPositions);
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
