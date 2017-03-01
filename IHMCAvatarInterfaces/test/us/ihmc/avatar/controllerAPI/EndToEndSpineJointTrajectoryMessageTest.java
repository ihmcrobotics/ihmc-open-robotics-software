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

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndSpineJointTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final double DESIRED_EPSILON = 1.0E-10;
   private static final double DESIRED_QUAT_EPSILON = 0.01;
   private static final double MAX_SPEED_FOR_CONTINOUS = 10.0;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      executeMessage(message2);
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

      SpineTrajectoryMessage message = new SpineTrajectoryMessage(numberOfJoints, waypoints);
      for (int waypoint = 0; waypoint < waypoints; waypoint++)
      {
         double timeAtWaypoint = totalTime * (double) waypoint / (double) waypoints;
         double desiredPosition = amplitude * Math.sin(2.0 * Math.PI * frequency * timeAtWaypoint);
         double desiredVelocity = 2.0 * Math.PI * frequency * amplitude * Math.cos(2.0 * Math.PI * frequency * timeAtWaypoint);

         if (waypoint == 0 || waypoint == waypoints - 1)
            desiredVelocity = 0.0;

         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            if (jointIdx == 0)
               message.setTrajectoryPoint(jointIdx, waypoint, timeAtWaypoint, desiredPosition, desiredVelocity);
            else
               message.setTrajectoryPoint(jointIdx, waypoint, timeAtWaypoint, 0.0, 0.0);
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
      SpineTrajectoryMessage message = new SpineTrajectoryMessage(numberOfJoints, waypoints);
      for (int waypoint = 0; waypoint < waypoints; waypoint++)
         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            message.setTrajectoryPoint(jointIdx, waypoint, 0.1 * (double) waypoint, 0.0, 0.0);
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
         SpineTrajectoryMessage message = new SpineTrajectoryMessage(numberOfJoints, numberOfPoints);
         double timeInMessage = timePerWaypoint;

         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            double desiredPosition = amplitude * Math.sin(2.0 * Math.PI * frequency * totalTime);
            double desiredVelocity = 2.0 * Math.PI * frequency * amplitude * Math.cos(2.0 * Math.PI * frequency * totalTime);

            if (msgIdx == 0 && pointIdx == 0)
               desiredVelocity = 0.0;
            if (msgIdx == numberOfMessages - 1 && pointIdx == numberOfPoints - 1)
               desiredVelocity = 0.0;

            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
               message.setTrajectoryPoint(jointIdx, pointIdx, timeInMessage, desiredPosition, desiredVelocity);

            totalTime += timePerWaypoint;
            timeInMessage += timePerWaypoint;
         }

         message.setUniqueId(msgIdx + 1);
         message.setExecutionMode(ExecutionMode.QUEUE, msgIdx);
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

      SpineTrajectoryMessage message = new SpineTrajectoryMessage(numberOfJoints);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         int numberOfPoinsForJoint = numberOfPoints[jointIdx];
         double timePerPoint = trajectoryTime[jointIdx] / (double) numberOfPoinsForJoint;
         double time = timePerPoint;

         OneDoFJointTrajectoryMessage jointTrajectoryMessage = new OneDoFJointTrajectoryMessage(numberOfPoinsForJoint);
         for (int pointIdx = 0; pointIdx < numberOfPoinsForJoint; pointIdx++)
         {
            double position = getRandomJointAngleInRange(random, spineJoints[jointIdx]);
            jointTrajectoryMessage.setTrajectoryPoint(pointIdx, time, position, 0.0);
            time += timePerPoint;
         }
         message.setTrajectory1DMessage(jointIdx, jointTrajectoryMessage);
      }

      // send message
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.send(message);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * controllerDT);

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
      IntegerYoVariable numberOfPoints = getIntegerYoVariable(scs, prefix + "_" + jointName + "_numberOfPoints", prefix + "ControlModule");
      assertEquals("Unexpected number of trajectory points for " + jointName, points, numberOfPoints.getIntegerValue());
   }

   private static void assertNumberOfTrajectoryPointsInGenerator(int points, OneDoFJoint joint, SimulationConstructionSet scs)
   {
      String bodyName = "utorso";
      String prefix = bodyName + "Jointspace";
      String jointName = joint.getName();
      IntegerYoVariable numberOfPoints = getIntegerYoVariable(scs, prefix + "_" + jointName + "_numberOfPointsInGenerator", prefix + "ControlModule");
      assertEquals("Unexpected number of trajectory points for " + jointName, points, numberOfPoints.getIntegerValue());
   }

   private static void assertNumberOfTrajectoryPointsInQueue(int points, OneDoFJoint joint, SimulationConstructionSet scs)
   {
      String bodyName = "utorso";
      String prefix = bodyName + "Jointspace";
      String jointName = joint.getName();
      IntegerYoVariable numberOfPoints = getIntegerYoVariable(scs, prefix + "_" + jointName + "_numberOfPointsInQueue", prefix + "ControlModule");
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
      return new SpineTrajectoryMessage(trajectoryTime, jointDesireds);
   }

   private double getRandomJointAngleInRange(Random random, OneDoFJoint joint)
   {
      double jointLimitUpper = joint.getJointLimitUpper();
      double jointLimitLower = joint.getJointLimitLower();
      return RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper);
   }

   private ChestTrajectoryMessage createRandomChestMessage(double trajectoryTime, Random random)
   {
      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      Quaternion desiredOrientation = new Quaternion();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      return new ChestTrajectoryMessage(trajectoryTime, desiredOrientation);
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
         OneDoFJointTrajectoryMessage jointTrajectory = message.getTrajectoryPointLists()[jointIdx];
         double jointTrajectoryTime = jointTrajectory.getLastTrajectoryPoint().getTime();
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
         OneDoFJointTrajectoryMessage jointTrajectory = message.getTrajectoryPointLists()[jointIdx];
         double desired = jointTrajectory.getLastTrajectoryPoint().getPosition();
         OneDoFJoint joint = spineJoints[jointIdx];
         assertJointDesired(scs, joint, desired);
      }
   }

   private void executeMessage(ChestTrajectoryMessage message) throws SimulationExceededMaximumTimeException
   {
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.send(message);

      double trajectoryTime = message.getLastTrajectoryPoint().getTime();
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 5.0 * controllerDT));

      Quaternion desired = new Quaternion();
      message.getLastTrajectoryPoint().getOrientation(desired);
      assertChestDesired(drcSimulationTestHelper.getSimulationConstructionSet(), desired);
   }

   private static void assertChestDesired(SimulationConstructionSet scs, Quaternion desired)
   {
      Quaternion controllerDesired = EndToEndChestTrajectoryMessageTest.findControllerDesiredOrientation(scs);
      EuclidCoreTestTools.assertQuaternionEquals(desired, controllerDesired, DESIRED_QUAT_EPSILON);
   }

   private static void assertJointDesired(SimulationConstructionSet scs, OneDoFJoint joint, double desired)
   {
      DoubleYoVariable scsDesired = findJointDesired(scs, joint);
      assertEquals(desired, scsDesired.getDoubleValue(), DESIRED_EPSILON);
   }

   private static DoubleYoVariable findJointDesired(SimulationConstructionSet scs, OneDoFJoint joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "q_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace);
   }

   private static BooleanYoVariable findOrientationControlEnabled(SimulationConstructionSet scs, RigidBody body)
   {
      String bodyName = body.getName();
      String namespace = bodyName + "SpatialFBController";
      String variable = bodyName + "IsSpatialFBControllerEnabled";
      return getBooleanYoVariable(scs, variable, namespace);
   }

   private static BooleanYoVariable findJointControlEnabled(SimulationConstructionSet scs, OneDoFJoint joint)
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
      DoubleYoVariable qx = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQx", namespace);
      DoubleYoVariable qy = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQy", namespace);
      DoubleYoVariable qz = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQz", namespace);
      DoubleYoVariable qs = getDoubleYoVariable(scs, bodyName + "DesiredOrientationQs", namespace);
      return new YoFrameQuaternion(qx, qy, qz, qs, ReferenceFrame.getWorldFrame());
   }

   private static BooleanYoVariable getBooleanYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, BooleanYoVariable.class);
   }

   private static IntegerYoVariable getIntegerYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, IntegerYoVariable.class);
   }

   private static DoubleYoVariable getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, DoubleYoVariable.class);
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
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());
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

      private final Map<OneDoFJoint, BooleanYoVariable> jointControlEnabled = new HashMap<>();
      private final Map<OneDoFJoint, DoubleYoVariable> jointDesiredsMap = new HashMap<>();

      private final BooleanYoVariable orientationControlEnabled;
      private final YoFrameQuaternion desiredOrientation;

      private final YoFrameQuaternion currentDesiredOrientation = new YoFrameQuaternion("CurrentDesired", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameQuaternion previousDesiredOrientation = new YoFrameQuaternion("PreviousDesired", ReferenceFrame.getWorldFrame(), registry);

      private final BooleanYoVariable inconsistentControl = new BooleanYoVariable("InconsistentControl", registry);
      private final DoubleYoVariable maxSpeed = new DoubleYoVariable("maxSpeed", registry);

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
            FrameOrientation chestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
            chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
            currentDesiredOrientation.set(chestOrientation);
         }
         else
         {
            currentDesiredOrientation.set(desiredOrientation);
         }

         if (!currentDesiredOrientation.containsNaN() && !previousDesiredOrientation.containsNaN())
         {
            Quaternion previous = previousDesiredOrientation.getQuaternionCopy();
            Quaternion current = currentDesiredOrientation.getQuaternionCopy();
            Vector4D derivative = new Vector4D();
            quaternionCalculus.computeQDotByFiniteDifferenceCentral(previous, current, controllerDT, derivative);
            Vector3D angularVelocity = new Vector3D();
            quaternionCalculus.computeAngularVelocityInWorldFrame(current, derivative, angularVelocity);
            double speed = angularVelocity.length();
            if (speed > maxSpeed.getDoubleValue())
               maxSpeed.set(speed);
         }

         previousDesiredOrientation.set(currentDesiredOrientation);
      }

      public double getMaxSpeed()
      {
         PrintTools.info("Max Speed: " + maxSpeed.getDoubleValue());
         return maxSpeed.getDoubleValue();
      }

      public boolean wasControlInconsistent()
      {
         return inconsistentControl.getBooleanValue();
      }
   }

}
