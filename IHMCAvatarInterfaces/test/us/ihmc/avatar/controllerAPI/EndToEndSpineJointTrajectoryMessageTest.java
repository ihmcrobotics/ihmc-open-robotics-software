package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.random.RandomTools;
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
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndSpineJointTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final double DESIRED_EPSILON = 1.0E-10;
   private static final double DESIRED_QUAT_EPSILON = 0.01;
   private static final double MAX_SPEED_FOR_CONTINOUS = 20.0;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private final Random random = new Random(1991L);
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RigidBody pelvis;
   private RigidBody chest;
   private OneDoFJoint[] spineJoints;
   private int numberOfJoints;
   private ControllerSpy controllerSpy;

   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double trajectoryTime = 1.0;
      SpineTrajectoryMessage message = createRandomSpineMessage(trajectoryTime, random);

      executeMessage(message);
      assertDesiredsContinous(controllerSpy);
   }

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

      assertDesiredsContinous(controllerSpy);
   }

   private SpineTrajectoryMessage createRandomSpineMessage(double trajectoryTime, Random random)
   {
      double[] jointDesireds = new double[numberOfJoints];
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJoint joint = spineJoints[jointIdx];
         double jointLimitUpper = joint.getJointLimitUpper();
         double jointLimitLower = joint.getJointLimitLower();
         double desired = RandomTools.generateRandomDouble(random, jointLimitLower, jointLimitUpper);
         jointDesireds[jointIdx] = desired;
      }
      return new SpineTrajectoryMessage(trajectoryTime, jointDesireds);
   }

   private ChestTrajectoryMessage createRandomChestMessage(double trajectoryTime, Random random)
   {
      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      Quat4d desiredOrientation = new Quat4d();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      return new ChestTrajectoryMessage(trajectoryTime, desiredOrientation);
   }

   private static void assertDesiredsContinous(ControllerSpy controllerSpy)
   {
      assertFalse("Joint and Taskspace control was inconsistent.", controllerSpy.wasControlInconsistent());
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
         OneDoFJointTrajectoryMessage jointTrajectory = message.getTrajectoryPointsLists()[jointIdx];
         double jointTrajectoryTime = jointTrajectory.getLastTrajectoryPoint().getTime();
         if (jointTrajectoryTime > trajectoryTime)
            trajectoryTime = jointTrajectoryTime;
      }
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 5.0 * controllerDT));

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         OneDoFJointTrajectoryMessage jointTrajectory = message.getTrajectoryPointsLists()[jointIdx];
         double desired = jointTrajectory.getLastTrajectoryPoint().getPosition();
         OneDoFJoint joint = spineJoints[jointIdx];
         assertJointDesired(drcSimulationTestHelper.getSimulationConstructionSet(), joint, desired);
      }
   }

   private void executeMessage(ChestTrajectoryMessage message) throws SimulationExceededMaximumTimeException
   {
      double controllerDT = getRobotModel().getControllerDT();
      drcSimulationTestHelper.send(message);

      double trajectoryTime = message.getLastTrajectoryPoint().getTime();
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 5.0 * controllerDT));

      Quat4d desired = new Quat4d();
      message.getLastTrajectoryPoint().getOrientation(desired);
      assertChestDesired(drcSimulationTestHelper.getSimulationConstructionSet(), desired);
   }

   private static void assertChestDesired(SimulationConstructionSet scs, Quat4d desired)
   {
      Quat4d controllerDesired = EndToEndChestTrajectoryMessageTest.findControllerDesiredOrientation(scs);
      JUnitTools.assertQuaternionsEqual(desired, controllerDesired, DESIRED_QUAT_EPSILON);
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
      String namespace = bodyName + "OrientationFBController";
      String variable = bodyName + "IsOrientationFBControllerEnabled";
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
      return (BooleanYoVariable) getYoVariable(scs, name, namespace, BooleanYoVariable.class);
   }

   private static DoubleYoVariable getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return (DoubleYoVariable) getYoVariable(scs, name, namespace, DoubleYoVariable.class);
   }

   private static YoVariable<?> getYoVariable(SimulationConstructionSet scs, String name, String namespace, Class<? extends YoVariable<?>> clazz)
   {
      YoVariable<?> uncheckedVariable = scs.getVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return uncheckedVariable;
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
            Quat4d previous = previousDesiredOrientation.getQuaternionCopy();
            Quat4d current = currentDesiredOrientation.getQuaternionCopy();
            Quat4d derivative = new Quat4d();
            quaternionCalculus.computeQDotByFiniteDifferenceCentral(previous, current, controllerDT, derivative);
            Vector3d angularVelocity = new Vector3d();
            quaternionCalculus.computeAngularVelocityInWorldFrame(current, derivative, angularVelocity);
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
