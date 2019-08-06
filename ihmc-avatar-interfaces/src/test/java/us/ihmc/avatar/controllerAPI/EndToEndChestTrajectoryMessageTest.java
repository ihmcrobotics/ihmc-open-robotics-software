package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

@Tag("controller-api")
public abstract class EndToEndChestTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double EPSILON_FOR_DESIREDS = 5.0e-4;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private final String prefix = "Orientation";

   @Test
   public void testLookingLeftAndRight() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      double trajectoryTime = 1.0;
      FrameQuaternion lookStraightAhead = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), new Quaternion());
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookLeftQuat = new Quaternion();
      lookLeftQuat.appendYawRotation(Math.PI / 8.0);
      lookLeftQuat.appendPitchRotation(Math.PI / 16.0);
      lookLeftQuat.appendRollRotation(-Math.PI / 16.0);
      FrameQuaternion lookLeft = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookRightQuat = new Quaternion();
      lookRightQuat.appendYawRotation(-Math.PI / 8.0);
      lookRightQuat.appendPitchRotation(-Math.PI / 16.0);
      lookRightQuat.appendRollRotation(Math.PI / 16.0);
      FrameQuaternion lookRight = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage lookStraightAheadMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                          lookStraightAhead,
                                                                                                          ReferenceFrame.getWorldFrame(),
                                                                                                          pelvisZUpFrame);
      lookStraightAheadMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookStraightAheadMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookLeftMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                 lookLeft,
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 pelvisZUpFrame);
      lookLeftMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookLeftMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookRightMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                  lookRight,
                                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                                  pelvisZUpFrame);
      lookRightMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookRightMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookRightMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * trajectoryTime + 1.0));
   }

   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();

      OneDoFJointBasics[] spineClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);
      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);
      RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        ReferenceFrame.getWorldFrame(),
                                                                                                        pelvisZUpFrame);
      chestTrajectoryMessage.setSequenceId(random.nextLong());
      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(controllerDT));

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(chestTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.STARTED,
                                                        0.0,
                                                        chest.getName(),
                                                        statusMessages.remove(0),
                                                        controllerDT);

      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertControlErrorIsLow(scs, chest, 1.0e-2);
      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs, chest, prefix);

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(chestTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.COMPLETED,
                                                        trajectoryTime,
                                                        null,
                                                        desiredRandomChestOrientation,
                                                        chest.getName(),
                                                        statusMessages.remove(0),
                                                        1.0e-4,
                                                        controllerDT);
   }

   public void testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      RigidBodyBasics chest = fullRobotModel.getChest();

      //      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      //      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      //      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      //      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      //      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(pelvisZUpFrame, Math.PI / 8.0, Math.PI / 8.0, Math.PI / 16.0);

      Quaternion desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        pelvisZUpFrame,
                                                                                                        pelvisZUpFrame);
      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      selectionMatrix3D.selectZAxis(false);
      selectionMatrix3D.selectYAxis(false);
      selectionMatrix3D.selectXAxis(false);
      selectionMatrix3D.setSelectionFrame(pelvisZUpFrame);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix3D));
      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
      //      desiredRandomChestOrientation.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      Vector3D rotationVector = new Vector3D();
      desiredChestOrientation.getRotationVector(rotationVector);
      DenseMatrix64F rotationVectorMatrix = new DenseMatrix64F(3, 1);
      rotationVector.get(rotationVectorMatrix);

      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 3);
      selectionMatrix3D.getFullSelectionMatrixInFrame(pelvisZUpFrame, selectionMatrix);

      DenseMatrix64F result = new DenseMatrix64F(3, 1);
      CommonOps.mult(selectionMatrix, rotationVectorMatrix, result);
      rotationVector.set(result);
      desiredChestOrientation.setRotationVector(rotationVector);

      System.out.println(desiredChestOrientation);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      //      assertControlErrorIsLow(scs, chest, 1.0e-2);

      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);
      humanoidReferenceFrames.updateFrames();
      FrameQuaternion achievedChestOrientation = new FrameQuaternion();
      achievedChestOrientation.setToZero(chest.getBodyFixedFrame());
      achievedChestOrientation.changeFrame(pelvisZUpFrame);
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      EuclidCoreTestTools.assertQuaternionEquals(desiredChestOrientation, achievedChestOrientation, 1e-2);
      //      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs, chest);
   }

   public void testSettingWeightMatrixUsingSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();

      OneDoFJointBasics[] spineClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);

      for (int i = 0; i < 50; i++)
      {
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);
         RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
         FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
         desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);

         ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                           desiredOrientation,
                                                                                                           pelvisZUpFrame,
                                                                                                           pelvisZUpFrame);

         WeightMatrix3D weightMatrix = new WeightMatrix3D();

         double xWeight = random.nextDouble();
         double yWeight = random.nextDouble();
         double zWeight = random.nextDouble();

         weightMatrix.setWeights(xWeight, yWeight, zWeight);
         chestTrajectoryMessage.getSo3Trajectory().getWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(weightMatrix));
         drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 4.0));
         assertWeightsMatch(xWeight, yWeight, zWeight, chest, scs);
      }

      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);
      RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);

      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        pelvisZUpFrame,
                                                                                                        pelvisZUpFrame);

      WeightMatrix3D weightMatrix = new WeightMatrix3D();

      double xWeight = Double.NaN;
      double yWeight = Double.NaN;
      double zWeight = Double.NaN;

      weightMatrix.setWeights(xWeight, yWeight, zWeight);
      chestTrajectoryMessage.getSo3Trajectory().getWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(weightMatrix));
      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 4.0));
      assertAngularWeightsMatchDefault(chest, scs);

   }

   private void assertAngularWeightsMatchDefault(RigidBodyBasics rigidBody, SimulationConstructionSet scs)
   {
      String prefix = rigidBody.getName() + "TaskspaceOrientation";
      YoDouble angularWeightX = (YoDouble) scs.getVariable(prefix + "CurrentWeightX");
      YoDouble angularWeightY = (YoDouble) scs.getVariable(prefix + "CurrentWeightY");
      YoDouble angularWeightZ = (YoDouble) scs.getVariable(prefix + "CurrentWeightZ");
      YoDouble defaultAngularWeightX = (YoDouble) scs.getVariable("ChestAngularWeightX");
      YoDouble defaultAngularWeightY = (YoDouble) scs.getVariable("ChestAngularWeightY");
      YoDouble defaultAngularWeightZ = (YoDouble) scs.getVariable("ChestAngularWeightZ");
      assertEquals(defaultAngularWeightX.getDoubleValue(), angularWeightX.getDoubleValue(), 1e-8);
      assertEquals(defaultAngularWeightY.getDoubleValue(), angularWeightY.getDoubleValue(), 1e-8);
      assertEquals(defaultAngularWeightZ.getDoubleValue(), angularWeightZ.getDoubleValue(), 1e-8);

   }

   private void assertWeightsMatch(double xWeight, double yWeight, double zWeight, RigidBodyBasics rigidBody, SimulationConstructionSet scs)
   {
      String prefix = rigidBody.getName() + "TaskspaceOrientation";
      YoDouble angularWeightX = (YoDouble) scs.getVariable(prefix + "CurrentWeightX");
      YoDouble angularWeightY = (YoDouble) scs.getVariable(prefix + "CurrentWeightY");
      YoDouble angularWeightZ = (YoDouble) scs.getVariable(prefix + "CurrentWeightZ");

      assertEquals(xWeight, angularWeightX.getDoubleValue(), 1e-8);
      assertEquals(yWeight, angularWeightY.getDoubleValue(), 1e-8);
      assertEquals(zWeight, angularWeightZ.getDoubleValue(), 1e-8);
   }

   public void testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(56457L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      RigidBodyBasics chest = fullRobotModel.getChest();

      //      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      //      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      //      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      //      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      //      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(pelvisZUpFrame, Math.PI / 16.0, Math.PI / 20.0, Math.PI / 24.0);

      Quaternion desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        pelvisZUpFrame,
                                                                                                        pelvisZUpFrame);
      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      selectionMatrix3D.selectZAxis(random.nextBoolean());
      selectionMatrix3D.selectYAxis(random.nextBoolean());
      selectionMatrix3D.selectXAxis(random.nextBoolean());
      selectionMatrix3D.setSelectionFrame(pelvisZUpFrame);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix3D));
      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
      //      desiredRandomChestOrientation.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      Vector3D rotationVector = new Vector3D();
      desiredChestOrientation.getRotationVector(rotationVector);
      DenseMatrix64F rotationVectorMatrix = new DenseMatrix64F(3, 1);
      rotationVector.get(rotationVectorMatrix);

      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 3);
      selectionMatrix3D.getFullSelectionMatrixInFrame(pelvisZUpFrame, selectionMatrix);

      DenseMatrix64F result = new DenseMatrix64F(3, 1);
      CommonOps.mult(selectionMatrix, rotationVectorMatrix, result);
      rotationVector.set(result);
      desiredChestOrientation.setRotationVector(rotationVector);

      System.out.println(desiredChestOrientation);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      //      assertControlErrorIsLow(scs, chest, 1.0e-2);

      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);
      humanoidReferenceFrames.updateFrames();
      FrameQuaternion achievedChestOrientation = new FrameQuaternion();
      achievedChestOrientation.setToZero(chest.getBodyFixedFrame());
      achievedChestOrientation.changeFrame(pelvisZUpFrame);
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      EuclidCoreTestTools.assertQuaternionEquals(desiredChestOrientation, achievedChestOrientation, 0.04);
      //      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs, chest);
   }

   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(3456357);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 15;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics chest = fullRobotModel.getChest();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      chestTrajectoryMessage.setSequenceId(random.nextLong());

      FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
      FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
         double pitch = amp * Math.sin(t * w);
         double pitchDot = w * amp * Math.cos(t * w);
         desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion();
         desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
         desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D();
         desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);
         if (trajectoryPointIndex == numberOfTrajectoryPoints - 1)
            desiredChestAngularVelocities[trajectoryPointIndex].setToZero();

         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(t);
         trajectoryPoint.getOrientation().set(desiredChestOrientations[trajectoryPointIndex]);
         trajectoryPoint.getAngularVelocity().set(desiredChestAngularVelocities[trajectoryPointIndex]);
      }

      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(controllerDT); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      humanoidReferenceFrames.updateFrames();

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(chestTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.STARTED,
                                                        0.0,
                                                        chest.getName(),
                                                        statusMessages.remove(0),
                                                        controllerDT);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         desiredChestOrientations[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(controllerDT);
      assertTrue(success);

      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, numberOfTrajectoryPoints + 1, scs);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = so3Trajectory.getTaskspaceTrajectoryPoints().get(trajectoryPointIndex).getTime();
         SO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs, chest);
         assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         desiredChestOrientations[trajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         desiredChestAngularVelocities[trajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      SO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(chestTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.COMPLETED,
                                                        trajectoryTime,
                                                        null,
                                                        desiredChestOrientations[desiredChestOrientations.length - 1],
                                                        chest.getName(),
                                                        statusMessages.remove(0),
                                                        1.0e-3,
                                                        controllerDT);
   }

   public void testMessageWithALotOfTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.05;
      int numberOfTrajectoryPoints = 65;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics chest = fullRobotModel.getChest();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

      FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
      FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         double roll = amp * Math.sin(t * w);
         double rollDot = w * amp * Math.cos(t * w);
         desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion();
         desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, 0.0, roll);
         desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D();

         if (trajectoryPointIndex == 0 || trajectoryPointIndex == numberOfTrajectoryPoints - 1)
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, 0.0, 0.0);
         else
            desiredChestAngularVelocities[trajectoryPointIndex].set(rollDot, 0.0, 0.0);

         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(t);
         trajectoryPoint.getOrientation().set(desiredChestOrientations[trajectoryPointIndex]);
         trajectoryPoint.getAngularVelocity().set(desiredChestAngularVelocities[trajectoryPointIndex]);
      }

      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      humanoidReferenceFrames.updateFrames();

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint + getRobotModel().getControllerDT());
      assertTrue(success);

      int expectedTrajectoryPointIndex = 0;
      double previousTimeInState = timePerWaypoint;

      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(),
                                                                       prefix,
                                                                       Math.min(RigidBodyTaskspaceControlState.maxPoints,
                                                                                numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1),
                                                                       scs);

      double timeInState = 0.0;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = so3Trajectory.getTaskspaceTrajectoryPoints().get(expectedTrajectoryPointIndex).getTime();
         SO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs, chest);
         assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         desiredChestOrientations[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         desiredChestAngularVelocities[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

         timeInState = Math.max(time, timeInState);

         expectedTrajectoryPointIndex++;

         if (expectedTrajectoryPointIndex == numberOfTrajectoryPoints)
            break;
      }

      double simulationTime = timeInState - previousTimeInState;
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
      previousTimeInState = timeInState;

      SO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint * numberOfTrajectoryPoints + 0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   public void testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.05;
      int numberOfTrajectoryPoints = 65;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics chest = fullRobotModel.getChest();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));

      FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
      FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         double roll = amp * Math.sin(t * w);
         double rollDot = w * amp * Math.cos(t * w);
         desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion();
         desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, 0.0, roll);
         desiredChestOrientations[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
         desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D();

         if (trajectoryPointIndex == 0 || trajectoryPointIndex == numberOfTrajectoryPoints - 1)
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, 0.0, 0.0);
         else
            desiredChestAngularVelocities[trajectoryPointIndex].set(rollDot, 0.0, 0.0);
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(pelvisZUpFrame);

         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(t);
         trajectoryPoint.getOrientation().set(desiredChestOrientations[trajectoryPointIndex]);
         trajectoryPoint.getAngularVelocity().set(desiredChestAngularVelocities[trajectoryPointIndex]);
      }

      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      humanoidReferenceFrames.updateFrames();

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint + getRobotModel().getControllerDT());
      assertTrue(success);

      int expectedTrajectoryPointIndex = 0;
      double previousTimeInState = timePerWaypoint;

      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(),
                                                                       prefix,
                                                                       Math.min(RigidBodyTaskspaceControlState.maxPoints,
                                                                                numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1),
                                                                       scs);

      double timeInState = 0.0;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = so3Trajectory.getTaskspaceTrajectoryPoints().get(expectedTrajectoryPointIndex).getTime();
         SO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs, chest);
         assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         desiredChestOrientations[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         desiredChestAngularVelocities[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

         timeInState = Math.max(time, timeInState);

         expectedTrajectoryPointIndex++;

         if (expectedTrajectoryPointIndex == numberOfTrajectoryPoints)
            break;
      }

      double simulationTime = timeInState - previousTimeInState;
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
      previousTimeInState = timeInState;

      SO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint * numberOfTrajectoryPoints + 0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(532);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      double timePerWaypoint = 0.05;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics chest = fullRobotModel.getChest();

      List<FrameQuaternion[]> desiredChestOrientationsList = new ArrayList<>();
      List<FrameVector3D[]> desiredChestAngularVelocitiesList = new ArrayList<>();
      List<ChestTrajectoryMessage> messageList = new ArrayList<>();

      double t = 0.0;
      double w = 2.0 * Math.PI / (trajectoryTime * numberOfMessages);
      double amp = Math.toRadians(20.0);
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
         FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
         chestTrajectoryMessage.setSequenceId(random.nextLong());
         SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
         so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));
         so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

         so3Trajectory.getQueueingProperties().setMessageId(id);
         if (messageIndex > 0)
         {
            so3Trajectory.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            so3Trajectory.getQueueingProperties().setPreviousMessageId(id - 1);
         }
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
            double pitch = amp * Math.sin((t + tOffset) * w);
            double pitchDot = w * amp * Math.cos((t + tOffset) * w);
            desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestOrientations[trajectoryPointIndex].changeFrame(ReferenceFrame.getWorldFrame());
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(ReferenceFrame.getWorldFrame());

            if (messageIndex == numberOfMessages - 1 && trajectoryPointIndex == numberOfTrajectoryPoints - 1)
               desiredChestAngularVelocities[trajectoryPointIndex].setToZero();
            if (messageIndex == 0 && trajectoryPointIndex == 0)
               desiredChestAngularVelocities[trajectoryPointIndex].setToZero();

            SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
            trajectoryPoint.setTime(t);
            trajectoryPoint.getOrientation().set(desiredChestOrientations[trajectoryPointIndex]);
            trajectoryPoint.getAngularVelocity().set(desiredChestAngularVelocities[trajectoryPointIndex]);
         }
         drcSimulationTestHelper.publishToController(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(controllerDT); // Trick to get frames synchronized with the controller.
         assertTrue(success);

         humanoidReferenceFrames.updateFrames();
         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         }

         desiredChestOrientationsList.add(desiredChestOrientations);
         desiredChestAngularVelocitiesList.add(desiredChestAngularVelocities);
         messageList.add(chestTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * controllerDT);
      assertTrue(success);

      int totalPoints = numberOfMessages * numberOfTrajectoryPoints + 1;
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, totalPoints, scs);

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         ChestTrajectoryMessage chestTrajectoryMessage = messageList.get(messageIndex);
         double simulationTime = chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime();
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      // These asserts do not work well since the controller switches frames on the desireds differently then the test.
      //      humanoidReferenceFrames.updateFrames();
      //      FrameOrientation desiredOrientation = desiredChestOrientationsList.get(numberOfMessages - 1)[numberOfTrajectoryPoints - 1];
      //      FrameVector desiredAngularVelocity = desiredChestAngularVelocitiesList.get(numberOfMessages - 1)[numberOfTrajectoryPoints - 1];
      //      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      //      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      //      desiredAngularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      //      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientation.getQuaternion(), controllerTrajectoryPoint.getOrientationCopy(), 1.0e-3);
      //      EuclidCoreTestTools.assertTuple3DEquals(desiredAngularVelocity.getVector(), controllerTrajectoryPoint.getAngularVelocityCopy(), 1.0e-3);

      int maxPointsInGenerator = RigidBodyTaskspaceControlState.maxPointsInGenerator;
      int pointsInLastTrajectory = totalPoints - Math.min((numberOfTrajectoryPoints + 1), maxPointsInGenerator); // fist set in generator
      while (pointsInLastTrajectory > (maxPointsInGenerator - 1))
         pointsInLastTrajectory -= (maxPointsInGenerator - 1); // keep filling the generator
      pointsInLastTrajectory++;

      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, pointsInLastTrajectory, scs);
      assertControlErrorIsLow(scs, chest, 1.0e-2);

      assertEquals(2 * messageList.size(), statusMessages.size());

      double startTime = 0.0;

      for (int inputIndex = 0; inputIndex < messageList.size(); inputIndex++)
      {
         ChestTrajectoryMessage chestTrajectoryMessage = messageList.get(inputIndex);
         Object<SO3TrajectoryPointMessage> taskspaceTrajectoryPoints = chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints();

         double endTime = startTime + taskspaceTrajectoryPoints.getLast().getTime();
         if (inputIndex > 0)
            startTime += taskspaceTrajectoryPoints.getFirst().getTime();

         TaskspaceTrajectoryStatusMessage startedStatus = statusMessages.remove(0);
         TaskspaceTrajectoryStatusMessage completedStatus = statusMessages.remove(0);
         long expectedSequenceID = chestTrajectoryMessage.getSequenceId();
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                                           TrajectoryExecutionStatus.STARTED,
                                                           startTime,
                                                           chest.getName(),
                                                           startedStatus,
                                                           controllerDT);
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           endTime,
                                                           chest.getName(),
                                                           completedStatus,
                                                           controllerDT);
         startTime = endTime;
      }
   }

   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics chest = fullRobotModel.getChest();
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      double timePerWaypoint = 0.02;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

      List<FrameQuaternion[]> desiredChestOrientationsList = new ArrayList<>();
      List<FrameVector3D[]> desiredChestAngularVelocitiesList = new ArrayList<>();
      List<ChestTrajectoryMessage> messageList = new ArrayList<>();

      double t = 0.0;
      double w = 2.0 * Math.PI / (trajectoryTime * numberOfMessages);
      double amp = Math.toRadians(20.0);
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
         FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
         SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
         so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));
         so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         so3Trajectory.getQueueingProperties().setMessageId(id);
         if (messageIndex > 0)
         {
            long previousMessageId = id - 1;
            if (messageIndex == numberOfMessages - 1)
               previousMessageId = id + 100; // Bad ID

            so3Trajectory.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            so3Trajectory.getQueueingProperties().setPreviousMessageId(previousMessageId);
         }
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint;
            double pitch = amp * Math.sin((t + tOffset) * w);
            double pitchDot = w * amp * Math.cos((t + tOffset) * w);
            desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion();
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D();
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);

            SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
            trajectoryPoint.setTime(t);
            trajectoryPoint.getOrientation().set(desiredChestOrientations[trajectoryPointIndex]);
            trajectoryPoint.getAngularVelocity().set(desiredChestAngularVelocities[trajectoryPointIndex]);
         }
         drcSimulationTestHelper.publishToController(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
         assertTrue(success);
         humanoidReferenceFrames.updateFrames();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         }

         desiredChestOrientationsList.add(desiredChestOrientations);
         desiredChestAngularVelocitiesList.add(desiredChestAngularVelocities);
         messageList.add(chestTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05 + getRobotModel().getControllerDT());
      assertTrue(success);

      RigidBodyControlMode defaultControlMode = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlMode == null)
      {
         defaultControlMode = RigidBodyControlMode.JOINTSPACE;
      }
      assertEquals(defaultControlMode, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), scs));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 1, scs);
   }

   public void testQueueWithUsingDifferentTrajectoryFrameWithoutOverride() throws Exception
   {

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();
      RigidBodyBasics chest = fullRobotModel.getChest();

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      double trajectoryTime = 1.0;
      FrameQuaternion lookStraightAhead = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), new Quaternion());
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookLeftQuat = new Quaternion();
      lookLeftQuat.appendYawRotation(Math.PI / 8.0);
      lookLeftQuat.appendPitchRotation(Math.PI / 16.0);
      lookLeftQuat.appendRollRotation(-Math.PI / 16.0);
      FrameQuaternion lookLeft = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookRightQuat = new Quaternion();
      lookRightQuat.appendYawRotation(-Math.PI / 8.0);
      lookRightQuat.appendPitchRotation(-Math.PI / 16.0);
      lookRightQuat.appendRollRotation(Math.PI / 16.0);
      FrameQuaternion lookRight = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage lookStraightAheadMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                          lookStraightAhead,
                                                                                                          ReferenceFrame.getWorldFrame(),
                                                                                                          pelvisZUpFrame);
      lookStraightAheadMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      lookStraightAheadMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);

      ChestTrajectoryMessage lookRightMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                  lookRight,
                                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                                  pelvisZUpFrame);
      lookRightMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookRightMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookRightMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 3, scs);

      ChestTrajectoryMessage LookLeftMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                 lookLeft,
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 pelvisZUpFrame);
      LookLeftMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      LookLeftMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(LookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 4, scs);

      ChestTrajectoryMessage LookLeftMessageWithChangeTrajFrame = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                                    lookLeft,
                                                                                                                    ReferenceFrame.getWorldFrame(),
                                                                                                                    ReferenceFrame.getWorldFrame());
      LookLeftMessageWithChangeTrajFrame.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      LookLeftMessageWithChangeTrajFrame.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(LookLeftMessageWithChangeTrajFrame);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      RigidBodyControlMode defaultControlMode = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlMode == null)
      {
         defaultControlMode = RigidBodyControlMode.JOINTSPACE;
      }
      assertEquals(defaultControlMode, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), scs));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 1, scs);

      drcSimulationTestHelper.publishToController(lookRightMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);

      drcSimulationTestHelper.publishToController(LookLeftMessageWithChangeTrajFrame);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));

      assertEquals(defaultControlMode, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), scs));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 1, scs);

      drcSimulationTestHelper.publishToController(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);
      LookLeftMessageWithChangeTrajFrame.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      LookLeftMessageWithChangeTrajFrame.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      drcSimulationTestHelper.publishToController(LookLeftMessageWithChangeTrajFrame);
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * trajectoryTime + 1.0));

   }

   public void testLookingLeftAndRightInVariousTrajectoryFrames() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      double trajectoryTime = 1.0;
      FrameQuaternion lookStraightAhead = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), new Quaternion());
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookLeftQuat = new Quaternion();
      lookLeftQuat.appendYawRotation(Math.PI / 8.0);
      lookLeftQuat.appendPitchRotation(Math.PI / 16.0);
      lookLeftQuat.appendRollRotation(-Math.PI / 16.0);
      FrameQuaternion lookLeft = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookRightQuat = new Quaternion();
      lookRightQuat.appendYawRotation(-Math.PI / 8.0);
      lookRightQuat.appendPitchRotation(-Math.PI / 16.0);
      lookRightQuat.appendRollRotation(Math.PI / 16.0);
      FrameQuaternion lookRight = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage lookStraightAheadMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                          lookStraightAhead,
                                                                                                          ReferenceFrame.getWorldFrame(),
                                                                                                          pelvisZUpFrame);
      lookStraightAheadMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookStraightAheadMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookLeftMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                 lookLeft,
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 pelvisZUpFrame);
      lookLeftMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookLeftMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookRightMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                  lookRight,
                                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                                  pelvisZUpFrame);
      lookRightMessage.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      lookRightMessage.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(-1);
      drcSimulationTestHelper.publishToController(lookRightMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * trajectoryTime + 1.0));
   }

   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.02;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics chest = fullRobotModel.getChest();

      List<FrameQuaternion[]> desiredChestOrientationsList = new ArrayList<>();
      List<FrameVector3D[]> desiredChestAngularVelocitiesList = new ArrayList<>();
      List<ChestTrajectoryMessage> messageList = new ArrayList<>();

      double t = 0.0;
      double w = 2.0 * Math.PI / (trajectoryTime * numberOfMessages);
      double amp = Math.toRadians(20.0);
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
         FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
         SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
         so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));
         so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

         so3Trajectory.getQueueingProperties().setMessageId(id);
         if (messageIndex > 0)
         {
            so3Trajectory.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            so3Trajectory.getQueueingProperties().setPreviousMessageId(id - 1);
         }
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint;
            double pitch = amp * Math.sin((t + tOffset) * w);
            double pitchDot = w * amp * Math.cos((t + tOffset) * w);
            desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion();
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D();
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);

            SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
            trajectoryPoint.setTime(t);
            trajectoryPoint.getOrientation().set(desiredChestOrientations[trajectoryPointIndex]);
            trajectoryPoint.getAngularVelocity().set(desiredChestAngularVelocities[trajectoryPointIndex]);
         }
         drcSimulationTestHelper.publishToController(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
         assertTrue(success);
         humanoidReferenceFrames.updateFrames();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            desiredChestOrientations[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
         }

         desiredChestOrientationsList.add(desiredChestOrientations);
         desiredChestAngularVelocitiesList.add(desiredChestAngularVelocities);
         messageList.add(chestTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      OneDoFJointBasics[] spineClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);

      Random random = new Random(21651L);
      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);

      RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      trajectoryTime = 0.5;
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        ReferenceFrame.getWorldFrame(),
                                                                                                        pelvisZUpFrame);
      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs, chest, prefix);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 5.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();

      MovingZUpFrame pelvisZUpFrame = new MovingZUpFrame(fullRobotModel.getRootJoint().getFrameAfterJoint(), "pelvisZUpFrame");
      OneDoFJointBasics[] spineClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(pelvis, chest);

      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, spineClone);

      RigidBodyBasics chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        ReferenceFrame.getWorldFrame(),
                                                                                                        pelvisZUpFrame);

      pelvisZUpFrame.update();
      desiredRandomChestOrientation.changeFrame(pelvisZUpFrame);

      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      QuaternionReadOnly desiredOrientationBeforeStop = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs);

      assertEquals(RigidBodyControlMode.TASKSPACE, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), scs));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 2, scs);

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.publishToController(stopAllTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      QuaternionReadOnly desiredOrientationAfterStop = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs);

      RigidBodyControlMode defaultControlMode = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlMode == null)
      {
         defaultControlMode = RigidBodyControlMode.JOINTSPACE;
      }
      assertEquals(defaultControlMode, EndToEndTestTools.findRigidBodyControlManagerState(chest.getName(), scs));
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(chest.getName(), prefix, 1, scs);

      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientationBeforeStop, desiredOrientationAfterStop, 1.0e-3);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   /**
    * This test is to reproduce a bug found on Valkyrie where sending a stop all trajectory would cause
    * the robot to move the chest.
    */
   public void testStopAllTrajectoryRepeatedly() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.4, 0.0, 1.0), new Point3D(0.4, 8.0, 1.0));
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));

      // Apply a push to the robot so we get some tracking error going
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      double forceMagnitude = fullRobotModel.getTotalMass() * 0.1;
      double zOffset = 0.3;
      String pushJointName = fullRobotModel.getChest().getParentJoint().getName();
      PushRobotController pushController = new PushRobotController(drcSimulationTestHelper.getRobot(),
                                                                   pushJointName,
                                                                   new Vector3D(0, 0, zOffset),
                                                                   1.0 / forceMagnitude);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphic(pushController.getForceVisualizer());
      pushController.applyForce(new Vector3D(1.0, 0.0, 0.0), forceMagnitude, Double.POSITIVE_INFINITY);

      // Need to hold in world to avoid error from slight robot motions.
      double trajectoryTime = 0.5;
      Quaternion desiredOrientation = new Quaternion();
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        desiredOrientation,
                                                                                                        referenceFrames.getPelvisZUpFrame());
      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);
      drcSimulationTestHelper.simulateAndBlock(trajectoryTime + 0.1);

      // Step the trajectory repeatedly
      for (int i = 0; i < 20; i++)
      {
         StopAllTrajectoryMessage stopMessage = new StopAllTrajectoryMessage();
         drcSimulationTestHelper.publishToController(stopMessage);
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      }

      // Record the desired chest orientation (in world)
      referenceFrames.updateFrames();
      FrameQuaternion finalOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(),
                                                             EndToEndTestTools.findFeedbackControllerDesiredOrientation(fullRobotModel.getChest().getName(),
                                                                                                                        drcSimulationTestHelper.getSimulationConstructionSet()));
      finalOrientation.changeFrame(referenceFrames.getPelvisZUpFrame());
      Quaternion finalDesiredChestOrientation = new Quaternion(finalOrientation);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, finalDesiredChestOrientation, 1.0e-5);
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(54651);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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

      YoFrameQuaternion initialOrientation = new YoFrameQuaternion("chestInitialOrientation", worldFrame, testRegistry);
      YoFrameQuaternion finalOrientation = new YoFrameQuaternion("chestFinalOrientation", worldFrame, testRegistry);
      YoFrameQuaternion desiredOrientation = new YoFrameQuaternion("chestDesiredOrientation", worldFrame, testRegistry);
      YoFrameVector3D desiredAngularVelocity = new YoFrameVector3D("chestDesiredAngularVelocity", worldFrame, testRegistry);

      RigidBodyBasics chest = fullRobotModel.getChest();
      initialOrientation.setFromReferenceFrame(chest.getBodyFixedFrame());
      OneDoFJointBasics[] cloneOneDoFJointKinematicChain = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(fullRobotModel.getPelvis(), chest);
      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, cloneOneDoFJointKinematicChain);
      cloneOneDoFJointKinematicChain[0].updateFramesRecursively();
      finalOrientation.setFromReferenceFrame(cloneOneDoFJointKinematicChain[cloneOneDoFJointKinematicChain.length - 1].getSuccessor().getBodyFixedFrame());

      drcSimulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         private boolean everyOtherTick = false;
         private final OrientationInterpolationCalculator calculator = new OrientationInterpolationCalculator();

         @Override
         public void doControl()
         {
            everyOtherTick = !everyOtherTick;

            if (!everyOtherTick)
               return;
            
            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            desiredOrientation.interpolate(initialOrientation, finalOrientation, alpha);
            if (alpha <= 0.0 || alpha >= 1.0)
               desiredAngularVelocity.setToZero();
            else
               calculator.computeAngularVelocity(desiredAngularVelocity, initialOrientation, finalOrientation, 1.0 / trajectoryTime.getValue());
            ChestTrajectoryMessage message = HumanoidMessageTools.createChestTrajectoryMessage(0.0, desiredOrientation, desiredAngularVelocity, worldFrame);
            message.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
            message.getSo3Trajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
            drcSimulationTestHelper.publishToController(message);
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

      SO3TrajectoryPoint currentDesiredTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      double desiredEpsilon = 6.0e-3;

      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, currentDesiredTrajectoryPoint.getOrientation(), desiredEpsilon);
      EuclidCoreTestTools.assertTuple3DEquals(desiredAngularVelocity, currentDesiredTrajectoryPoint.getAngularVelocity(), desiredEpsilon);
      assertControlErrorIsLow(scs, chest, 1.0e-2);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);
      
      currentDesiredTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      desiredEpsilon = 1.0e-7;
      
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, currentDesiredTrajectoryPoint.getOrientation(), desiredEpsilon);
      EuclidCoreTestTools.assertTuple3DEquals(desiredAngularVelocity, currentDesiredTrajectoryPoint.getAngularVelocity(), desiredEpsilon);
      assertControlErrorIsLow(scs, chest, 2.0e-4);
   }

   public static Vector3D findControlErrorRotationVector(SimulationConstructionSet scs, RigidBodyBasics chest)
   {
      String chestPrefix = chest.getName();
      String nameSpace = FeedbackControllerToolbox.class.getSimpleName();
      String varName = chestPrefix + "ErrorRotationVector";
      return EndToEndTestTools.findVector3D(nameSpace, varName, scs);
   }

   public static SO3TrajectoryPoint findTrajectoryPoint(int trajectoryPointIndex, SimulationConstructionSet scs, RigidBodyBasics chestRigidBody)
   {
      String chestPrefix = chestRigidBody.getName();
      String orientationTrajectoryName = chestPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = chestPrefix + "Time";
      String orientationName = chestPrefix + "Orientation";
      String angularVelocityName = chestPrefix + "AngularVelocity";

      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setTime(scs.getVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSO3TrajectoryPoint.setOrientation(EndToEndTestTools.findQuaternion(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSO3TrajectoryPoint.setAngularVelocity(EndToEndTestTools.findVector3D(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSO3TrajectoryPoint;
   }

   public static SO3TrajectoryPoint findCurrentDesiredTrajectoryPoint(SimulationConstructionSet scs, RigidBodyBasics chest)
   {
      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setOrientation(EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs));
      simpleSO3TrajectoryPoint.setAngularVelocity(EndToEndTestTools.findFeedbackControllerDesiredAngularVelocity(chest.getName(), scs));
      return simpleSO3TrajectoryPoint;
   }

   public static void assertControlErrorIsLow(SimulationConstructionSet scs, RigidBodyBasics chest, double errorTolerance)
   {
      Vector3D error = findControlErrorRotationVector(scs, chest);
      boolean isErrorLow = error.length() <= errorTolerance;
      assertTrue(isErrorLow, "Error: " + error.length());
   }

   public static void assertSingleWaypointExecuted(FrameQuaternion desiredOrientation, SimulationConstructionSet scs, RigidBodyBasics body, String prefix)
   {
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(body.getName(), prefix, 2, scs);

      QuaternionReadOnly controllerDesiredOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(body.getName(), scs);
      FrameQuaternion controllerDesiredFrameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), controllerDesiredOrientation);
      controllerDesiredFrameOrientation.changeFrame(desiredOrientation.getReferenceFrame());

      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientation, controllerDesiredFrameOrientation, EPSILON_FOR_DESIREDS);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

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

   public static void main(String[] args)
   {
      RigidBodyTransform t1 = new RigidBodyTransform();
      Vector3D rotationVector = new Vector3D();
      DenseMatrix64F rotationVectorMatrix = new DenseMatrix64F(3, 1);

      t1.appendYawRotation(Math.PI / 8.0);
      t1.getRotation(rotationVector);
      rotationVector.get(rotationVectorMatrix);

      SelectionMatrix3D selectionMatrix3d = new SelectionMatrix3D();
      selectionMatrix3d.selectZAxis(false);
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 3);
      selectionMatrix3d.getFullSelectionMatrixInFrame(null, selectionMatrix);

      DenseMatrix64F result = new DenseMatrix64F(3, 1);
      CommonOps.mult(selectionMatrix, rotationVectorMatrix, result);

      System.out.println(result);
      rotationVector.set(result);
      RotationMatrix rm = new RotationMatrix(rotationVector);
      System.out.println(rm);
   }
}
