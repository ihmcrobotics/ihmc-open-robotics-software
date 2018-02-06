package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSO3TrajectoryPoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class EndToEndChestTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double EPSILON_FOR_DESIREDS = 5.0e-4;

   private DRCSimulationTestHelper drcSimulationTestHelper;

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

      ChestTrajectoryMessage lookStraightAheadMessage = new ChestTrajectoryMessage(trajectoryTime, lookStraightAhead, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookStraightAheadMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookLeftMessage = new ChestTrajectoryMessage(trajectoryTime, lookLeft, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookLeftMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookRightMessage = new ChestTrajectoryMessage(trajectoryTime, lookRight, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookRightMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookRightMessage);

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

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertControlErrorIsLow(scs, chest, 1.0e-2);
      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs, chest);
   }

   public void testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

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
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

//      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
//      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
//      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
//      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
//      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(pelvisZUpFrame,Math.PI / 8.0, Math.PI / 8.0, Math.PI / 16.0);

      Quaternion desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame, pelvisZUpFrame);
      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      selectionMatrix3D.selectZAxis(false);
      selectionMatrix3D.selectYAxis(false);
      selectionMatrix3D.selectXAxis(false);
      selectionMatrix3D.setSelectionFrame(pelvisZUpFrame);
      chestTrajectoryMessage.setSelectionMatrix(selectionMatrix3D);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
//      desiredRandomChestOrientation.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      Vector3D rotationVector = new Vector3D();
      desiredChestOrientation.get(rotationVector);
      DenseMatrix64F rotationVectorMatrix = new DenseMatrix64F(3, 1);
      rotationVector.get(rotationVectorMatrix);

      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 3);
      selectionMatrix3D.getFullSelectionMatrixInFrame(pelvisZUpFrame, selectionMatrix);

      DenseMatrix64F result = new DenseMatrix64F(3, 1);
      CommonOps.mult(selectionMatrix, rotationVectorMatrix, result);
      rotationVector.set(result);
      desiredChestOrientation.set(rotationVector);

      System.out.println(desiredChestOrientation);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
//      assertControlErrorIsLow(scs, chest, 1.0e-2);

      assertNumberOfWaypoints(2, scs, chest);
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
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);

      for(int i = 0; i < 50; i++)
      {
         ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
         RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
         FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
         desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);

         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame, pelvisZUpFrame);
         chestTrajectoryMessage.setExecutionMode(ExecutionMode.OVERRIDE, -1);

         WeightMatrix3D weightMatrix = new WeightMatrix3D();

         double xWeight = random.nextDouble();
         double yWeight = random.nextDouble();
         double zWeight = random.nextDouble();

         weightMatrix.setWeights(xWeight, yWeight, zWeight);
         chestTrajectoryMessage.setWeightMatrix(weightMatrix);
         drcSimulationTestHelper.send(chestTrajectoryMessage);

         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 4.0));
         assertWeightsMatch(xWeight, yWeight, zWeight, chest, scs);
      }

      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame, pelvisZUpFrame);
      chestTrajectoryMessage.setExecutionMode(ExecutionMode.OVERRIDE, -1);

      WeightMatrix3D weightMatrix = new WeightMatrix3D();

      double xWeight = Double.NaN;
      double yWeight = Double.NaN;
      double zWeight = Double.NaN;

      weightMatrix.setWeights(xWeight, yWeight, zWeight);
      chestTrajectoryMessage.setWeightMatrix(weightMatrix);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 4.0));
      assertAngularWeightsMatchDefault(chest, scs);

   }

   private void assertAngularWeightsMatchDefault(RigidBody rigidBody, SimulationConstructionSet scs)
   {
      String prefix = rigidBody.getName() + "Taskspace";
      YoDouble angularWeightX = (YoDouble) scs.getVariable(prefix + "CurrentAngularWeightX");
      YoDouble angularWeightY = (YoDouble) scs.getVariable(prefix + "CurrentAngularWeightY");
      YoDouble angularWeightZ = (YoDouble) scs.getVariable(prefix + "CurrentAngularWeightZ");
      YoDouble defaultAngularWeightX = (YoDouble) scs.getVariable("ChestAngularWeightX");
      YoDouble defaultAngularWeightY = (YoDouble) scs.getVariable("ChestAngularWeightY");
      YoDouble defaultAngularWeightZ = (YoDouble) scs.getVariable("ChestAngularWeightZ");
      assertEquals(defaultAngularWeightX.getDoubleValue(), angularWeightX.getDoubleValue(), 1e-8);
      assertEquals(defaultAngularWeightY.getDoubleValue(), angularWeightY.getDoubleValue(), 1e-8);
      assertEquals(defaultAngularWeightZ.getDoubleValue(), angularWeightZ.getDoubleValue(), 1e-8);

   }

   private void assertWeightsMatch(double xWeight, double yWeight, double zWeight, RigidBody rigidBody, SimulationConstructionSet scs)
   {
//      AngularWeightX
      String prefix = rigidBody.getName() + "Taskspace";
      YoDouble angularWeightX = (YoDouble) scs.getVariable(prefix + "CurrentAngularWeightX");
      YoDouble angularWeightY = (YoDouble) scs.getVariable(prefix + "CurrentAngularWeightY");
      YoDouble angularWeightZ = (YoDouble) scs.getVariable(prefix + "CurrentAngularWeightZ");
//      YoDouble linearWeightX = (YoDouble) scs.getVariable(prefix + "CurrentLinearWeightX");
//      YoDouble linearWeightY = (YoDouble) scs.getVariable(prefix + "CurrentLinearWeightY");
//      YoDouble linearWeightZ = (YoDouble) scs.getVariable(prefix + "CurrentLinearWeightZ");

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
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

//      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
//      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
//      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
//      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
//      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(pelvisZUpFrame,Math.PI / 16.0, Math.PI / 20.0, Math.PI / 24.0);

      Quaternion desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame, pelvisZUpFrame);
      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      selectionMatrix3D.selectZAxis(random.nextBoolean());
      selectionMatrix3D.selectYAxis(random.nextBoolean());
      selectionMatrix3D.selectXAxis(random.nextBoolean());
      selectionMatrix3D.setSelectionFrame(pelvisZUpFrame);
      chestTrajectoryMessage.setSelectionMatrix(selectionMatrix3D);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
//      desiredRandomChestOrientation.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      Vector3D rotationVector = new Vector3D();
      desiredChestOrientation.get(rotationVector);
      DenseMatrix64F rotationVectorMatrix = new DenseMatrix64F(3, 1);
      rotationVector.get(rotationVectorMatrix);

      DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, 3);
      selectionMatrix3D.getFullSelectionMatrixInFrame(pelvisZUpFrame, selectionMatrix);

      DenseMatrix64F result = new DenseMatrix64F(3, 1);
      CommonOps.mult(selectionMatrix, rotationVectorMatrix, result);
      rotationVector.set(result);
      desiredChestOrientation.set(rotationVector);

      System.out.println(desiredChestOrientation);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
//      assertControlErrorIsLow(scs, chest, 1.0e-2);

      assertNumberOfWaypoints(2, scs, chest);
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

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

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
      RigidBody chest = fullRobotModel.getChest();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUpFrame);
      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(ReferenceFrame.getWorldFrame());

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

         Quaternion desiredOrientation = new Quaternion(desiredChestOrientations[trajectoryPointIndex]);
         Vector3D desiredAngularVelocity = new Vector3D(desiredChestAngularVelocities[trajectoryPointIndex]);

         chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
      }

      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      humanoidReferenceFrames.updateFrames();

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         desiredChestOrientations[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(pelvisZUpFrame);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(numberOfTrajectoryPoints + 1, scs, chest);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = chestTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex).getTime();
         SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs, chest);
         assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         desiredChestOrientations[trajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         desiredChestAngularVelocities[trajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
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
      RigidBody chest = fullRobotModel.getChest();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUpFrame);
      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(ReferenceFrame.getWorldFrame());

      FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
      FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
         double roll = amp * Math.sin(t * w);
         double rollDot = w * amp * Math.cos(t * w);
         desiredChestOrientations[trajectoryPointIndex] = new FrameQuaternion();
         desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, 0.0, roll);
         desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector3D();

         if (trajectoryPointIndex == 0 || trajectoryPointIndex == numberOfTrajectoryPoints - 1)
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, 0.0, 0.0);
         else
            desiredChestAngularVelocities[trajectoryPointIndex].set(rollDot, 0.0, 0.0);

         Quaternion desiredOrientation = new Quaternion(desiredChestOrientations[trajectoryPointIndex]);
         Vector3D desiredAngularVelocity = new Vector3D(desiredChestAngularVelocities[trajectoryPointIndex]);

         chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
      }

      drcSimulationTestHelper.send(chestTrajectoryMessage);

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

      assertNumberOfWaypoints(Math.min(RigidBodyTaskspaceControlState.maxPoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs, chest);

      double timeInState = 0.0;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = chestTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex).getTime();
         SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs, chest);
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

      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
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
      RigidBody chest = fullRobotModel.getChest();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUpFrame);

      FrameQuaternion[] desiredChestOrientations = new FrameQuaternion[numberOfTrajectoryPoints];
      FrameVector3D[] desiredChestAngularVelocities = new FrameVector3D[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
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

         Quaternion desiredOrientation = new Quaternion(desiredChestOrientations[trajectoryPointIndex]);
         Vector3D desiredAngularVelocity = new Vector3D(desiredChestAngularVelocities[trajectoryPointIndex]);

         chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity, pelvisZUpFrame);
      }

      drcSimulationTestHelper.send(chestTrajectoryMessage);

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

      assertNumberOfWaypoints(Math.min(RigidBodyTaskspaceControlState.maxPoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs, chest);

      double timeInState = 0.0;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = chestTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex).getTime();
         SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs, chest);
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

      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs, chest);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint * numberOfTrajectoryPoints + 0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   public void testQueuedMessages() throws Exception
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

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();

      double timePerWaypoint = 0.05;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody chest = fullRobotModel.getChest();


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
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
         chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUpFrame);
         chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(ReferenceFrame.getWorldFrame());

         chestTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            chestTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint;
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

            Quaternion desiredOrientation = new Quaternion(desiredChestOrientations[trajectoryPointIndex]);
            Vector3D desiredAngularVelocity = new Vector3D(desiredChestAngularVelocities[trajectoryPointIndex]);

            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
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


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      int totalPoints = numberOfMessages * numberOfTrajectoryPoints + 1;
      assertNumberOfWaypoints(totalPoints, scs, chest);

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         ChestTrajectoryMessage chestTrajectoryMessage = messageList.get(messageIndex);
         double simulationTime = chestTrajectoryMessage.getLastTrajectoryPoint().getTime();
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

      assertNumberOfWaypoints(pointsInLastTrajectory, scs, chest);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
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
      RigidBody chest = fullRobotModel.getChest();
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
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
         chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUpFrame);
         chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(ReferenceFrame.getWorldFrame());
         chestTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
         {
            long previousMessageId = id - 1;
            if (messageIndex == numberOfMessages - 1)
               previousMessageId = id + 100; // Bad ID

            chestTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, previousMessageId);
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

            Quaternion desiredOrientation = new Quaternion(desiredChestOrientations[trajectoryPointIndex]);
            Vector3D desiredAngularVelocity = new Vector3D(desiredChestAngularVelocities[trajectoryPointIndex]);

            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
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
      assertEquals(defaultControlMode, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
      assertNumberOfWaypoints(1, scs, chest);
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
      RigidBody chest = fullRobotModel.getChest();

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

      ChestTrajectoryMessage lookStraightAheadMessage = new ChestTrajectoryMessage(trajectoryTime, lookStraightAhead, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookStraightAheadMessage.setExecutionMode(ExecutionMode.OVERRIDE, -1);
      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      assertNumberOfWaypoints(2, scs, chest);

      ChestTrajectoryMessage lookRightMessage = new ChestTrajectoryMessage(trajectoryTime, lookRight, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookRightMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookRightMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      assertNumberOfWaypoints(3, scs, chest);

      ChestTrajectoryMessage LookLeftMessage = new ChestTrajectoryMessage(trajectoryTime, lookLeft, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      LookLeftMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(LookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      assertNumberOfWaypoints(4, scs, chest);

      ChestTrajectoryMessage LookLeftMessageWithChangeTrajFrame = new ChestTrajectoryMessage(trajectoryTime, lookLeft, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      LookLeftMessageWithChangeTrajFrame.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(LookLeftMessageWithChangeTrajFrame);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      RigidBodyControlMode defaultControlMode = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlMode == null)
      {
         defaultControlMode = RigidBodyControlMode.JOINTSPACE;
      }
      assertEquals(defaultControlMode, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
      assertNumberOfWaypoints(1, scs, chest);

      drcSimulationTestHelper.send(lookRightMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      assertNumberOfWaypoints(2, scs, chest);

      drcSimulationTestHelper.send(LookLeftMessageWithChangeTrajFrame);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));

      assertEquals(defaultControlMode, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
      assertNumberOfWaypoints(1, scs, chest);

      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      assertNumberOfWaypoints(2, scs, chest);

      LookLeftMessageWithChangeTrajFrame.setExecutionMode(ExecutionMode.OVERRIDE, -1);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2.0));
      drcSimulationTestHelper.send(LookLeftMessageWithChangeTrajFrame);
      assertNumberOfWaypoints(2, scs, chest);


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

      ChestTrajectoryMessage lookStraightAheadMessage = new ChestTrajectoryMessage(trajectoryTime, lookStraightAhead, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookStraightAheadMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookLeftMessage = new ChestTrajectoryMessage(trajectoryTime, lookLeft, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookLeftMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookRightMessage = new ChestTrajectoryMessage(trajectoryTime, lookRight, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookRightMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookRightMessage);

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
      RigidBody chest = fullRobotModel.getChest();


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
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
         chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(pelvisZUpFrame);
         chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(ReferenceFrame.getWorldFrame());

         chestTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            chestTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
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

            Quaternion desiredOrientation = new Quaternion(desiredChestOrientations[trajectoryPointIndex]);
            Vector3D desiredAngularVelocity = new Vector3D(desiredChestAngularVelocities[trajectoryPointIndex]);

            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
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

      RigidBody pelvis = fullRobotModel.getPelvis();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);

      Random random = new Random(21651L);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);

      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      trajectoryTime = 0.5;
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      assertNumberOfWaypoints(2, scs, chest);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs, chest);

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
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);

      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);

      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);

      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(pelvisZUpFrame);

      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      Quaternion desiredOrientationBeforeStop = findControllerDesiredOrientation(scs, chest);

      assertEquals(RigidBodyControlMode.TASKSPACE, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
      assertNumberOfWaypoints(2, scs, chest);

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.send(stopAllTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Quaternion desiredOrientationAfterStop = findControllerDesiredOrientation(scs, chest);

      RigidBodyControlMode defaultControlMode = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlMode == null)
      {
         defaultControlMode = RigidBodyControlMode.JOINTSPACE;
      }
      assertEquals(defaultControlMode, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
      assertNumberOfWaypoints(1, scs, chest);

      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientationBeforeStop, desiredOrientationAfterStop, 1.0e-3);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   public static Quaternion findControllerDesiredOrientation(SimulationConstructionSet scs, RigidBody chest)
   {
      String chestPrefix = chest.getName();
      return findQuat4d("FeedbackControllerToolbox", chestPrefix + "DesiredOrientation", scs);
   }

   public static Vector3D findControllerDesiredAngularVelocity(SimulationConstructionSet scs, RigidBody chest)
   {
      String chestPrefix = chest.getName();
      return findVector3d("FeedbackControllerToolbox", chestPrefix + "DesiredAngularVelocity", scs);
   }

   public static int findControllerNumberOfWaypoints(SimulationConstructionSet scs, RigidBody chest)
   {
      String chestPrefix = chest.getName();
      int numberOfWaypoints = ((YoInteger) scs.getVariable(chestPrefix + "TaskspaceControlModule", chestPrefix + "TaskspaceNumberOfPoints")).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsInGenerator(SimulationConstructionSet scs, RigidBody chest)
   {
      String chestPrefix = chest.getName();
      int numberOfWaypoints = ((YoInteger) scs.getVariable(chestPrefix + "TaskspaceControlModule", chestPrefix + "TaskspaceNumberOfPointsInGenerator")).getIntegerValue();
      return numberOfWaypoints;
   }

   public static Vector3D findControlErrorRotationVector(SimulationConstructionSet scs, RigidBody chest)
   {
      String chestPrefix = chest.getName();
      String nameSpace = FeedbackControllerToolbox.class.getSimpleName();
      String varName = chestPrefix + "ErrorRotationVector";
      return findVector3d(nameSpace, varName, scs);
   }

   public static SimpleSO3TrajectoryPoint findTrajectoryPoint(int trajectoryPointIndex, SimulationConstructionSet scs, RigidBody chestRigidBody)
   {
      String chestPrefix = chestRigidBody.getName();
      String orientationTrajectoryName = chestPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = chestPrefix + "Time";
      String orientationName = chestPrefix + "Orientation";
      String angularVelocityName = chestPrefix + "AngularVelocity";

      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setTime(scs.getVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSO3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSO3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSO3TrajectoryPoint;
   }

   public static SimpleSO3TrajectoryPoint findCurrentDesiredTrajectoryPoint(SimulationConstructionSet scs, RigidBody chest)
   {
      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setOrientation(findControllerDesiredOrientation(scs, chest));
      simpleSO3TrajectoryPoint.setAngularVelocity(findControllerDesiredAngularVelocity(scs, chest));
      return simpleSO3TrajectoryPoint;
   }

   public static void assertControlErrorIsLow(SimulationConstructionSet scs, RigidBody chest, double errorTolerance)
   {
      Vector3D error = findControlErrorRotationVector(scs, chest);
      boolean isErrorLow = error.length() <= errorTolerance;
      assertTrue("Error: " + error, isErrorLow);
   }

   public static void assertSingleWaypointExecuted(FrameQuaternion desiredChestOrientation, SimulationConstructionSet scs, RigidBody chest)
   {
      assertNumberOfWaypoints(2, scs, chest);

      Quaternion controllerDesiredOrientation = findControllerDesiredOrientation(scs, chest);
      FrameQuaternion controllerDesiredFrameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), controllerDesiredOrientation);
      controllerDesiredFrameOrientation.changeFrame(desiredChestOrientation.getReferenceFrame());

      EuclidCoreTestTools.assertQuaternionEquals(desiredChestOrientation, controllerDesiredFrameOrientation, EPSILON_FOR_DESIREDS);
   }

   public static void assertNumberOfWaypoints(int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs, RigidBody chest)
   {
      assertEquals(expectedNumberOfTrajectoryPoints, findControllerNumberOfWaypoints(scs, chest));
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
