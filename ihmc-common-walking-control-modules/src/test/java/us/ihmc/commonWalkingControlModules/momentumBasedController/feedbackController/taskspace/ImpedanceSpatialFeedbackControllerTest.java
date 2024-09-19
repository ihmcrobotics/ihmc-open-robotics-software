package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.ImpedanceSpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
import us.ihmc.convexOptimization.quadraticProgram.OASESConstrainedQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.PDSE3Stiffnesses;
import us.ihmc.robotics.controllers.pidGains.YoPD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPDSE3Stiffnesses;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPD3DStiffnesses;
import us.ihmc.robotics.Assert;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public final class ImpedanceSpatialFeedbackControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testBaseFrame()
   {
      double controlDT = 0.004;
      double simulationTime = 30.0;
      Random random = new Random(562968L);
      YoRegistry registry = new YoRegistry("TestRegistry");

      // Create two floating joints. This test attempts to control the end effector body with respect to the moving base body.
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint joint1 = MultiBodySystemRandomTools.nextSixDoFJoint(random, "joint1", elevator);
      RigidBody baseBody = MultiBodySystemRandomTools.nextRigidBody(random, "baseBody", joint1);
      SixDoFJoint joint2 = MultiBodySystemRandomTools.nextSixDoFJoint(random, "joint2", baseBody);
      RigidBody endEffector = MultiBodySystemRandomTools.nextRigidBody(random, "endEffector", joint2);

      // Create the feedback controller for the end effector.
      JointBasics[] joints = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, joints, centerOfMassFrame, null, null, registry);
      toolbox.setupForInverseDynamicsSolver(null);
      FeedbackControllerToolbox feedbackControllerToolbox = new FeedbackControllerToolbox(registry);
      ImpedanceSpatialFeedbackController impedanceSpatialFeedbackController = new ImpedanceSpatialFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      // Scramble the joint states.
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      elevator.updateFramesRecursively();

      // Create a command with baseBody as base and desired values in the baseBody frame.
      FramePose3D desiredPose = EuclidFrameRandomTools.nextFramePose3D(random, baseBody.getBodyFixedFrame());
      SpatialVector zero = new SpatialVector(desiredPose.getReferenceFrame());
      PDSE3Stiffnesses gains = new DefaultPDSE3Stiffnesses();
      gains.setPositionProportionalStiffnesses(2.0);
      gains.setPositionDerivativeStiffnesses(Double.NaN);
      gains.setOrientationProportionalStiffnesses(2.0);
      gains.setOrientationDerivativeStiffnesses(Double.NaN);
      ImpedanceSpatialFeedbackControlCommand impedanceSpatialFeedbackControlCommand = new ImpedanceSpatialFeedbackControlCommand();
      impedanceSpatialFeedbackControlCommand.set(baseBody, endEffector);
      impedanceSpatialFeedbackControlCommand.setGains(gains);
      impedanceSpatialFeedbackControlCommand.setInverseDynamics(desiredPose, zero, zero);
      impedanceSpatialFeedbackController.submitFeedbackControlCommand(impedanceSpatialFeedbackControlCommand);
      impedanceSpatialFeedbackController.setEnabled(true);

      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      NativeQPInputTypeA motionQPInput = new NativeQPInputTypeA(MultiBodySystemTools.computeDegreesOfFreedom(joints));
      DMatrixRMaj jointAccelerations = new DMatrixRMaj(0, 0);
      double damping = 0.001;
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      for (int i = 0; i < simulationTime / controlDT; i++)
      {
         impedanceSpatialFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand spatialAccelerationCommand = impedanceSpatialFeedbackController.getInverseDynamicsOutput();
         Assert.assertTrue(motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput));
         NativeCommonOps.solveDamped(new DMatrixRMaj(motionQPInput.getTaskJacobian()), new DMatrixRMaj(motionQPInput.getTaskObjective()), damping, jointAccelerations);
         integrator.integrateJointAccelerations(joints, jointAccelerations);
         integrator.integrateJointVelocities(joints, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(joints, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
         elevator.updateFramesRecursively();
      }

      // Assert pose is close to desired
      FramePose3D pose = new FramePose3D(endEffector.getBodyFixedFrame());
      pose.changeFrame(desiredPose.getReferenceFrame());
      EuclidCoreTestTools.assertGeometricallyEquals(desiredPose, pose, 1.0E-10);
   }

//   Not a fair test for an Impedance controller that is done with a SpringDamper System.
//   It will vibrate around the point, but it will converge in the end.
//   Now low stiffness to it is still approaching the point.
   @Test
   public void testConvergence() throws Exception
   {
      Random random = new Random(5641654L);

      int numberOfJoints = 10;
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = EuclidCoreRandomTools.nextVector3D(random, 1.0);

      YoRegistry registry = new YoRegistry("Dummy");
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      RigidBodyBasics elevator = randomFloatingChain.getElevator();
      RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();
      FramePoint3D bodyFixedPointToControl = EuclidFrameRandomTools.nextFramePoint3D(random, endEffector.getBodyFixedFrame(), 1.0, 1.0, 1.0);

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, 0.0, 1.0, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();
      FramePoint3D desiredPosition = new FramePoint3D();
      desiredPosition.setIncludingFrame(bodyFixedPointToControl);
      desiredPosition.changeFrame(worldFrame);
      FrameQuaternion desiredOrientation = new FrameQuaternion();
      desiredOrientation.setToZero(bodyFixedPointToControl.getReferenceFrame());
      desiredOrientation.changeFrame(worldFrame);
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, 0.0, 1.0, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      JointBasics[] jointsToOptimizeFor = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;


      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, jointsToOptimizeFor, centerOfMassFrame, null,
                                                                            null, registry);
      toolbox.setupForInverseDynamicsSolver(null);
      FeedbackControllerToolbox feedbackControllerToolbox = new FeedbackControllerToolbox(registry);
      ImpedanceSpatialFeedbackController impedanceSpatialFeedbackController = new ImpedanceSpatialFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      ImpedanceSpatialFeedbackControlCommand impedanceSpatialFeedbackControlCommand = new ImpedanceSpatialFeedbackControlCommand();
      impedanceSpatialFeedbackControlCommand.set(elevator, endEffector);
      DefaultPDSE3Stiffnesses gains = new DefaultPDSE3Stiffnesses();
      gains.getPositionStiffnesses().setProportialAndDerivativeStiffnesses(0.1, Double.NaN);
      gains.getOrientationStiffnesses().setProportialAndDerivativeStiffnesses(0.1, Double.NaN);
      impedanceSpatialFeedbackControlCommand.setGains(gains);
      impedanceSpatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedPointToControl);
      impedanceSpatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredPosition, new FrameVector3D(worldFrame), new FrameVector3D(worldFrame), new FrameVector3D(worldFrame), new FrameVector3D(worldFrame));
      impedanceSpatialFeedbackController.submitFeedbackControlCommand(impedanceSpatialFeedbackControlCommand);
      impedanceSpatialFeedbackController.setEnabled(true);

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      NativeQPInputTypeA motionQPInput = new NativeQPInputTypeA(numberOfDoFs);
      LinearSolverDense<DMatrixRMaj> pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
      DMatrixRMaj jInverse = new DMatrixRMaj(numberOfDoFs, 6);
      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      DMatrixRMaj jointAccelerations = new DMatrixRMaj(numberOfDoFs, 1);
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      FramePoint3D currentPosition = new FramePoint3D();
      FrameQuaternion currentOrientation = new FrameQuaternion();
      FrameQuaternion differenceOrientation = new FrameQuaternion();

      FrameVector3D positionError = new FrameVector3D();
      FrameVector3D rotationError = new FrameVector3D();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         impedanceSpatialFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand output = impedanceSpatialFeedbackController.getInverseDynamicsOutput();

         motionQPInputCalculator.convertSpatialAccelerationCommand(output, motionQPInput);
         pseudoInverseSolver.setA(new DMatrixRMaj(motionQPInput.taskJacobian));
         pseudoInverseSolver.invert(jInverse);
         CommonOps_DDRM.mult(jInverse, new DMatrixRMaj(motionQPInput.taskObjective), jointAccelerations);

         integrator.integrateJointAccelerations(jointsToOptimizeFor, jointAccelerations);
         integrator.integrateJointVelocities(jointsToOptimizeFor, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.VELOCITY, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
         elevator.updateFramesRecursively();

         currentPosition.setIncludingFrame(bodyFixedPointToControl);
         currentPosition.changeFrame(worldFrame);
         positionError.sub(desiredPosition, currentPosition);

         currentOrientation.setToZero(bodyFixedPointToControl.getReferenceFrame());
         currentOrientation.changeFrame(worldFrame);

         differenceOrientation.difference(desiredOrientation, currentOrientation);
         differenceOrientation.normalizeAndLimitToPi();
         differenceOrientation.getRotationVector(rotationError);

         errorMagnitude = Math.sqrt(positionError.lengthSquared() + rotationError.lengthSquared());
         boolean isErrorReducing = errorMagnitude < previousErrorMagnitude;
         assertTrue(isErrorReducing);
         previousErrorMagnitude = errorMagnitude;
      }
   }

//   Same as the previous test (testConvergence()). This test is not fair for an Impedance Controller.
   @Test
   public void testConvergenceWithJerryQP() throws Exception
   {
      Random random = new Random(54654L);

      int numberOfJoints = 10;
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = EuclidCoreRandomTools.nextVector3D(random, 1.0);

      YoRegistry registry = new YoRegistry("Dummy");
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      RigidBodyBasics elevator = randomFloatingChain.getElevator();
      RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();
      FramePoint3D bodyFixedPointToControl = EuclidFrameRandomTools.nextFramePoint3D(random, endEffector.getBodyFixedFrame(), 1.0, 1.0, 1.0);

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, 0.0, 1.0, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();
      FramePoint3D desiredPosition = new FramePoint3D();
      desiredPosition.setIncludingFrame(bodyFixedPointToControl);
      desiredPosition.changeFrame(worldFrame);
      FrameQuaternion desiredOrientation = new FrameQuaternion();
      desiredOrientation.setToZero(bodyFixedPointToControl.getReferenceFrame());
      desiredOrientation.changeFrame(worldFrame);
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, 0.0, 1.0, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      JointBasics[] jointsToOptimizeFor = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, jointsToOptimizeFor, centerOfMassFrame, null,
                                                                            null, registry);
      toolbox.setupForInverseDynamicsSolver(null);
      FeedbackControllerToolbox feedbackControllerToolbox = new FeedbackControllerToolbox(registry);
      ImpedanceSpatialFeedbackController impedanceSpatialFeedbackController = new ImpedanceSpatialFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      ImpedanceSpatialFeedbackControlCommand impedanceSpatialFeedbackControlCommand = new ImpedanceSpatialFeedbackControlCommand();
      impedanceSpatialFeedbackControlCommand.set(elevator, endEffector);
      DefaultPDSE3Stiffnesses gains = new DefaultPDSE3Stiffnesses();
      gains.getPositionStiffnesses().setProportialAndDerivativeStiffnesses(1.0, Double.NaN);
      gains.getOrientationStiffnesses().setProportialAndDerivativeStiffnesses(1.0, Double.NaN);
      impedanceSpatialFeedbackControlCommand.setGains(gains);
      impedanceSpatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedPointToControl);
      impedanceSpatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredPosition, new FrameVector3D(worldFrame), new FrameVector3D(worldFrame), new FrameVector3D(worldFrame), new FrameVector3D(worldFrame));
      impedanceSpatialFeedbackController.submitFeedbackControlCommand(impedanceSpatialFeedbackControlCommand);
      impedanceSpatialFeedbackController.setEnabled(true);

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      NativeQPInputTypeA motionQPInput = new NativeQPInputTypeA(numberOfDoFs);
      LinearSolverDense<DMatrixRMaj> pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
      DMatrixRMaj jInverse = new DMatrixRMaj(numberOfDoFs, 6);
      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      DMatrixRMaj jointAccelerations = new DMatrixRMaj(numberOfDoFs, 1);
      DMatrixRMaj jointAccelerationsFromJerryQP = new DMatrixRMaj(numberOfDoFs, 1);
      DMatrixRMaj jointAccelerationsFromQPOASES = new DMatrixRMaj(numberOfDoFs, 1);
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      SimpleEfficientActiveSetQPSolver jerryQPSolver = new SimpleEfficientActiveSetQPSolver();
      OASESConstrainedQPSolver oasesQPSolver = new OASESConstrainedQPSolver();

      DMatrixRMaj solverInput_H = new DMatrixRMaj(numberOfDoFs, numberOfDoFs);
      DMatrixRMaj solverInput_f = new DMatrixRMaj(numberOfDoFs, 1);
      DMatrixRMaj solverInput_Aeq = new DMatrixRMaj(0, numberOfDoFs);
      DMatrixRMaj solverInput_beq = new DMatrixRMaj(0, 1);
      DMatrixRMaj solverInput_Ain = new DMatrixRMaj(0, numberOfDoFs);
      DMatrixRMaj solverInput_bin = new DMatrixRMaj(0, 1);
      DMatrixRMaj solverInput_lb = new DMatrixRMaj(numberOfDoFs, 1);
      DMatrixRMaj solverInput_ub = new DMatrixRMaj(numberOfDoFs, 1);
      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      DMatrixRMaj tempJtW = new DMatrixRMaj(numberOfDoFs, 6);

      FramePoint3D currentPosition = new FramePoint3D();
      FrameQuaternion currentOrientation = new FrameQuaternion();
      FrameQuaternion differenceOrientation = new FrameQuaternion();

      FrameVector3D positionError = new FrameVector3D();
      FrameVector3D rotationError = new FrameVector3D();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         impedanceSpatialFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand output = impedanceSpatialFeedbackController.getInverseDynamicsOutput();
         motionQPInputCalculator.convertSpatialAccelerationCommand(output, motionQPInput);

         MatrixTools.scaleTranspose(1.0, new DMatrixRMaj(motionQPInput.taskJacobian), tempJtW); // J^T W
         CommonOps_DDRM.mult(tempJtW, new DMatrixRMaj(motionQPInput.taskJacobian), solverInput_H); // H = J^T W J
         CommonOps_DDRM.mult(tempJtW, new DMatrixRMaj(motionQPInput.taskObjective), solverInput_f); // f = - J^T W xDDot
         CommonOps_DDRM.scale(-1.0, solverInput_f);

         for (int diag = 0; diag < numberOfDoFs; diag++)
            solverInput_H.add(diag, diag, 1e-8);

         jerryQPSolver.clear();
         jerryQPSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
         jerryQPSolver.solve(jointAccelerationsFromJerryQP);
         oasesQPSolver.solve(solverInput_H, solverInput_f, solverInput_Aeq, solverInput_beq, solverInput_Ain, solverInput_bin, solverInput_lb, solverInput_ub,
                             jointAccelerationsFromQPOASES, true);

         pseudoInverseSolver.setA(new DMatrixRMaj(motionQPInput.taskJacobian));
         pseudoInverseSolver.invert(jInverse);
         CommonOps_DDRM.mult(jInverse, new DMatrixRMaj(motionQPInput.taskObjective), jointAccelerations);

         integrator.integrateJointAccelerations(jointsToOptimizeFor, jointAccelerationsFromJerryQP);
         integrator.integrateJointVelocities(jointsToOptimizeFor, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.VELOCITY, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
         elevator.updateFramesRecursively();

         assertArrayEquals(jointAccelerations.data, jointAccelerationsFromJerryQP.data, 1.0e-4);
         assertArrayEquals(jointAccelerations.data, jointAccelerationsFromQPOASES.data, 3.0e-1);

         currentPosition.setIncludingFrame(bodyFixedPointToControl);
         currentPosition.changeFrame(worldFrame);
         positionError.sub(desiredPosition, currentPosition);

         currentOrientation.setToZero(bodyFixedPointToControl.getReferenceFrame());
         currentOrientation.changeFrame(worldFrame);

         differenceOrientation.difference(desiredOrientation, currentOrientation);
         differenceOrientation.normalizeAndLimitToPi();
         differenceOrientation.getRotationVector(rotationError);

         errorMagnitude = Math.sqrt(positionError.lengthSquared() + rotationError.lengthSquared());
         boolean isErrorReducing = errorMagnitude < previousErrorMagnitude;
         assertTrue(isErrorReducing);
         previousErrorMagnitude = errorMagnitude;
      }
   }
}
