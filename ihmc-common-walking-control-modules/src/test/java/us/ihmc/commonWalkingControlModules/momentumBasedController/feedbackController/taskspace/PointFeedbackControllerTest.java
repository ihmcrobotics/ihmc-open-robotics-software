package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.robotics.Assert.assertArrayEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.NormOps;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.convexOptimization.quadraticProgram.OASESConstrainedQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public final class PointFeedbackControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testBaseFrame()
   {
      double controlDT = 0.004;
      Random random = new Random(562968L);
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");

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
      PointFeedbackController pointFeedbackController = new PointFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      // Scramble the joint states.
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      elevator.updateFramesRecursively();

      // Create a command with baseBody as base and desired values in the baseBody frame.
      FramePoint3D desiredPosition = EuclidFrameRandomTools.nextFramePoint3D(random, baseBody.getBodyFixedFrame());
      FrameVector3D zero = new FrameVector3D(desiredPosition.getReferenceFrame());
      PID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(500.0);
      gains.setDerivativeGains(50.0);
      PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
      pointFeedbackControlCommand.set(baseBody, endEffector);
      pointFeedbackControlCommand.setGains(gains);
      pointFeedbackController.setEnabled(true);

      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      QPInput motionQPInput = new QPInput(MultiBodySystemTools.computeDegreesOfFreedom(joints));
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(0, 0);
      double damping = 0.001;
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      for (int i = 0; i < 1000; i++)
      {
         // Keep updating the desired since it is converted to world frame inside the command.
         // TODO: This can be removed once we start saving the desired values in their trajectory frames.
         pointFeedbackControlCommand.setInverseDynamics(desiredPosition, zero, zero);
         pointFeedbackController.submitFeedbackControlCommand(pointFeedbackControlCommand);
         pointFeedbackController.computeInverseDynamics();

         SpatialAccelerationCommand spatialAccelerationCommand = pointFeedbackController.getInverseDynamicsOutput();
         Assert.assertTrue(motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput));
         NativeCommonOps.solveDamped(motionQPInput.getTaskJacobian(), motionQPInput.getTaskObjective(), damping, jointAccelerations);
         integrator.integrateJointAccelerations(joints, jointAccelerations);
         integrator.integrateJointVelocities(joints, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(joints, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
         elevator.updateFramesRecursively();
      }

      // Assert position is close to desired
      FramePoint3D position = new FramePoint3D(endEffector.getBodyFixedFrame());
      position.changeFrame(desiredPosition.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(desiredPosition, position, 1.0E-3);
   }

   @Test
   public void testConvergence() throws Exception
   {
      Random random = new Random(5641654L);

      int numberOfJoints = 10;
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomGeometry.nextVector3D(random, 1.0);

      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      RigidBodyBasics elevator = randomFloatingChain.getElevator();
      RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();
      FramePoint3D bodyFixedPointToControl = EuclidFrameRandomTools.nextFramePoint3D(random, endEffector.getBodyFixedFrame(), 1.0, 1.0, 1.0);

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();
      FramePoint3D desiredPosition = new FramePoint3D();
      desiredPosition.setIncludingFrame(bodyFixedPointToControl);
      desiredPosition.changeFrame(worldFrame);
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      JointBasics[] jointsToOptimizeFor = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, jointsToOptimizeFor, centerOfMassFrame, null, null,
                                                                            registry);
      toolbox.setupForInverseDynamicsSolver(null);
      FeedbackControllerToolbox feedbackControllerToolbox = new FeedbackControllerToolbox(registry);
      PointFeedbackController pointFeedbackController = new PointFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
      pointFeedbackControlCommand.set(elevator, endEffector);
      PID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(100.0);
      gains.setDerivativeGains(50.0);
      pointFeedbackControlCommand.setGains(gains);
      pointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
      pointFeedbackControlCommand.setInverseDynamics(desiredPosition, new FrameVector3D(worldFrame), new FrameVector3D(worldFrame));
      pointFeedbackController.submitFeedbackControlCommand(pointFeedbackControlCommand);
      pointFeedbackController.setEnabled(true);

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      QPInput motionQPInput = new QPInput(numberOfDoFs);
      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      DenseMatrix64F jInverse = new DenseMatrix64F(numberOfDoFs, 6);
      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      FramePoint3D currentPosition = new FramePoint3D();
      FrameVector3D errorVector = new FrameVector3D();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         pointFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand output = pointFeedbackController.getInverseDynamicsOutput();

         motionQPInputCalculator.convertSpatialAccelerationCommand(output, motionQPInput);
         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.invert(jInverse);
         CommonOps.mult(jInverse, motionQPInput.taskObjective, jointAccelerations);

         integrator.integrateJointAccelerations(jointsToOptimizeFor, jointAccelerations);
         integrator.integrateJointVelocities(jointsToOptimizeFor, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.VELOCITY, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
         elevator.updateFramesRecursively();

         currentPosition.setIncludingFrame(bodyFixedPointToControl);
         currentPosition.changeFrame(worldFrame);
         errorVector.sub(desiredPosition, currentPosition);
         errorMagnitude = errorVector.length();
         boolean isErrorReducing = errorMagnitude < previousErrorMagnitude;
         assertTrue(isErrorReducing);
         previousErrorMagnitude = errorMagnitude;
      }
   }

   @Test
   public void testConvergenceWithJerryQP() throws Exception
   {
      Random random = new Random(5641654L);

      int numberOfJoints = 10;
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomGeometry.nextVector3D(random, 1.0);

      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
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
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, 0.0, 1.0, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      JointBasics[] jointsToOptimizeFor = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, jointsToOptimizeFor, centerOfMassFrame, null, null,
                                                                            registry);
      toolbox.setupForInverseDynamicsSolver(null);
      FeedbackControllerToolbox feedbackControllerToolbox = new FeedbackControllerToolbox(registry);
      PointFeedbackController pointFeedbackController = new PointFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
      pointFeedbackControlCommand.set(elevator, endEffector);
      PID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(10.0);
      gains.setDerivativeGains(5.0);
      pointFeedbackControlCommand.setGains(gains);
      pointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
      pointFeedbackControlCommand.setInverseDynamics(desiredPosition, new FrameVector3D(worldFrame), new FrameVector3D(worldFrame));
      pointFeedbackController.submitFeedbackControlCommand(pointFeedbackControlCommand);
      pointFeedbackController.setEnabled(true);

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      QPInput motionQPInput = new QPInput(numberOfDoFs);
      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      DenseMatrix64F jInverse = new DenseMatrix64F(numberOfDoFs, 6);
      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      DenseMatrix64F jointAccelerationsFromJerryQP = new DenseMatrix64F(numberOfDoFs, 1);
      DenseMatrix64F jointAccelerationsFromQPOASES = new DenseMatrix64F(numberOfDoFs, 1);
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      SimpleEfficientActiveSetQPSolver jerryQPSolver = new SimpleEfficientActiveSetQPSolver();
      OASESConstrainedQPSolver oasesQPSolver = new OASESConstrainedQPSolver(registry);

      DenseMatrix64F solverInput_H = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      DenseMatrix64F solverInput_f = new DenseMatrix64F(numberOfDoFs, 1);
      DenseMatrix64F solverInput_Aeq = new DenseMatrix64F(0, numberOfDoFs);
      DenseMatrix64F solverInput_beq = new DenseMatrix64F(0, 1);
      DenseMatrix64F solverInput_Ain = new DenseMatrix64F(0, numberOfDoFs);
      DenseMatrix64F solverInput_bin = new DenseMatrix64F(0, 1);
      DenseMatrix64F solverInput_lb = new DenseMatrix64F(numberOfDoFs, 1);
      DenseMatrix64F solverInput_ub = new DenseMatrix64F(numberOfDoFs, 1);
      CommonOps.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      DenseMatrix64F tempJtW = new DenseMatrix64F(numberOfDoFs, 3);

      FramePoint3D currentPosition = new FramePoint3D();
      FrameVector3D errorVector = new FrameVector3D();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         pointFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand output = pointFeedbackController.getInverseDynamicsOutput();
         motionQPInputCalculator.convertSpatialAccelerationCommand(output, motionQPInput);

         MatrixTools.scaleTranspose(1.0, motionQPInput.taskJacobian, tempJtW); // J^T W
         CommonOps.mult(tempJtW, motionQPInput.taskJacobian, solverInput_H); // H = J^T W J
         CommonOps.mult(tempJtW, motionQPInput.taskObjective, solverInput_f); // f = - J^T W xDDot
         CommonOps.scale(-1.0, solverInput_f);

         for (int diag = 0; diag < numberOfDoFs; diag++)
            solverInput_H.add(diag, diag, 1e-5);

         jerryQPSolver.clear();
         jerryQPSolver.setQuadraticCostFunction(solverInput_H, solverInput_f, 0.0);
         jerryQPSolver.solve(jointAccelerationsFromJerryQP, new DenseMatrix64F(1, 1), new DenseMatrix64F(1, 1));
         oasesQPSolver.solve(solverInput_H, solverInput_f, solverInput_Aeq, solverInput_beq, solverInput_Ain, solverInput_bin, solverInput_lb, solverInput_ub,
                             jointAccelerationsFromQPOASES, true);

         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.invert(jInverse);
         CommonOps.mult(jInverse, motionQPInput.taskObjective, jointAccelerations);

         integrator.integrateJointAccelerations(jointsToOptimizeFor, jointAccelerationsFromJerryQP);
         integrator.integrateJointVelocities(jointsToOptimizeFor, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.VELOCITY, integrator.getJointVelocities());
         MultiBodySystemTools.insertJointsState(jointsToOptimizeFor, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
         elevator.updateFramesRecursively();

         assertArrayEquals(jointAccelerations.data, jointAccelerationsFromJerryQP.data, 1.0e-4);
         assertArrayEquals(jointAccelerations.data, jointAccelerationsFromQPOASES.data, 2.0e-1);

         currentPosition.setIncludingFrame(bodyFixedPointToControl);
         currentPosition.changeFrame(worldFrame);
         errorVector.sub(desiredPosition, currentPosition);
         errorMagnitude = errorVector.length();
         boolean isErrorReducing = errorMagnitude < previousErrorMagnitude;
         assertTrue(isErrorReducing);
         previousErrorMagnitude = errorMagnitude;
      }
   }

   @Test
   public void testCompareAgainstSpatialController() throws Exception
   {
      Random random = new Random(5641654L);

      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      int numberOfRevoluteJoints = 10;
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      RigidBodyBasics elevator = randomFloatingChain.getElevator();
      RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      JointBasics[] jointsToOptimizeFor = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, jointsToOptimizeFor, centerOfMassFrame, null, null,
                                                                            registry);
      toolbox.setupForInverseDynamicsSolver(null);
      // Making the controllers to run with different instances of the toolbox so they don't share variables.
      PointFeedbackController pointFeedbackController = new PointFeedbackController(endEffector, toolbox, new FeedbackControllerToolbox(new YoVariableRegistry("Dummy")), registry);
      SpatialFeedbackController spatialFeedbackController = new SpatialFeedbackController(endEffector, toolbox, new FeedbackControllerToolbox(new YoVariableRegistry("Dummy")), registry);
      pointFeedbackController.setEnabled(true);
      spatialFeedbackController.setEnabled(true);

      PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
      pointFeedbackControlCommand.set(elevator, endEffector);
      PID3DGains positionGains = new DefaultPID3DGains();

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
      spatialFeedbackControlCommand.set(elevator, endEffector);
      spatialFeedbackControlCommand.getSpatialAccelerationCommand().setSelectionMatrixForLinearControl();

      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      QPInput pointMotionQPInput = new QPInput(toolbox.getJointIndexHandler().getNumberOfDoFs());
      QPInput spatialMotionQPInput = new QPInput(toolbox.getJointIndexHandler().getNumberOfDoFs());

      SpatialAccelerationCommand pointControllerOutput = pointFeedbackController.getInverseDynamicsOutput();
      SpatialAccelerationCommand spatialControllerOutput = spatialFeedbackController.getInverseDynamicsOutput();

      for (int i = 0; i < 300; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         joints.get(0).getPredecessor().updateFramesRecursively();
         centerOfMassFrame.update();

         double proportionalGain = RandomNumbers.nextDouble(random, 10.0, 200.0);
         double derivativeGain = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double integralGain = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double maxIntegralError = RandomNumbers.nextDouble(random, 0.0, 10.0);
         positionGains.setGains(proportionalGain, derivativeGain, integralGain, maxIntegralError);
         positionGains.setMaxProportionalError(RandomNumbers.nextDouble(random, 0.0, 10.0));
         positionGains.setMaxDerivativeError(RandomNumbers.nextDouble(random, 0.0, 10.0));
         positionGains.setMaxFeedbackAndFeedbackRate(RandomNumbers.nextDouble(random, 0.1, 10.0), RandomNumbers.nextDouble(random, 0.1, 10.0));
         pointFeedbackControlCommand.setGains(positionGains);
         spatialFeedbackControlCommand.setPositionGains(positionGains);

         FramePoint3D bodyFixedPointToControl = EuclidFrameRandomTools.nextFramePoint3D(random, endEffector.getBodyFixedFrame(), 1.0, 1.0, 1.0);
         FramePoint3D desiredPosition = new FramePoint3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D feedForwardLinearAcceleration = new FrameVector3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));

         pointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
         spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedPointToControl);

         pointFeedbackControlCommand.setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         spatialFeedbackControlCommand.setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);

         spatialFeedbackController.submitFeedbackControlCommand(spatialFeedbackControlCommand);
         pointFeedbackController.submitFeedbackControlCommand(pointFeedbackControlCommand);

         spatialFeedbackController.computeInverseDynamics();
         pointFeedbackController.computeInverseDynamics();

         motionQPInputCalculator.convertSpatialAccelerationCommand(pointControllerOutput, pointMotionQPInput);
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialControllerOutput, spatialMotionQPInput);

         DenseMatrix64F pointDesiredAcceleration = new DenseMatrix64F(3, 1);
         DenseMatrix64F spatialDesiredAcceleration = new DenseMatrix64F(3, 1);
         pointControllerOutput.getDesiredSpatialAcceleration(pointDesiredAcceleration);
         spatialControllerOutput.getDesiredSpatialAcceleration(spatialDesiredAcceleration);

         assertEquals(spatialDesiredAcceleration, pointDesiredAcceleration, 1.0e-12);

         assertEquals(spatialMotionQPInput.taskJacobian, pointMotionQPInput.taskJacobian, 1.0e-12);
         assertEquals(spatialMotionQPInput.taskObjective, pointMotionQPInput.taskObjective, 1.0e-12);
         assertEquals(spatialMotionQPInput.taskWeightMatrix, pointMotionQPInput.taskWeightMatrix, 1.0e-12);
      }
   }

   private static void assertEquals(DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      assertTrue(assertErrorMessage(expected, actual), MatrixFeatures.isEquals(expected, actual, epsilon));
   }

   private static String assertErrorMessage(DenseMatrix64F expected, DenseMatrix64F actual)
   {
      DenseMatrix64F diff = new DenseMatrix64F(expected.getNumRows(), expected.getNumCols());
      CommonOps.subtract(expected, actual, diff);
      return "Expected:\n" + expected + "\nActual:\n" + actual + ", difference: " + NormOps.normP2(diff);
   }
}
