package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
//import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.ImpedancePointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
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
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.matrixlib.NativeMatrix;
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
import us.ihmc.robotics.controllers.pidGains.PD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPD3DStiffnesses;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public final class ImpedancePointFeedbackControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testBaseFrame()
   {
      double controlDT = 0.004;
      double simulationTime = 4.0;
      int integrationSteps = 100;
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
      System.out.println("joints = " + joints);
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, joints, centerOfMassFrame, null, null, registry);
      toolbox.setupForInverseDynamicsSolver(null);
      FeedbackControllerToolbox feedbackControllerToolbox = new FeedbackControllerToolbox(registry);
      ImpedancePointFeedbackController impedancePointFeedbackController = new ImpedancePointFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      // Scramble the joint states.
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      elevator.updateFramesRecursively();

      // Create a command with baseBody as base and desired values in the baseBody frame.
      FramePoint3D desiredPosition = EuclidFrameRandomTools.nextFramePoint3D(random, baseBody.getBodyFixedFrame());
      FrameVector3D zero = new FrameVector3D(desiredPosition.getReferenceFrame());

      PD3DStiffnesses stiffnesses = new DefaultPD3DStiffnesses();
      stiffnesses.setProportionalStiffnesses(5.0);
      stiffnesses.setDerivativeStiffnesses(Double.NaN);
      ImpedancePointFeedbackControlCommand impedancePointFeedbackControlCommand = new ImpedancePointFeedbackControlCommand();
      impedancePointFeedbackControlCommand.set(baseBody, endEffector);
      impedancePointFeedbackControlCommand.setGains(stiffnesses);
      impedancePointFeedbackControlCommand.setInverseDynamics(desiredPosition, zero, zero);
      impedancePointFeedbackController.submitFeedbackControlCommand(impedancePointFeedbackControlCommand);
      impedancePointFeedbackController.setEnabled(true);

      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      NativeQPInputTypeA motionQPInput = new NativeQPInputTypeA(MultiBodySystemTools.computeDegreesOfFreedom(joints));
      DMatrixRMaj jointAccelerations = new DMatrixRMaj(0, 0);
      double damping = 0.001;
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT / integrationSteps);

      for (int i = 0; i < simulationTime / controlDT; i++)
      {
         impedancePointFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand spatialAccelerationCommand = impedancePointFeedbackController.getInverseDynamicsOutput();
         assertTrue(motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput));
         NativeCommonOps.solveDamped(new DMatrixRMaj(motionQPInput.getTaskJacobian()), new DMatrixRMaj(motionQPInput.getTaskObjective()), damping, jointAccelerations);

         // Need to do a fine grain integration since the point we care about will be the center of rotation of the body and we need
         // to maintain that situation.
         for (int j = 0; j < integrationSteps; j++)
         {
            integrator.integrateJointAccelerations(joints, jointAccelerations);
            integrator.integrateJointVelocities(joints, integrator.getJointVelocities());
            MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, integrator.getJointVelocities());
            MultiBodySystemTools.insertJointsState(joints, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
            elevator.updateFramesRecursively();
         }
      }

      // Assert position is close to desired
      FramePoint3D position = new FramePoint3D(endEffector.getBodyFixedFrame());
      position.changeFrame(desiredPosition.getReferenceFrame());
      EuclidCoreTestTools.assertEquals(desiredPosition, position, 1.0E-5);
   }

   @Test
   public void testConvergence() throws Exception
   {
      Random random = new Random(5641654L);

      int numberOfJoints = 10;
      Vector3D[] jointAxes = new Vector3D[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

      YoRegistry registry = new YoRegistry("Dummy");
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
      ImpedancePointFeedbackController ImpedancePointFeedbackController = new ImpedancePointFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      ImpedancePointFeedbackControlCommand impedancePointFeedbackControlCommand = new ImpedancePointFeedbackControlCommand();
      impedancePointFeedbackControlCommand.set(elevator, endEffector);
      PD3DStiffnesses stiffnesses = new DefaultPD3DStiffnesses();
      stiffnesses.setProportionalStiffnesses(100.0);
      stiffnesses.setDerivativeStiffnesses(Double.NaN);
//      stiffnesses.setDerivativeStiffnesses(50.0);
      impedancePointFeedbackControlCommand.setGains(stiffnesses);
      impedancePointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
      impedancePointFeedbackControlCommand.setInverseDynamics(desiredPosition, new FrameVector3D(worldFrame), new FrameVector3D(worldFrame));
      ImpedancePointFeedbackController.submitFeedbackControlCommand(impedancePointFeedbackControlCommand);
      ImpedancePointFeedbackController.setEnabled(true);

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      NativeQPInputTypeA motionQPInput = new NativeQPInputTypeA(numberOfDoFs);
      LinearSolverDense<DMatrixRMaj> pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
      DMatrixRMaj jInverse = new DMatrixRMaj(numberOfDoFs, 6);
      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      DMatrixRMaj jointAccelerations = new DMatrixRMaj(numberOfDoFs, 1);
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      FramePoint3D currentPosition = new FramePoint3D();
      FrameVector3D errorVector = new FrameVector3D();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         ImpedancePointFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand output = ImpedancePointFeedbackController.getInverseDynamicsOutput();

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
         errorVector.sub(desiredPosition, currentPosition);
         errorMagnitude = errorVector.norm();
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
         jointAxes[i] = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

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
      ImpedancePointFeedbackController ImpedancePointFeedbackController = new ImpedancePointFeedbackController(endEffector, toolbox, feedbackControllerToolbox, registry);

      ImpedancePointFeedbackControlCommand impedancePointFeedbackControlCommand = new ImpedancePointFeedbackControlCommand();
      impedancePointFeedbackControlCommand.set(elevator, endEffector);
      PD3DStiffnesses stiffnesses = new DefaultPD3DStiffnesses();
      stiffnesses.setProportionalStiffnesses(20.0);
      stiffnesses.setDerivativeStiffnesses(Double.NaN);
      impedancePointFeedbackControlCommand.setGains(stiffnesses);
      impedancePointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
      impedancePointFeedbackControlCommand.setInverseDynamics(desiredPosition, new FrameVector3D(worldFrame), new FrameVector3D(worldFrame));
      ImpedancePointFeedbackController.submitFeedbackControlCommand(impedancePointFeedbackControlCommand);
      ImpedancePointFeedbackController.setEnabled(true);

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

      DMatrixRMaj tempJtW = new DMatrixRMaj(numberOfDoFs, 3);

      FramePoint3D currentPosition = new FramePoint3D();
      FrameVector3D errorVector = new FrameVector3D();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         ImpedancePointFeedbackController.computeInverseDynamics();
         SpatialAccelerationCommand output = ImpedancePointFeedbackController.getInverseDynamicsOutput();
         motionQPInputCalculator.convertSpatialAccelerationCommand(output, motionQPInput);

         MatrixTools.scaleTranspose(1.0, new DMatrixRMaj(motionQPInput.taskJacobian), tempJtW); // J^T W
         CommonOps_DDRM.mult(tempJtW, new DMatrixRMaj(motionQPInput.taskJacobian), solverInput_H); // H = J^T W J
         CommonOps_DDRM.mult(tempJtW, new DMatrixRMaj(motionQPInput.taskObjective), solverInput_f); // f = - J^T W xDDot
         CommonOps_DDRM.scale(-1.0, solverInput_f);

         for (int diag = 0; diag < numberOfDoFs; diag++)
            solverInput_H.add(diag, diag, 1e-5);

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
         assertArrayEquals(jointAccelerations.data, jointAccelerationsFromQPOASES.data, 2.0e-1);

         currentPosition.setIncludingFrame(bodyFixedPointToControl);
         currentPosition.changeFrame(worldFrame);
         errorVector.sub(desiredPosition, currentPosition);
         errorMagnitude = errorVector.norm();
         boolean isErrorReducing = errorMagnitude < previousErrorMagnitude;
         assertTrue(isErrorReducing);
         previousErrorMagnitude = errorMagnitude;
      }
   }

   @Test
   public void testCompareAgainstSpatialController() throws Exception
   {
      Random random = new Random(5641654L);

      YoRegistry registry = new YoRegistry("Dummy");
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
      ImpedancePointFeedbackController impedancePointFeedbackController = new ImpedancePointFeedbackController(endEffector, toolbox, new FeedbackControllerToolbox(new YoRegistry("Dummy")), registry);
      SpatialFeedbackController spatialFeedbackController = new SpatialFeedbackController(endEffector, toolbox, new FeedbackControllerToolbox(new YoRegistry("Dummy")), registry);
      impedancePointFeedbackController.setEnabled(true);
      spatialFeedbackController.setEnabled(true);

      ImpedancePointFeedbackControlCommand impedancePointFeedbackControlCommand = new ImpedancePointFeedbackControlCommand();
      impedancePointFeedbackControlCommand.set(elevator, endEffector);
      PD3DStiffnesses stiffnesses = new DefaultPD3DStiffnesses();

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
      spatialFeedbackControlCommand.set(elevator, endEffector);
      spatialFeedbackControlCommand.getSpatialAccelerationCommand().setSelectionMatrixForLinearControl();

      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      NativeQPInputTypeA pointMotionQPInput = new NativeQPInputTypeA(toolbox.getJointIndexHandler().getNumberOfDoFs());
      NativeQPInputTypeA spatialMotionQPInput = new NativeQPInputTypeA(toolbox.getJointIndexHandler().getNumberOfDoFs());

      SpatialAccelerationCommand pointControllerOutput = impedancePointFeedbackController.getInverseDynamicsOutput();
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
         PID3DGains positionGains = new DefaultPID3DGains();
         positionGains.setGains(proportionalGain, derivativeGain, integralGain, maxIntegralError);
         positionGains.setMaxProportionalError(RandomNumbers.nextDouble(random, 0.0, 10.0));
         positionGains.setMaxDerivativeError(RandomNumbers.nextDouble(random, 0.0, 10.0));
         positionGains.setMaxFeedbackAndFeedbackRate(RandomNumbers.nextDouble(random, 0.1, 10.0), RandomNumbers.nextDouble(random, 0.1, 10.0));
         stiffnesses.setProportionalStiffnesses(proportionalGain);
         stiffnesses.setDerivativeStiffnesses(derivativeGain);
         stiffnesses.setMaxFeedbackAndFeedbackRate(RandomNumbers.nextDouble(random, 0.1, 10.0), RandomNumbers.nextDouble(random, 0.1, 10.0));


         PD3DStiffnesses positionStiffness = new DefaultPD3DStiffnesses();
         positionStiffness.set(positionGains);
         positionStiffness.setDerivativeStiffnesses(Double.NaN);
         impedancePointFeedbackControlCommand.setGains(positionStiffness);
         spatialFeedbackControlCommand.setPositionGains(positionGains);

         FramePoint3D bodyFixedPointToControl = EuclidFrameRandomTools.nextFramePoint3D(random, endEffector.getBodyFixedFrame(), 1.0, 1.0, 1.0);
         FramePoint3D desiredPosition = new FramePoint3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D feedForwardLinearAcceleration = new FrameVector3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));

         impedancePointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
         spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedPointToControl);

         impedancePointFeedbackControlCommand.setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         spatialFeedbackControlCommand.setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);

         spatialFeedbackController.submitFeedbackControlCommand(spatialFeedbackControlCommand);
         impedancePointFeedbackController.submitFeedbackControlCommand(impedancePointFeedbackControlCommand);

         spatialFeedbackController.computeInverseDynamics();
         impedancePointFeedbackController.computeInverseDynamics();

         motionQPInputCalculator.convertSpatialAccelerationCommand(pointControllerOutput, pointMotionQPInput);
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialControllerOutput, spatialMotionQPInput);

         DMatrixRMaj pointDesiredAcceleration = new DMatrixRMaj(3, 1);
         DMatrixRMaj spatialDesiredAcceleration = new DMatrixRMaj(3, 1);
         pointControllerOutput.getDesiredSpatialAcceleration(pointDesiredAcceleration);
         spatialControllerOutput.getDesiredSpatialAcceleration(spatialDesiredAcceleration);

         assertEquals(spatialDesiredAcceleration, pointDesiredAcceleration, 1.0e-12);

         assertEquals(spatialMotionQPInput.taskJacobian, pointMotionQPInput.taskJacobian, 1.0e-12);
         assertEquals(spatialMotionQPInput.taskObjective, pointMotionQPInput.taskObjective, 1.0e-12);
         assertEquals(spatialMotionQPInput.taskWeightMatrix, pointMotionQPInput.taskWeightMatrix, 1.0e-12);
      }
   }

   private static void assertEquals(NativeMatrix expected, NativeMatrix actual, double epsilon)
   {
      assertEquals(new DMatrixRMaj(expected), new DMatrixRMaj(actual), epsilon);
   }

   private static void assertEquals(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      EjmlUnitTests.assertEquals(actual, expected, epsilon);
   }
}
