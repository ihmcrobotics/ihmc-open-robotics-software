package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.robotics.controllers.PositionPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class PointFeedbackControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testConvergence() throws Exception
   {
      Random random = new Random(5641654L);

      int numberOfJoints = 10;
      Vector3d[] jointAxes = new Vector3d[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);
      
      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      RigidBody elevator = randomFloatingChain.getElevator();
      RigidBody endEffector = joints.get(joints.size() - 1).getSuccessor();
      FramePoint bodyFixedPointToControl = FramePoint.generateRandomFramePoint(random, endEffector.getBodyFixedFrame(), 1.0, 1.0, 1.0);

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      joints.get(0).getPredecessor().updateFramesRecursively();
      FramePoint desiredPosition = new FramePoint();
      desiredPosition.setIncludingFrame(bodyFixedPointToControl);
      desiredPosition.changeFrame(worldFrame);
      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      joints.get(0).getPredecessor().updateFramesRecursively();
      
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, elevator);
      twistCalculator.compute();
      InverseDynamicsJoint[] jointsToOptimizeFor = ScrewTools.computeSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(null, null, controlDT, 0.0, geometricJacobianHolder, twistCalculator, null, null);
      PointFeedbackController pointFeedbackController = new PointFeedbackController(endEffector, toolbox, registry);
      
      PointFeedbackControlCommand pointFeedbackControlCommand = new PointFeedbackControlCommand();
      pointFeedbackControlCommand.set(elevator, endEffector);
      PositionPIDGains gains = new PositionPIDGains();
      gains.setGains(100.0, 50.0);
      pointFeedbackControlCommand.setGains(gains);
      pointFeedbackControlCommand.setBodyFixedPointToControl(bodyFixedPointToControl);
      pointFeedbackControlCommand.set(desiredPosition, new FrameVector(worldFrame), new FrameVector(worldFrame));
      pointFeedbackController.submitFeedbackControlCommand(pointFeedbackControlCommand);
      pointFeedbackController.setEnabled(true);

      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);
      MotionQPInput motionQPInput = new MotionQPInput(numberOfDoFs);
      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      DenseMatrix64F jInverse = new DenseMatrix64F(numberOfDoFs, 6);
      MotionQPInputCalculator motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame, geometricJacobianHolder, twistCalculator, jointsToOptimizeFor, 0.0, registry);
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);
      RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(controlDT);

      FramePoint currentPosition = new FramePoint();
      FrameVector errorVector = new FrameVector();

      double previousErrorMagnitude = Double.POSITIVE_INFINITY;
      double errorMagnitude = previousErrorMagnitude;

      for (int i = 0; i < 100; i++)
      {
         twistCalculator.compute();
         geometricJacobianHolder.compute();

         pointFeedbackController.compute();
         PointAccelerationCommand output = pointFeedbackController.getOutput();

         motionQPInputCalculator.convertPointAccelerationCommand(output, motionQPInput);
         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.invert(jInverse);
         CommonOps.mult(jInverse, motionQPInput.taskObjective, jointAccelerations);

         integrator.integrateJointAccelerations(jointsToOptimizeFor, jointAccelerations);
         integrator.integrateJointVelocities(jointsToOptimizeFor, integrator.getJointVelocities());
         ScrewTools.setVelocities(jointsToOptimizeFor, integrator.getJointVelocities());
         ScrewTools.setJointPositions(jointsToOptimizeFor, integrator.getJointConfigurations());
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
}
