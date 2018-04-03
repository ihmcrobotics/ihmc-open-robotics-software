package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculatorTest;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class MotionQPInputCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 500;

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConvertSpatialAccelerationCommand() throws Exception
   {
      Random random = new Random(34L);

      int numberOfJoints = 20;

      List<RevoluteJoint> joints = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);
      RigidBody rootBody = joints.get(0).getPredecessor();
      ReferenceFrame rootFrame = rootBody.getBodyFixedFrame();
      RigidBody endEffector = joints.get(numberOfJoints - 1).getSuccessor();
      ReferenceFrame endEffectorFrame = endEffector.getBodyFixedFrame();
      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(joints);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("comFrame", worldFrame, rootBody);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, 0.0, true);
      JointIndexHandler jointIndexHandler = new JointIndexHandler(joints);
      YoVariableRegistry registry = new YoVariableRegistry("dummyRegistry");
      CentroidalMomentumHandler centroidalMomentumHandler = new CentroidalMomentumHandler(rootBody, centerOfMassFrame);
      MotionQPInputCalculator motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame, centroidalMomentumHandler, jointIndexHandler, null, registry);

      MotionQPInput motionQPInput = new MotionQPInput(numberOfDoFs);
      SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
      spatialAccelerationCommand.set(rootBody, endEffector);
      spatialAccelerationCommand.setWeight(random.nextDouble());

      LinearSolver<DenseMatrix64F> pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      DenseMatrix64F desiredJointAccelerations = new DenseMatrix64F(numberOfDoFs, 1);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);
         joints.get(0).updateFramesRecursively();

         centerOfMassFrame.update();

         SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector(endEffectorFrame, rootFrame, endEffectorFrame);
         desiredSpatialAcceleration.setLinearPart(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         desiredSpatialAcceleration.setAngularPart(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         spatialAccelerationCommand.setSpatialAcceleration(endEffectorFrame, desiredSpatialAcceleration);

         motionQPInputCalculator.initialize();
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput);

         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.solve(motionQPInput.taskObjective, desiredJointAccelerations);

         ScrewTools.setDesiredAccelerations(joints, desiredJointAccelerations);

         spatialAccelerationCalculator.compute();
         SpatialAccelerationVector achievedSpatialAcceleration = new SpatialAccelerationVector(endEffectorFrame, rootFrame, endEffectorFrame);
         spatialAccelerationCalculator.getRelativeAcceleration(rootBody, endEffector, achievedSpatialAcceleration);
         SpatialAccelerationCalculatorTest.assertSpatialAccelerationVectorEquals(achievedSpatialAcceleration, desiredSpatialAcceleration, 1.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with the controlFrame not located at the endEffectorFrame
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);
         joints.get(0).updateFramesRecursively();

         centerOfMassFrame.update();

         RigidBodyTransform controlFrameTransform = new RigidBodyTransform();
         controlFrameTransform.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, 10.0));
         ReferenceFrame controlFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("controlFrame" + i, endEffectorFrame, controlFrameTransform);

         SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector(endEffectorFrame, rootFrame, controlFrame);
         desiredSpatialAcceleration.setLinearPart(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         desiredSpatialAcceleration.setAngularPart(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         spatialAccelerationCommand.setSpatialAcceleration(controlFrame, desiredSpatialAcceleration);

         motionQPInputCalculator.initialize();
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput);

         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.solve(motionQPInput.taskObjective, desiredJointAccelerations);

         ScrewTools.setDesiredAccelerations(joints, desiredJointAccelerations);

         spatialAccelerationCalculator.compute();
         SpatialAccelerationVector achievedSpatialAcceleration = new SpatialAccelerationVector(endEffectorFrame, rootFrame, endEffectorFrame);
         spatialAccelerationCalculator.getRelativeAcceleration(rootBody, endEffector, achievedSpatialAcceleration);
         achievedSpatialAcceleration.changeFrameNoRelativeMotion(controlFrame);
         SpatialAccelerationCalculatorTest.assertSpatialAccelerationVectorEquals(achievedSpatialAcceleration, desiredSpatialAcceleration, 1.0e-10);
      }
   }
}
