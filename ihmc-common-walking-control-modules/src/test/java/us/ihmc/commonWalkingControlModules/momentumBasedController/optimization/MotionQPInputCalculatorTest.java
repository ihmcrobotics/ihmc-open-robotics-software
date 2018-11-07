package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.After;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculatorTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MotionQPInputCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 500;

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConvertSpatialAccelerationCommand() throws Exception
   {
      Random random = new Random(34L);

      int numberOfJoints = 20;

      List<RevoluteJoint> joints = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);
      RigidBodyBasics rootBody = joints.get(0).getPredecessor();
      ReferenceFrame rootFrame = rootBody.getBodyFixedFrame();
      RigidBodyBasics endEffector = joints.get(numberOfJoints - 1).getSuccessor();
      ReferenceFrame endEffectorFrame = endEffector.getBodyFixedFrame();
      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(joints);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("comFrame", worldFrame, rootBody);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, 0.0);
      JointIndexHandler jointIndexHandler = new JointIndexHandler(joints);
      YoVariableRegistry registry = new YoVariableRegistry("dummyRegistry");
      CentroidalMomentumRateCalculator centroidalMomentumHandler = new CentroidalMomentumRateCalculator(rootBody, centerOfMassFrame);
      MotionQPInputCalculator motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame, centroidalMomentumHandler, jointIndexHandler, null, registry);

      QPInput motionQPInput = new QPInput(numberOfDoFs);
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

         SpatialAcceleration desiredSpatialAcceleration = new SpatialAcceleration(endEffectorFrame, rootFrame, endEffectorFrame);
         desiredSpatialAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         desiredSpatialAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         spatialAccelerationCommand.setSpatialAcceleration(endEffectorFrame, desiredSpatialAcceleration);

         motionQPInputCalculator.initialize();
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput);

         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.solve(motionQPInput.taskObjective, desiredJointAccelerations);

         ScrewTools.setJointAccelerations(joints, desiredJointAccelerations);

         spatialAccelerationCalculator.compute();
         SpatialAcceleration achievedSpatialAcceleration = new SpatialAcceleration(endEffectorFrame, rootFrame, endEffectorFrame);
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

         SpatialAcceleration desiredSpatialAcceleration = new SpatialAcceleration(endEffectorFrame, rootFrame, controlFrame);
         desiredSpatialAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         desiredSpatialAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         spatialAccelerationCommand.setSpatialAcceleration(controlFrame, desiredSpatialAcceleration);

         motionQPInputCalculator.initialize();
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialAccelerationCommand, motionQPInput);

         pseudoInverseSolver.setA(motionQPInput.taskJacobian);
         pseudoInverseSolver.solve(motionQPInput.taskObjective, desiredJointAccelerations);

         ScrewTools.setJointAccelerations(joints, desiredJointAccelerations);

         spatialAccelerationCalculator.compute();
         SpatialAcceleration achievedSpatialAcceleration = new SpatialAcceleration(endEffectorFrame, rootFrame, endEffectorFrame);
         spatialAccelerationCalculator.getRelativeAcceleration(rootBody, endEffector, achievedSpatialAcceleration);
         achievedSpatialAcceleration.changeFrame(controlFrame);
         SpatialAccelerationCalculatorTest.assertSpatialAccelerationVectorEquals(achievedSpatialAcceleration, desiredSpatialAcceleration, 1.0e-10);
      }
   }
}
