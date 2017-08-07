package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVectorTest;
import us.ihmc.robotics.random.RandomGeometry;

/**
 * @author twan
 *         Date: 5/18/13
 */
public class PointJacobianTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVersusNumericalDifferentiation()
   {
      Random random = new Random(1252523L);
      Vector3D[] jointAxes = new Vector3D[]
      {
         X, Y, Z, Y, Y, X
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      RigidBody base = randomFloatingChain.getRootJoint().getSuccessor();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      GeometricJacobian geometricJacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      geometricJacobian.compute();

      FramePoint point = new FramePoint(base.getBodyFixedFrame(), RandomGeometry.nextVector3D(random));
      PointJacobian pointJacobian = new PointJacobian();
      pointJacobian.set(geometricJacobian, point);
      pointJacobian.compute();

      InverseDynamicsJoint[] joints = geometricJacobian.getJointsInOrder();

      DenseMatrix64F jointVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(joints), 1);
      ScrewTools.getJointVelocitiesMatrix(joints, jointVelocities);

      DenseMatrix64F pointVelocityFromJacobianMatrix = new DenseMatrix64F(3, 1);
      CommonOps.mult(pointJacobian.getJacobianMatrix(), jointVelocities, pointVelocityFromJacobianMatrix);
      FrameVector pointVelocityFromJacobian = new FrameVector(pointJacobian.getFrame());
      pointVelocityFromJacobian.getVector().set(pointVelocityFromJacobianMatrix);

      FramePoint point2 = new FramePoint(point);
      point2.changeFrame(endEffector.getBodyFixedFrame());
      double dt = 1e-8;
      ScrewTestTools.integrateVelocities(randomFloatingChain.getRevoluteJoints(), dt);
      point2.changeFrame(base.getBodyFixedFrame());

      FrameVector pointVelocityFromNumericalDifferentiation = new FrameVector(point2);
      pointVelocityFromNumericalDifferentiation.sub(point);
      pointVelocityFromNumericalDifferentiation.scale(1.0 / dt);

      FrameVectorTest.assertFrameVectorEquals(pointVelocityFromNumericalDifferentiation, pointVelocityFromJacobian, 1e-6);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSingularValuesOfTwoPointJacobians()
   {
      Random random = new Random(12351235L);
      Vector3D[] jointAxes = new Vector3D[]
      {
         X, Y, Z, Y, Y, X
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);

      RigidBody base = randomFloatingChain.getRootJoint().getSuccessor();
      RigidBody endEffector = randomFloatingChain.getLeafBody();
      GeometricJacobian geometricJacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      geometricJacobian.compute();

      FramePoint point1 = new FramePoint(base.getBodyFixedFrame(), RandomGeometry.nextVector3D(random));
      FramePoint point2 = new FramePoint(base.getBodyFixedFrame(), RandomGeometry.nextVector3D(random));

      PointJacobian pointJacobian1 = new PointJacobian();
      pointJacobian1.set(geometricJacobian, point1);
      pointJacobian1.compute();

      PointJacobian pointJacobian2 = new PointJacobian();
      pointJacobian2.set(geometricJacobian, point2);
      pointJacobian2.compute();

      DenseMatrix64F assembledJacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, geometricJacobian.getNumberOfColumns());
      CommonOps.insert(pointJacobian1.getJacobianMatrix(), assembledJacobian, 0, 0);
      CommonOps.insert(pointJacobian2.getJacobianMatrix(), assembledJacobian, 3, 0);

      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(assembledJacobian.getNumRows(), assembledJacobian.getNumCols(), true, true,
                                                          false);
      svd.decompose(assembledJacobian);

      double[] singularValues = svd.getSingularValues();
      double smallestSingularValue = Double.POSITIVE_INFINITY;
      for (double singularValue : singularValues)
      {
         if (singularValue < smallestSingularValue)
         {
            smallestSingularValue = singularValue;
         }
      }

      double epsilon = 1e-12;
      assertTrue(smallestSingularValue < epsilon);
   }
}
