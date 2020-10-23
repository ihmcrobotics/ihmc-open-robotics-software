package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;
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

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testVersusNumericalDifferentiation()
   {
      Random random = new Random(1252523L);
      Vector3D[] jointAxes = new Vector3D[]
      {
         X, Y, Z, Y, Y, X
      };
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      randomFloatingChain.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY);

      RigidBodyBasics base = randomFloatingChain.getRootJoint().getSuccessor();
      RigidBodyBasics endEffector = randomFloatingChain.getLeafBody();
      GeometricJacobian geometricJacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      geometricJacobian.compute();

      FramePoint3D point = new FramePoint3D(base.getBodyFixedFrame(), RandomGeometry.nextVector3D(random));
      PointJacobian pointJacobian = new PointJacobian();
      pointJacobian.set(geometricJacobian, point);
      pointJacobian.compute();

      JointBasics[] joints = geometricJacobian.getJointsInOrder();

      DMatrixRMaj jointVelocities = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);

      DMatrixRMaj pointVelocityFromJacobianMatrix = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(pointJacobian.getJacobianMatrix(), jointVelocities, pointVelocityFromJacobianMatrix);
      FrameVector3D pointVelocityFromJacobian = new FrameVector3D(pointJacobian.getFrame());
      pointVelocityFromJacobian.set(pointVelocityFromJacobianMatrix);

      FramePoint3D point2 = new FramePoint3D(point);
      point2.changeFrame(endEffector.getBodyFixedFrame());
      double dt = 1e-8;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);
      integrator.integrateFromVelocity(randomFloatingChain.getRevoluteJoints());
      randomFloatingChain.getElevator().updateFramesRecursively();
      point2.changeFrame(base.getBodyFixedFrame());

      FrameVector3D pointVelocityFromNumericalDifferentiation = new FrameVector3D(point2);
      pointVelocityFromNumericalDifferentiation.sub(point);
      pointVelocityFromNumericalDifferentiation.scale(1.0 / dt);

      EuclidFrameTestTools.assertFrameTuple3DEquals(pointVelocityFromNumericalDifferentiation, pointVelocityFromJacobian, 1e-6);
   }

	@Test
   public void testSingularValuesOfTwoPointJacobians()
   {
      Random random = new Random(12351235L);
      Vector3D[] jointAxes = new Vector3D[]
      {
         X, Y, Z, Y, Y, X
      };
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      randomFloatingChain.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY);

      RigidBodyBasics base = randomFloatingChain.getRootJoint().getSuccessor();
      RigidBodyBasics endEffector = randomFloatingChain.getLeafBody();
      GeometricJacobian geometricJacobian = new GeometricJacobian(base, endEffector, base.getBodyFixedFrame());
      geometricJacobian.compute();

      FramePoint3D point1 = new FramePoint3D(base.getBodyFixedFrame(), RandomGeometry.nextVector3D(random));
      FramePoint3D point2 = new FramePoint3D(base.getBodyFixedFrame(), RandomGeometry.nextVector3D(random));

      PointJacobian pointJacobian1 = new PointJacobian();
      pointJacobian1.set(geometricJacobian, point1);
      pointJacobian1.compute();

      PointJacobian pointJacobian2 = new PointJacobian();
      pointJacobian2.set(geometricJacobian, point2);
      pointJacobian2.compute();

      DMatrixRMaj assembledJacobian = new DMatrixRMaj(SpatialVector.SIZE, geometricJacobian.getNumberOfColumns());
      CommonOps_DDRM.insert(pointJacobian1.getJacobianMatrix(), assembledJacobian, 0, 0);
      CommonOps_DDRM.insert(pointJacobian2.getJacobianMatrix(), assembledJacobian, 3, 0);

      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(assembledJacobian.getNumRows(), assembledJacobian.getNumCols(), true, true,
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
