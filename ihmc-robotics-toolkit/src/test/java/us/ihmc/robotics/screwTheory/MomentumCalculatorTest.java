package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

public class MomentumCalculatorTest
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSingleRigidBodyTranslation()
   {
      Random random = new Random(1766L);

      RigidBodyBasics elevator = new RigidBody("elevator", world);
      Vector3D jointAxis = RandomGeometry.nextVector3D(random);
      jointAxis.normalize();
      PrismaticJoint joint = new PrismaticJoint("joint", elevator, new Vector3D(), jointAxis);
      RigidBodyBasics body = new RigidBody("body", joint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(), new Vector3D());

      joint.setQ(random.nextDouble());
      joint.setQd(random.nextDouble());

      Momentum momentum = computeMomentum(elevator, world);

      momentum.changeFrame(world);
      FrameVector3D linearMomentum = new FrameVector3D(momentum.getReferenceFrame(), new Vector3D(momentum.getLinearPart()));
      FrameVector3D angularMomentum = new FrameVector3D(momentum.getReferenceFrame(), new Vector3D(momentum.getAngularPart()));

      FrameVector3D linearMomentumCheck = new FrameVector3D(joint.getFrameBeforeJoint(), jointAxis);
      linearMomentumCheck.scale(body.getInertia().getMass() * joint.getQd());
      FrameVector3D angularMomentumCheck = new FrameVector3D(world);

      double epsilon = 1e-9;
      EuclidCoreTestTools.assertTuple3DEquals(linearMomentumCheck, linearMomentum, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentumCheck, angularMomentum, epsilon);
      assertTrue(linearMomentum.length() > epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSingleRigidBodyRotation()
   {
      Random random = new Random(1766L);

      RigidBodyBasics elevator = new RigidBody("elevator", world);
      Vector3D jointAxis = RandomGeometry.nextVector3D(random);
      jointAxis.normalize();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setIdentity();
      RevoluteJoint joint = new RevoluteJoint("joint", elevator, transformToParent, jointAxis);
      RigidBodyBasics body = new RigidBody("body", joint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(), new Vector3D());

      joint.setQ(random.nextDouble());
      joint.setQd(random.nextDouble());

      Momentum momentum = computeMomentum(elevator, world);

      momentum.changeFrame(world);
      FrameVector3D linearMomentum = new FrameVector3D(momentum.getReferenceFrame(), new Vector3D(momentum.getLinearPart()));
      FrameVector3D angularMomentum = new FrameVector3D(momentum.getReferenceFrame(), new Vector3D(momentum.getAngularPart()));

      FrameVector3D linearMomentumCheck = new FrameVector3D(world);
      Matrix3D inertia = new Matrix3D(body.getInertia().getMomentOfInertia());
      Vector3D angularMomentumCheckVector = new Vector3D(jointAxis);
      angularMomentumCheckVector.scale(joint.getQd());
      inertia.transform(angularMomentumCheckVector);
      FrameVector3D angularMomentumCheck = new FrameVector3D(body.getInertia().getReferenceFrame(), angularMomentumCheckVector);
      angularMomentumCheck.changeFrame(world);

      double epsilon = 1e-9;
      EuclidCoreTestTools.assertTuple3DEquals(linearMomentumCheck, linearMomentum, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentumCheck, angularMomentum, epsilon);
      assertTrue(angularMomentum.length() > epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChainAgainstCentroidalMomentumMatrix()
   {
      Random random = new Random(17679L);

      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", world);
      int nJoints = 10;
      Vector3D[] jointAxes = new Vector3D[nJoints];
      for (int i = 0; i < nJoints; i++)
      {
         Vector3D jointAxis = RandomGeometry.nextVector3D(random);
         jointAxis.normalize();
         jointAxes[i] = jointAxis;
      }

      joints.addAll(MultiBodySystemRandomTools.nextRevoluteJointChain(random, "randomChain", elevator, jointAxes));
      JointBasics[] jointsArray = new RevoluteJoint[joints.size()];
      joints.toArray(jointsArray);
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("comFrame", world, elevator);

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(random.nextDouble());
         joint.setQd(random.nextDouble());
      }

      Momentum momentum = computeMomentum(elevator, centerOfMassFrame);
      DenseMatrix64F momentumMatrix = new DenseMatrix64F(Momentum.SIZE, 1);
      momentum.get(momentumMatrix);

      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);
      centroidalMomentumMatrix.reset();
      DenseMatrix64F centroidalMomentumMatrixMatrix = centroidalMomentumMatrix.getCentroidalMomentumMatrix();
      DenseMatrix64F jointVelocitiesMatrix = new DenseMatrix64F(MultiBodySystemTools.computeDegreesOfFreedom(jointsArray), 1);
      MultiBodySystemTools.extractJointsState(jointsArray, JointStateType.VELOCITY, jointVelocitiesMatrix);
      DenseMatrix64F momentumFromCentroidalMomentumMatrix = new DenseMatrix64F(Momentum.SIZE, 1);
      CommonOps.mult(centroidalMomentumMatrixMatrix, jointVelocitiesMatrix, momentumFromCentroidalMomentumMatrix);

      double epsilon = 1e-9;
      assertEquals(momentum.getReferenceFrame(), centerOfMassFrame);
      JUnitTools.assertMatrixEquals(momentumFromCentroidalMomentumMatrix, momentumMatrix, epsilon);
      double norm = NormOps.normP2(momentumMatrix);
      assertTrue(norm > epsilon);
   }

   private Momentum computeMomentum(RigidBodyBasics elevator, ReferenceFrame frame)
   {
      elevator.updateFramesRecursively();
      MomentumCalculator momentumCalculator = new MomentumCalculator(elevator);
      Momentum momentum = new Momentum(frame);
      momentumCalculator.computeAndPack(momentum);

      return momentum;
   }
}
