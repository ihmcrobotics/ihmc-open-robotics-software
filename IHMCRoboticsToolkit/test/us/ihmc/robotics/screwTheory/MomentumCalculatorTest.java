package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.testing.JUnitTools;

public class MomentumCalculatorTest
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSingleRigidBodyTranslation()
   {
      Random random = new Random(1766L);

      RigidBody elevator = new RigidBody("elevator", world);
      Vector3D jointAxis = RandomGeometry.nextVector3D(random);
      jointAxis.normalize();
      PrismaticJoint joint = ScrewTools.addPrismaticJoint("joint", elevator, new Vector3D(), jointAxis);
      RigidBody body = ScrewTools.addRigidBody("body", joint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(), new Vector3D());

      joint.setQ(random.nextDouble());
      joint.setQd(random.nextDouble());

      Momentum momentum = computeMomentum(elevator, world);

      momentum.changeFrame(world);
      FrameVector linearMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getLinearPartCopy());
      FrameVector angularMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getAngularPartCopy());

      FrameVector linearMomentumCheck = new FrameVector(joint.getFrameBeforeJoint(), jointAxis);
      linearMomentumCheck.scale(body.getInertia().getMass() * joint.getQd());
      FrameVector angularMomentumCheck = new FrameVector(world);

      double epsilon = 1e-9;
      EuclidCoreTestTools.assertTuple3DEquals(linearMomentumCheck.getVector(), linearMomentum.getVector(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentumCheck.getVector(), angularMomentum.getVector(), epsilon);
      assertTrue(linearMomentum.length() > epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSingleRigidBodyRotation()
   {
      Random random = new Random(1766L);

      RigidBody elevator = new RigidBody("elevator", world);
      Vector3D jointAxis = RandomGeometry.nextVector3D(random);
      jointAxis.normalize();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setIdentity();
      RevoluteJoint joint = ScrewTools.addRevoluteJoint("joint", elevator, transformToParent, jointAxis);
      RigidBody body = ScrewTools.addRigidBody("body", joint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(), new Vector3D());

      joint.setQ(random.nextDouble());
      joint.setQd(random.nextDouble());

      Momentum momentum = computeMomentum(elevator, world);

      momentum.changeFrame(world);
      FrameVector linearMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getLinearPartCopy());
      FrameVector angularMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getAngularPartCopy());

      FrameVector linearMomentumCheck = new FrameVector(world);
      Matrix3D inertia = body.getInertia().getMassMomentOfInertiaPartCopy();
      Vector3D angularMomentumCheckVector = new Vector3D(jointAxis);
      angularMomentumCheckVector.scale(joint.getQd());
      inertia.transform(angularMomentumCheckVector);
      FrameVector angularMomentumCheck = new FrameVector(body.getInertia().getExpressedInFrame(), angularMomentumCheckVector);
      angularMomentumCheck.changeFrame(world);

      double epsilon = 1e-9;
      EuclidCoreTestTools.assertTuple3DEquals(linearMomentumCheck.getVector(), linearMomentum.getVector(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentumCheck.getVector(), angularMomentum.getVector(), epsilon);
      assertTrue(angularMomentum.length() > epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChainAgainstCentroidalMomentumMatrix()
   {
      Random random = new Random(17679L);

      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      RigidBody elevator = new RigidBody("elevator", world);
      int nJoints = 10;
      Vector3D[] jointAxes = new Vector3D[nJoints];
      for (int i = 0; i < nJoints; i++)
      {
         Vector3D jointAxis = RandomGeometry.nextVector3D(random);
         jointAxis.normalize();
         jointAxes[i] = jointAxis;
      }

      ScrewTestTools.createRandomChainRobot("randomChain", joints, elevator, jointAxes, random);
      InverseDynamicsJoint[] jointsArray = new RevoluteJoint[joints.size()];
      joints.toArray(jointsArray);
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("comFrame", world, elevator);

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(random.nextDouble());
         joint.setQd(random.nextDouble());
      }

      Momentum momentum = computeMomentum(elevator, centerOfMassFrame);
      DenseMatrix64F momentumMatrix = new DenseMatrix64F(Momentum.SIZE, 1);
      momentum.getMatrix(momentumMatrix);

      CentroidalMomentumMatrix centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      centroidalMomentumMatrix.compute();
      DenseMatrix64F centroidalMomentumMatrixMatrix = centroidalMomentumMatrix.getMatrix();
      DenseMatrix64F jointVelocitiesMatrix = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointsArray), 1);
      ScrewTools.getJointVelocitiesMatrix(jointsArray, jointVelocitiesMatrix);
      DenseMatrix64F momentumFromCentroidalMomentumMatrix = new DenseMatrix64F(Momentum.SIZE, 1);
      CommonOps.mult(centroidalMomentumMatrixMatrix, jointVelocitiesMatrix, momentumFromCentroidalMomentumMatrix);

      double epsilon = 1e-9;
      assertEquals(momentum.getExpressedInFrame(), centerOfMassFrame);
      JUnitTools.assertMatrixEquals(momentumFromCentroidalMomentumMatrix, momentumMatrix, epsilon);
      double norm = NormOps.normP2(momentumMatrix);
      assertTrue(norm > epsilon);
   }

   private Momentum computeMomentum(RigidBody elevator, ReferenceFrame frame)
   {
      elevator.updateFramesRecursively();
      TwistCalculator twistCalculator = new TwistCalculator(world, elevator);
      twistCalculator.compute();

      MomentumCalculator momentumCalculator = new MomentumCalculator(twistCalculator);
      Momentum momentum = new Momentum(frame);
      momentumCalculator.computeAndPack(momentum);

      return momentum;
   }
}
