package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GeometricJacobianTest
{
   private final Random random = new Random(101L);

   private GeometricJacobian bodyManipulatorJacobian;
   private GeometricJacobian spatialManipulatorJacobian;

   private PrismaticJoint joint1;
   private RevoluteJoint joint2;
   private PrismaticJoint joint3;



   @Before
   public void setUp() throws Exception
   {
      buildMechanismAndJacobians();

      // joint positions
      joint1.setQ(random.nextDouble());
      joint2.setQ(random.nextDouble());
      joint3.setQ(random.nextDouble());
   }

   @After
   public void tearDown() throws Exception
   {
   }

   /**
    * Tests computation of the Jacobian for a simple 2D manipulator, as shown in
    * Duindam - Port-based modeling and control for efficient bipedal robots, p.34
    * Jacobian was also computed manually using a Matlab script.
    */

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDuindamExample()
   {
      // compute
      joint1.getPredecessor().updateFramesRecursively();
      bodyManipulatorJacobian.compute();
      spatialManipulatorJacobian.compute();

      // set some joint velocities, and get the twist back
      double q1d = random.nextDouble();
      double q2d = random.nextDouble();
      double q3d = random.nextDouble();

      DenseMatrix64F qd = new DenseMatrix64F(3, 1);
      qd.set(0, 0, q1d);
      qd.set(1, 0, q2d);
      qd.set(2, 0, q3d);

      DenseMatrix64F twistMatrixFromBodyManipulatorJacobian = new DenseMatrix64F(Twist.SIZE, 1);
      CommonOps.mult(bodyManipulatorJacobian.getJacobianMatrix(), qd, twistMatrixFromBodyManipulatorJacobian);
      Twist twistFromBodyManipulatorJacobian = new Twist(bodyManipulatorJacobian.getEndEffectorFrame(), bodyManipulatorJacobian.getBaseFrame(),
                                                  bodyManipulatorJacobian.getJacobianFrame(), twistMatrixFromBodyManipulatorJacobian);
      Vector3d omegaFromBodyManipulatorJacobian = twistFromBodyManipulatorJacobian.getAngularPartCopy();
      Vector3d vFromBodyManipulatorJacobian = twistFromBodyManipulatorJacobian.getLinearPartCopy();


      DenseMatrix64F twistMatrixFromSpatialManipulatorJacobian = new DenseMatrix64F(Twist.SIZE, 1);
      CommonOps.mult(spatialManipulatorJacobian.getJacobianMatrix(), qd, twistMatrixFromSpatialManipulatorJacobian);

      Twist twistFromSpatialManipulatorJacobian = new Twist(spatialManipulatorJacobian.getEndEffectorFrame(), spatialManipulatorJacobian.getBaseFrame(),
                                                     spatialManipulatorJacobian.getJacobianFrame(), twistMatrixFromSpatialManipulatorJacobian);
      Vector3d omegaFromSpatialManipulatorJacobian = twistFromSpatialManipulatorJacobian.getAngularPartCopy();
      Vector3d vFromSpatialManipulatorJacobian = twistFromSpatialManipulatorJacobian.getLinearPartCopy();

      // compare to hand calculations
      double q1 = joint1.getQ();
      double q2 = joint2.getQ();
      double q3 = joint3.getQ();
      double epsilon = 1e-8;
      Vector3d omegaBodyByHand = new Vector3d(-q2d, 0.0, 0.0);
      Vector3d vBodyByHand = new Vector3d(0.0, -Math.cos(q2) * q1d + q3 * q2d, -Math.sin(q2) * q1d + q3d);

      Vector3d omegaBodyError = new Vector3d(omegaFromBodyManipulatorJacobian);
      omegaBodyError.sub(omegaBodyByHand);
      assertEquals(0.0, omegaBodyError.length(), epsilon);

      Vector3d vBodyError = new Vector3d(vFromBodyManipulatorJacobian);
      vBodyError.sub(vBodyByHand);
      assertEquals(0.0, vBodyError.length(), epsilon);

      Vector3d omegaSpatialByHand = new Vector3d(-q2d, 0.0, 0.0);
      Vector3d vSpatialByHand = new Vector3d(0.0, -q1d - 3.0 * q2d + Math.sin(q2) * q3d, -q1 * q2d + Math.cos(q2) * q3d);

      Vector3d omegaSpatialError = new Vector3d(omegaFromSpatialManipulatorJacobian);
      omegaSpatialError.sub(omegaSpatialByHand);
      assertEquals(0.0, omegaSpatialError.length(), epsilon);

      Vector3d vSpatialError = new Vector3d(vFromSpatialManipulatorJacobian);
      vSpatialError.sub(vSpatialByHand);
      assertEquals(0.0, vSpatialError.length(), epsilon);

      // rotate components of twistFromBodyManipulatorJacobian to base frame and compare to hand calculations again
      Vector3d omegaInBaseFrame = new Vector3d();
      twistFromBodyManipulatorJacobian.getAngularVelocityInBaseFrame(omegaInBaseFrame);
      Vector3d vInBaseFrame = new Vector3d();
      twistFromBodyManipulatorJacobian.getBodyOriginLinearPartInBaseFrame(vInBaseFrame);

      Vector3d omegaInBaseFrameByHand = new Vector3d(-q2d, 0.0, 0.0);
      Vector3d vInBaseFrameByHand = new Vector3d(0.0, -q1d + Math.cos(q2) * q3 * q2d + Math.sin(q2) * q3d, -Math.sin(q2) * q3 * q2d + Math.cos(q2) * q3d);

      omegaBodyError = new Vector3d(omegaInBaseFrame);
      omegaBodyError.sub(omegaInBaseFrameByHand);
      assertEquals(0.0, omegaBodyError.length(), epsilon);

      vBodyError = new Vector3d(vInBaseFrame);
      vBodyError.sub(vInBaseFrameByHand);
      assertEquals(0.0, vBodyError.length(), epsilon);
   }

   private void buildMechanismAndJacobians()
   {
      RigidBody base = new RigidBody("base", ReferenceFrame.getWorldFrame());
      joint1 = ScrewTools.addPrismaticJoint("joint1", base, new Vector3d(), new Vector3d(0.0, -1.0, 0.0));
      RigidBody body1 = ScrewTools.addRigidBody("body1", joint1, new Matrix3d(), 0.0, new Vector3d());
      joint2 = ScrewTools.addRevoluteJoint("joint2", body1, new Vector3d(0.0, 0.0, 3.0), new Vector3d(-1.0, 0.0, 0.0));
      RigidBody body2 = ScrewTools.addRigidBody("body2", joint2, new Matrix3d(), 0.0, new Vector3d());
      joint3 = ScrewTools.addPrismaticJoint("joint4", body2, new Vector3d(), new Vector3d(0.0, 0.0, 1.0));
      RigidBody body3 = ScrewTools.addRigidBody("body3", joint3, new Matrix3d(), 0.0, new Vector3d());

      // create Jacobians
      bodyManipulatorJacobian = new GeometricJacobian(base, body3, joint3.getFrameAfterJoint());
      spatialManipulatorJacobian = new GeometricJacobian(base, body3, joint1.getFrameBeforeJoint());
   }
}
