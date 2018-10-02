package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class AnkleIKSolverTest
{
   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPitchRollAnkleWithSolePlaneConstraintSolver()
   {
      Random random = new Random(64292L);

      AnkleIKSolver ankleIKSolver = new AnkleIKSolver.PitchRollAnkleWithSolePlaneConstraintSolver();

      Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
      Matrix3D inertia = new Matrix3D(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      DenseMatrix64F achievedJointAngles = new DenseMatrix64F(1, 1);
      DenseMatrix64F achievedJointVelocities = new DenseMatrix64F(1, 1);
      DenseMatrix64F jacobianAngularPart = new DenseMatrix64F(3, 2);
      DenseMatrix64F angularVelocity = new DenseMatrix64F(3, 1);
      DenseMatrix64F expectedJointVelocities = new DenseMatrix64F(2, 1);
      DampedLeastSquaresSolver solver = new DampedLeastSquaresSolver(2);

      for (int i = 0; i < 1000; i++)
      {
         // Create a random ankle robot with one pitch and one roll joint:
         RigidBody shin = new RigidBody("Shin", ReferenceFrame.getWorldFrame());
         Vector3D offset1 = EuclidCoreRandomTools.nextVector3D(random);
         OneDoFJoint anklePitch = ScrewTools.addRevoluteJoint("anklePitch", shin, offset1, new Vector3D(0.0, 1.0, 0.0));
         RigidBody ankle = ScrewTools.addRigidBody("Ankle", anklePitch, inertia, 1.0, new RigidBodyTransform());
         Vector3D offset2 = EuclidCoreRandomTools.nextVector3D(random);
         OneDoFJoint ankleRoll = ScrewTools.addRevoluteJoint("ankleRoll", ankle, offset2, new Vector3D(1.0, 0.0, 0.0));
         RigidBody foot = ScrewTools.addRigidBody("Foot", ankleRoll, inertia, 1.0, new RigidBodyTransform());

         // Solve for some random desired values:
         Quaternion desiredOrientation = EuclidCoreRandomTools.nextQuaternion(random, Math.PI / 4.0);
         Vector3D desiredAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);
         ankleIKSolver.computeAngles(desiredOrientation, achievedJointAngles);
         ankleIKSolver.computeVelocities(desiredAngularVelocity, achievedJointAngles, achievedJointVelocities);

         // Update the robot for validation:
         anklePitch.setQ(achievedJointAngles.get(0));
         ankleRoll.setQ(achievedJointAngles.get(1));
         anklePitch.updateFramesRecursively();
         ReferenceFrame achievedFootFrame = ankleRoll.getFrameAfterJoint();
         PoseReferenceFrame desiredFootFrame = new PoseReferenceFrame("DesiredFrame", ReferenceFrame.getWorldFrame());
         desiredFootFrame.setOrientationAndUpdate(desiredOrientation);

         // The z axis in the achieved foot frame should match the z axis in the desired foot frame.
         FrameVector3D zAxisInFoot = new FrameVector3D(achievedFootFrame, zAxis);
         zAxisInFoot.changeFrame(desiredFootFrame);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(zAxis, zAxisInFoot, 1.0e-10);

         // For the velocities use a Jacobian to validate.
         GeometricJacobian jacobian = new GeometricJacobian(shin, foot, ReferenceFrame.getWorldFrame());
         jacobian.compute();
         CommonOps.extract(jacobian.getJacobianMatrix(), 0, 3, 0, 2, jacobianAngularPart, 0, 0);
         solver.setA(jacobianAngularPart);
         desiredAngularVelocity.get(angularVelocity);
         solver.solve(angularVelocity, expectedJointVelocities);
         Assert.assertEquals(expectedJointVelocities.get(0), achievedJointVelocities.get(0), 1.0e-10);
         Assert.assertEquals(expectedJointVelocities.get(1), achievedJointVelocities.get(1), 1.0e-10);
      }
   }
}
