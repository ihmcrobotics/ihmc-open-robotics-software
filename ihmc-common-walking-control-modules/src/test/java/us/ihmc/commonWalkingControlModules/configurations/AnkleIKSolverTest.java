package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class AnkleIKSolverTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testPitchRollAnkleWithSolePlaneConstraintSolver()
   {
      Random random = new Random(64292L);

      AnkleIKSolver ankleIKSolver = new AnkleIKSolver.PitchRollAnkleWithSolePlaneConstraintSolver();

      Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
      Matrix3D inertia = new Matrix3D(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      DMatrixRMaj achievedJointAngles = new DMatrixRMaj(1, 1);
      DMatrixRMaj achievedJointVelocities = new DMatrixRMaj(1, 1);
      DMatrixRMaj jacobianAngularPart = new DMatrixRMaj(3, 2);
      DMatrixRMaj angularVelocity = new DMatrixRMaj(3, 1);
      DMatrixRMaj expectedJointVelocities = new DMatrixRMaj(2, 1);
      DampedLeastSquaresSolver solver = new DampedLeastSquaresSolver(2);

      for (int i = 0; i < 1000; i++)
      {
         // Create a random ankle robot with one pitch and one roll joint:
         RigidBodyBasics shin = new RigidBody("Shin", ReferenceFrame.getWorldFrame());
         Vector3D offset1 = EuclidCoreRandomTools.nextVector3D(random);
         OneDoFJointBasics anklePitch = new RevoluteJoint("anklePitch", shin, offset1, new Vector3D(0.0, 1.0, 0.0));
         RigidBodyBasics ankle = new RigidBody("Ankle", anklePitch, inertia, 1.0, new RigidBodyTransform());
         Vector3D offset2 = EuclidCoreRandomTools.nextVector3D(random);
         OneDoFJointBasics ankleRoll = new RevoluteJoint("ankleRoll", ankle, offset2, new Vector3D(1.0, 0.0, 0.0));
         RigidBodyBasics foot = new RigidBody("Foot", ankleRoll, inertia, 1.0, new RigidBodyTransform());

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
         CommonOps_DDRM.extract(jacobian.getJacobianMatrix(), 0, 3, 0, 2, jacobianAngularPart, 0, 0);
         solver.setA(jacobianAngularPart);
         desiredAngularVelocity.get(angularVelocity);
         solver.solve(angularVelocity, expectedJointVelocities);
         Assert.assertEquals(expectedJointVelocities.get(0), achievedJointVelocities.get(0), 1.0e-10);
         Assert.assertEquals(expectedJointVelocities.get(1), achievedJointVelocities.get(1), 1.0e-10);
      }
   }
}
