package us.ihmc.robotics.screwTheory;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class MomentumTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCompute()
   {
      Random random = new Random(1776L);
      ReferenceFrame world = ReferenceFrame.getWorldFrame();
      RigidBodyTransform transformToParent = RigidBodyTransform.generateRandomTransform(random);

      // Transform3D transformToParent = new Transform3D();

      ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frame", world, transformToParent);
      Matrix3d massMomentOfInertia = RandomTools.generateRandomDiagonalMatrix3d(random);
      double mass = random.nextDouble();
      GeneralizedRigidBodyInertia inertia = new CompositeRigidBodyInertia(frame, massMomentOfInertia, mass);
      inertia.changeFrame(world);

      Vector3d linearVelocity = RandomTools.generateRandomVector(random);
      Vector3d angularVelocity = RandomTools.generateRandomVector(random);
      Twist twist = new Twist(frame, world, world, linearVelocity, angularVelocity);

      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(6, 6);
      inertia.getMatrix(inertiaMatrix);
      DenseMatrix64F twistMatrix = twist.toMatrix();
      DenseMatrix64F expensive = new DenseMatrix64F(inertiaMatrix.getNumRows(), twistMatrix.getNumCols());
      CommonOps.mult(inertiaMatrix, twistMatrix, expensive);

      Momentum momentum = new Momentum();
      momentum.compute(inertia, twist);
      DenseMatrix64F cheap = momentum.toDenseMatrix();

      JUnitTools.assertMatrixEquals(expensive, cheap, 1e-12);
   }
}
