package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.Random;

import static org.junit.Assert.assertEquals;

public class TransformToolsTest
{

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testRotate()
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      TransformTools.rotate(transform, Math.PI / 4, Axis.X);

      RigidBodyTransform transform2 = new RigidBodyTransform();

      transform2.rotX(Math.PI / 4);

      RigidBodyTransformTest.assertTransformEquals(transform, transform2, 1e-7);

      transform = new RigidBodyTransform();

      TransformTools.rotate(transform, 3 * Math.PI / 4, Axis.Y);

      transform2 = new RigidBodyTransform();

      transform2.rotY(3 * Math.PI / 4);

      RigidBodyTransformTest.assertTransformEquals(transform, transform2, 1e-7);

      transform = new RigidBodyTransform();

      TransformTools.rotate(transform, -Math.PI / 2, Axis.Z);

      transform2 = new RigidBodyTransform();

      transform2.rotZ(-Math.PI / 2);

      RigidBodyTransformTest.assertTransformEquals(transform, transform2, 1e-7);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testDifferentiate()
   {
      Random random = new Random();
      Transform3d transform1 = new Transform3d();
      Transform3d transform2 = new Transform3d();
      DenseMatrix64F matrix1 = new DenseMatrix64F(4, 4);
      DenseMatrix64F matrix2 = new DenseMatrix64F(4, 4);
      double dt = 0.1;

      createRandomTransformationMatrix(matrix1, random);
      createRandomTransformationMatrix(matrix2, random);
      transform1.set(matrix1);
      transform2.set(matrix2);

      CommonOps.subtract(matrix2, matrix1, matrix2);
      CommonOps.scale(1 / dt, matrix2);
      matrix2.set(3, 3, 1);

      DenseMatrix64F testMatrix = TransformTools.differentiate(transform1, transform2, dt);

      JUnitTools.assertMatrixEquals("", testMatrix, matrix2, 1e-8);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetTransformFromA1toA2Simple()
   {
      Vector3d vectorA1 = new Vector3d(-1.0, -2.0, -3.0);
      RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform();
      transformFromWorldToA1.setTranslation(vectorA1);


      Vector3d vectorA2 = new Vector3d(vectorA1);
      vectorA2.negate();
      RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform();
      transformFromWorldToA2.setTranslation(vectorA2);

      ReferenceFrame a1 = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("a1", ReferenceFrame.getWorldFrame(), transformFromWorldToA1);
      ReferenceFrame a2 = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("a2", ReferenceFrame.getWorldFrame(), transformFromWorldToA2);

      RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);

      Point3d a2Origin = new Point3d();
      transformA2toA1.transform(a2Origin);

      // System.out.println("a2Origin after transform" + a2Origin);

      FramePoint a2OriginFramePoint = new FramePoint(a2);

      a2OriginFramePoint.changeFrame(a1);

      // System.out.println("a2OriginFramePoint = " + a2OriginFramePoint);

      a2Origin.epsilonEquals(a2OriginFramePoint.getPoint(), 1e-9);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetTransformFromA1toA2Random()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
      Random random = new Random(111L);

      this.createRandomTransformationMatrix(matrix, random);

      RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform(matrix);

      this.createRandomTransformationMatrix(matrix, random);
      RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform(matrix);

      ReferenceFrame a1 = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("a1", ReferenceFrame.getWorldFrame(), transformFromWorldToA1);
      ReferenceFrame a2 = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("a2", ReferenceFrame.getWorldFrame(), transformFromWorldToA2);

      RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);

      Point3d a2Origin = new Point3d();
      transformA2toA1.transform(a2Origin);

      // System.out.println("a2Origin after transform" + a2Origin);

      FramePoint a2OriginFramePoint = new FramePoint(a2);

      a2OriginFramePoint.changeFrame(a1);

      // System.out.println("a2OriginFramePoint = " + a2OriginFramePoint);

      a2Origin.epsilonEquals(a2OriginFramePoint.getPoint(), 1e-9);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testgetTransformDifference()
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();

      Random random = new Random();

      int numberOfTests = 1000;
      for (int i = 0; i < numberOfTests; i++)
      {
         double angle = -Math.PI + 2 * Math.PI * random.nextDouble();
         Vector3d vector3d = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         vector3d.normalize();

         AxisAngle4d axisAngle = new AxisAngle4d(vector3d, angle);

         double vectorScale = 10.0;
         Vector3d vector3dTranlation = new Vector3d(-vectorScale + 2.0 * vectorScale * random.nextDouble(), -vectorScale + 2.0 * vectorScale * random.nextDouble(), -vectorScale + 2.0 * vectorScale * random.nextDouble());
         rigidBodyTransform.set(axisAngle, vector3dTranlation);
         
         double angleFromTransform = TransformTools.getMagnitudeOfAngleOfRotation(rigidBodyTransform);
         
         assertEquals(Math.abs(angle), angleFromTransform, 1e-9);
         
         double translation = TransformTools.getMagnitudeOfTranslation(rigidBodyTransform);
         assertEquals(vector3dTranlation.length(), translation, 1e-9);
         
         double radiusOfRotation = random.nextDouble();
         double magnitudeOfTransform = TransformTools.getSizeOfTransformWithRotationScaled(rigidBodyTransform, radiusOfRotation);
         assertEquals(vector3dTranlation.length() + Math.abs(angle) * radiusOfRotation, magnitudeOfTransform, 1e-9);
      }
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testgetTransformDifferenceBetweenTwoTransforms()
   {

      Random random = new Random();

      int numberOfTests = 1000;
      for (int i = 0; i < numberOfTests; i++)
      {
         DenseMatrix64F matrix = new DenseMatrix64F(4, 4);
         
         this.createRandomTransformationMatrix(matrix, random);

         RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform(matrix);

         this.createRandomTransformationMatrix(matrix, random);
         RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform(matrix);

         double radiusOfRotation = random.nextDouble();
         double magnitudeOfTransform1 = TransformTools.getSizeOfTransformBetweenTwoWithRotationScaled(transformFromWorldToA1, transformFromWorldToA2, radiusOfRotation);
         
         RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);
         double magnitudeOfTransform2 = TransformTools.getSizeOfTransformWithRotationScaled(transformA2toA1, radiusOfRotation);
         
         assertEquals(magnitudeOfTransform1, magnitudeOfTransform2, 1e-9);
      }
   }
   
   private void createRandomTransformationMatrix(DenseMatrix64F matrix, Random random)
   {
      Matrix3d rotX = new Matrix3d();
      Matrix3d rotY = new Matrix3d();
      Matrix3d rotZ = new Matrix3d();
      Vector3d trans = new Vector3d();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.mul(rotY);
      rotX.mul(rotZ);

      matrix.set(0, 0, rotX.m00);
      matrix.set(0, 1, rotX.m01);
      matrix.set(0, 2, rotX.m02);
      matrix.set(0, 3, trans.x);
      matrix.set(1, 0, rotX.m10);
      matrix.set(1, 1, rotX.m11);
      matrix.set(1, 2, rotX.m12);
      matrix.set(1, 3, trans.y);
      matrix.set(2, 0, rotX.m20);
      matrix.set(2, 1, rotX.m21);
      matrix.set(2, 2, rotX.m22);
      matrix.set(2, 3, trans.z);
      matrix.set(3, 0, 0);
      matrix.set(3, 1, 0);
      matrix.set(3, 2, 0);
      matrix.set(3, 3, 1);
   }

   private void createRandomRotationMatrixX(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = 1;
      matrix.m01 = 0;
      matrix.m02 = 0;
      matrix.m10 = 0;
      matrix.m11 = cTheta;
      matrix.m12 = -sTheta;
      matrix.m20 = 0;
      matrix.m21 = sTheta;
      matrix.m22 = cTheta;
   }

   private void createRandomRotationMatrixY(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = cTheta;
      matrix.m01 = 0;
      matrix.m02 = sTheta;
      matrix.m10 = 0;
      matrix.m11 = 1;
      matrix.m12 = 0;
      matrix.m20 = -sTheta;
      matrix.m21 = 0;
      matrix.m22 = cTheta;
   }

   private void createRandomRotationMatrixZ(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.m00 = cTheta;
      matrix.m01 = -sTheta;
      matrix.m02 = 0;
      matrix.m10 = sTheta;
      matrix.m11 = cTheta;
      matrix.m12 = 0;
      matrix.m20 = 0;
      matrix.m21 = 0;
      matrix.m22 = 1;
   }

   private void randomizeVector(Random random, Vector3d vector)
   {
      vector.x = random.nextDouble();
      vector.y = random.nextDouble();
      vector.z = random.nextDouble();
   }
}
