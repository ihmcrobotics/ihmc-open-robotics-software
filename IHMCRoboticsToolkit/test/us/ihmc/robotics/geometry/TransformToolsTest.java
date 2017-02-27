package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.testing.JUnitTools;

public class TransformToolsTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testRotate()
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      TransformTools.rotate(transform, Math.PI / 4, Axis.X);

      RigidBodyTransform transform2 = new RigidBodyTransform();

      transform2.setRotationRollAndZeroTranslation(Math.PI / 4);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transform2, 1e-7);

      transform = new RigidBodyTransform();

      TransformTools.rotate(transform, 3 * Math.PI / 4, Axis.Y);

      transform2 = new RigidBodyTransform();

      transform2.setRotationPitchAndZeroTranslation(3 * Math.PI / 4);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transform2, 1e-7);

      transform = new RigidBodyTransform();

      TransformTools.rotate(transform, -Math.PI / 2, Axis.Z);

      transform2 = new RigidBodyTransform();

      transform2.setRotationYawAndZeroTranslation(-Math.PI / 2);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transform2, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDifferentiate()
   {
      Random random = new Random();
      RigidBodyTransform transform1 = new RigidBodyTransform();
      RigidBodyTransform transform2 = new RigidBodyTransform();
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTransformFromA1toA2Simple()
   {
      Vector3D vectorA1 = new Vector3D(-1.0, -2.0, -3.0);
      RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform();
      transformFromWorldToA1.setTranslation(vectorA1);

      Vector3D vectorA2 = new Vector3D(vectorA1);
      vectorA2.negate();
      RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform();
      transformFromWorldToA2.setTranslation(vectorA2);

      ReferenceFrame a1 = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("a1", ReferenceFrame.getWorldFrame(), transformFromWorldToA1);
      ReferenceFrame a2 = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("a2", ReferenceFrame.getWorldFrame(), transformFromWorldToA2);

      RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);

      Point3D a2Origin = new Point3D();
      transformA2toA1.transform(a2Origin);

      // System.out.println("a2Origin after transform" + a2Origin);

      FramePoint a2OriginFramePoint = new FramePoint(a2);

      a2OriginFramePoint.changeFrame(a1);

      // System.out.println("a2OriginFramePoint = " + a2OriginFramePoint);

      a2Origin.epsilonEquals(a2OriginFramePoint.getPoint(), 1e-9);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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

      Point3D a2Origin = new Point3D();
      transformA2toA1.transform(a2Origin);

      // System.out.println("a2Origin after transform" + a2Origin);

      FramePoint a2OriginFramePoint = new FramePoint(a2);

      a2OriginFramePoint.changeFrame(a1);

      // System.out.println("a2OriginFramePoint = " + a2OriginFramePoint);

      a2Origin.epsilonEquals(a2OriginFramePoint.getPoint(), 1e-9);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testgetTransformDifference()
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();

      Random random = new Random();

      int numberOfTests = 1000;
      for (int i = 0; i < numberOfTests; i++)
      {
         double angle = -Math.PI + 2 * Math.PI * random.nextDouble();
         Vector3D vector3d = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         vector3d.normalize();

         AxisAngle axisAngle = new AxisAngle(vector3d, angle);

         double vectorScale = 10.0;
         Vector3D vector3dTranlation = new Vector3D(-vectorScale + 2.0 * vectorScale * random.nextDouble(),
                                                    -vectorScale + 2.0 * vectorScale * random.nextDouble(),
                                                    -vectorScale + 2.0 * vectorScale * random.nextDouble());
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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
         double magnitudeOfTransform1 = TransformTools.getSizeOfTransformBetweenTwoWithRotationScaled(transformFromWorldToA1, transformFromWorldToA2,
                                                                                                      radiusOfRotation);

         RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);
         double magnitudeOfTransform2 = TransformTools.getSizeOfTransformWithRotationScaled(transformA2toA1, radiusOfRotation);

         assertEquals(magnitudeOfTransform1, magnitudeOfTransform2, 1e-9);
      }
   }

   private void createRandomTransformationMatrix(DenseMatrix64F matrix, Random random)
   {
      RotationMatrix rotX = new RotationMatrix();
      RotationMatrix rotY = new RotationMatrix();
      RotationMatrix rotZ = new RotationMatrix();
      Vector3D trans = new Vector3D();

      randomizeVector(random, trans);
      createRandomRotationMatrixX(random, rotX);
      createRandomRotationMatrixY(random, rotY);
      createRandomRotationMatrixZ(random, rotZ);

      rotX.multiply(rotY);
      rotX.multiply(rotZ);

      matrix.set(0, 0, rotX.getM00());
      matrix.set(0, 1, rotX.getM01());
      matrix.set(0, 2, rotX.getM02());
      matrix.set(0, 3, trans.getX());
      matrix.set(1, 0, rotX.getM10());
      matrix.set(1, 1, rotX.getM11());
      matrix.set(1, 2, rotX.getM12());
      matrix.set(1, 3, trans.getY());
      matrix.set(2, 0, rotX.getM20());
      matrix.set(2, 1, rotX.getM21());
      matrix.set(2, 2, rotX.getM22());
      matrix.set(2, 3, trans.getZ());
      matrix.set(3, 0, 0);
      matrix.set(3, 1, 0);
      matrix.set(3, 2, 0);
      matrix.set(3, 3, 1);
   }

   private void createRandomRotationMatrixX(Random random, RotationMatrix matrix)
   {
      double theta = random.nextDouble();
      matrix.setToRollMatrix(theta);
   }

   private void createRandomRotationMatrixY(Random random, RotationMatrix matrix)
   {
      double theta = random.nextDouble();
      matrix.setToPitchMatrix(theta);
   }

   private void createRandomRotationMatrixZ(Random random, RotationMatrix matrix)
   {
      double theta = random.nextDouble();
      matrix.setToYawMatrix(theta);
   }

   private void randomizeVector(Random random, Vector3D vector)
   {
      vector.setX(random.nextDouble());
      vector.setY(random.nextDouble());
      vector.setZ(random.nextDouble());
   }
}
