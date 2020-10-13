package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class TransformToolsTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testRotate()
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      TransformTools.appendRotation(transform, Math.PI / 4, Axis3D.X);

      RigidBodyTransform transform2 = new RigidBodyTransform();

      transform2.setRotationRollAndZeroTranslation(Math.PI / 4);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transform2, 1e-7);

      transform = new RigidBodyTransform();

      TransformTools.appendRotation(transform, 3 * Math.PI / 4, Axis3D.Y);

      transform2 = new RigidBodyTransform();

      transform2.setRotationPitchAndZeroTranslation(3 * Math.PI / 4);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transform2, 1e-7);

      transform = new RigidBodyTransform();

      TransformTools.appendRotation(transform, -Math.PI / 2, Axis3D.Z);

      transform2 = new RigidBodyTransform();

      transform2.setRotationYawAndZeroTranslation(-Math.PI / 2);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transform2, 1e-7);
   }

   @Test
   public void testGetTransformFromA1toA2Simple()
   {
      Vector3D vectorA1 = new Vector3D(-1.0, -2.0, -3.0);
      RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform();
      transformFromWorldToA1.getTranslation().set(vectorA1);

      Vector3D vectorA2 = new Vector3D(vectorA1);
      vectorA2.negate();
      RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform();
      transformFromWorldToA2.getTranslation().set(vectorA2);

      ReferenceFrame a1 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("a1", ReferenceFrame.getWorldFrame(), transformFromWorldToA1);
      ReferenceFrame a2 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("a2", ReferenceFrame.getWorldFrame(), transformFromWorldToA2);

      RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);

      Point3D a2Origin = new Point3D();
      transformA2toA1.transform(a2Origin);

      // System.out.println("a2Origin after transform" + a2Origin);

      FramePoint3D a2OriginFramePoint = new FramePoint3D(a2);

      a2OriginFramePoint.changeFrame(a1);

      // System.out.println("a2OriginFramePoint = " + a2OriginFramePoint);

      a2Origin.epsilonEquals(a2OriginFramePoint, 1e-9);
   }

   @Test
   public void testGetTransformFromA1toA2Random()
   {
      DMatrixRMaj matrix = new DMatrixRMaj(4, 4);
      Random random = new Random(111L);

      this.createRandomTransformationMatrix(matrix, random);

      RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform(matrix);

      this.createRandomTransformationMatrix(matrix, random);
      RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform(matrix);

      ReferenceFrame a1 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("a1", ReferenceFrame.getWorldFrame(), transformFromWorldToA1);
      ReferenceFrame a2 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("a2", ReferenceFrame.getWorldFrame(), transformFromWorldToA2);

      RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);

      Point3D a2Origin = new Point3D();
      transformA2toA1.transform(a2Origin);

      // System.out.println("a2Origin after transform" + a2Origin);

      FramePoint3D a2OriginFramePoint = new FramePoint3D(a2);

      a2OriginFramePoint.changeFrame(a1);

      // System.out.println("a2OriginFramePoint = " + a2OriginFramePoint);

      a2Origin.epsilonEquals(a2OriginFramePoint, 1e-9);
   }

   @Test
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

   @Test
   public void testgetTransformDifferenceBetweenTwoTransforms()
   {

      Random random = new Random();

      int numberOfTests = 1000;
      for (int i = 0; i < numberOfTests; i++)
      {
         DMatrixRMaj matrix = new DMatrixRMaj(4, 4);

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

   private void createRandomTransformationMatrix(DMatrixRMaj matrix, Random random)
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
      matrix.setToRollOrientation(theta);
   }

   private void createRandomRotationMatrixY(Random random, RotationMatrix matrix)
   {
      double theta = random.nextDouble();
      matrix.setToPitchOrientation(theta);
   }

   private void createRandomRotationMatrixZ(Random random, RotationMatrix matrix)
   {
      double theta = random.nextDouble();
      matrix.setToYawOrientation(theta);
   }

   private void randomizeVector(Random random, Vector3D vector)
   {
      vector.setX(random.nextDouble());
      vector.setY(random.nextDouble());
      vector.setZ(random.nextDouble());
   }
}
