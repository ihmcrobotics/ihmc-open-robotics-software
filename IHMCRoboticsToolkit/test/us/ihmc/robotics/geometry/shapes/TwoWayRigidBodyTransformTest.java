package us.ihmc.robotics.geometry.shapes;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.Epsilons;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

public class TwoWayRigidBodyTransformTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSomeBasicTransformations() throws Exception
   {
      TwoWayRigidBodyTransform twoWayRigidBodyTransform = new TwoWayRigidBodyTransform();
      
      Random random = new Random(832498235L);
      
      Point3d translation = new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Matrix3d rotation = new Matrix3d();
      createRandomRotationMatrix(rotation, random);
      
      RigidBodyTransform forwardTransformUnsafe = twoWayRigidBodyTransform.getForwardTransformUnsafe();
      forwardTransformUnsafe.setTranslation(translation);
      forwardTransformUnsafe.setRotation(rotation);
      twoWayRigidBodyTransform.setBackwardTransform(forwardTransformUnsafe);
      
      Point3d originalPosition = new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector4d originalOrientation = new Vector4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), 1.0);
      
      Point3d resultPosition = new Point3d(originalPosition);
      Vector4d resultOrientation = new Vector4d(originalOrientation);
      
      twoWayRigidBodyTransform.transformForward(resultPosition);
      twoWayRigidBodyTransform.transformForward(resultOrientation);
      twoWayRigidBodyTransform.transformBackward(resultPosition);
      twoWayRigidBodyTransform.transformBackward(resultOrientation);
      
      JUnitTools.assertPoint3dEquals("not equal", originalPosition, resultPosition, Epsilons.ONE_TRILLIONTH);
   }
   
   private void createRandomRotationMatrix(Matrix3d matrix, Random random)
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

      matrix.setM00(rotX.getM00());
      matrix.setM01(rotX.getM01());
      matrix.setM02(rotX.getM02());
      matrix.setM10(rotX.getM10());
      matrix.setM11(rotX.getM11());
      matrix.setM12(rotX.getM12());
      matrix.setM20(rotX.getM20());
      matrix.setM21(rotX.getM21());
      matrix.setM22(rotX.getM22());
   }

   private void createRandomRotationMatrixX(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.setM00(1);
      matrix.setM01(0);
      matrix.setM02(0);
      matrix.setM10(0);
      matrix.setM11(cTheta);
      matrix.setM12(-sTheta);
      matrix.setM20(0);
      matrix.setM21(sTheta);
      matrix.setM22(cTheta);
   }

   private void createRandomRotationMatrixY(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.setM00(cTheta);
      matrix.setM01(0);
      matrix.setM02(sTheta);
      matrix.setM10(0);
      matrix.setM11(1);
      matrix.setM12(0);
      matrix.setM20(-sTheta);
      matrix.setM21(0);
      matrix.setM22(cTheta);
   }

   private void createRandomRotationMatrixZ(Random random, Matrix3d matrix)
   {
      double theta = random.nextDouble();
      double cTheta = Math.cos(theta);
      double sTheta = Math.sin(theta);
      matrix.setM00(cTheta);
      matrix.setM01(-sTheta);
      matrix.setM02(0);
      matrix.setM10(sTheta);
      matrix.setM11(cTheta);
      matrix.setM12(0);
      matrix.setM20(0);
      matrix.setM21(0);
      matrix.setM22(1);
   }

   private void randomizeVector(Random random, Vector3d vector)
   {
      vector.setX(random.nextDouble());
      vector.setY(random.nextDouble());
      vector.setZ(random.nextDouble());
   }
}
