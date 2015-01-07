package us.ihmc.graphics3DAdapter.jme.util;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import org.junit.Test;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.RotationFunctions;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Random;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.assertTrue;
import static us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils.transformFromZupToJMECoordinates;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/18/13
 * Time: 5:24 PM
 * To change this template use File | Settings | File Templates.
 */
public class JMEGeometryUtilsTest
{
   @Test(timeout=300000)
   public void testGetInverse()
   {
      RigidBodyTransform identity = new RigidBodyTransform();
      identity.setIdentity();

      Transform transformIdentity = Transform.IDENTITY;
      RigidBodyTransform transform3DIdentity = JMEDataTypeUtils.jmeTransformToTransform3D(transformIdentity);

      // Making sure that JME concept of identity is the same as java vecmath
      assertTrue(identity.epsilonEquals(transform3DIdentity, 1e-6));

      Random random = new Random(100L);
      for (int i = 0; i < 100; i++)
      {
         RigidBodyTransform transform3D = RandomTools.generateRandomTransform(random);
         Transform transform = JMEDataTypeUtils.j3dTransform3DToJMETransform(transform3D);
         Transform transformInverse = JMEGeometryUtils.getInverse(transform);
         RigidBodyTransform transform3DInverse = JMEDataTypeUtils.jmeTransformToTransform3D(transformInverse);

         RigidBodyTransform shouldBeIdentity = new RigidBodyTransform();
         shouldBeIdentity.multiply(transform3D, transform3DInverse);

         assertTrue(shouldBeIdentity.epsilonEquals(identity, 1e-6));
      }
   }

   @Test(timeout=300000)
   public void testTransformFromJMECoordinatesToZup()
   {
      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.setIdentity();
      transform3D.setTranslationAndIdentityRotation(new Vector3d(1.0, 0.0, 0.0));

      RigidBodyTransform transform = JMEGeometryUtils.transformFromJMECoordinatesToZup(transform3D);

      // Unit vector in x
      Vector3d originalVector = new Vector3d(1.0, 0.0, 0.0);
      Vector3d originalVectorTransformedToZup = new Vector3d();
      transform.transform(originalVector, originalVectorTransformedToZup);

      Vector3d expectedAnswer = new Vector3d(0.0, 1.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Y
      originalVector = new Vector3d(0.0, 1.0, 0.0);
      originalVectorTransformedToZup = new Vector3d();
      transform.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3d(0.0, 0.0, 1.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Z
      originalVector = new Vector3d(0.0, 0.0, 1.0);
      originalVectorTransformedToZup = new Vector3d();
      transform.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3d(1.0, 0.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));
   }

   @Test(timeout=300000)
   public void testTransformFromJMECoordinatesToZupWithItsInverse()
   {
      Random random = new Random(100L);
      for (int i = 0; i < 100; i++)
      {
         RigidBodyTransform transform3Doriginal = RandomTools.generateRandomTransform(random);
         RigidBodyTransform transformToZup = JMEGeometryUtils.transformFromJMECoordinatesToZup(transform3Doriginal);
         RigidBodyTransform transform3backToOriginal = transformFromZupToJMECoordinates(transformToZup);

         assertTrue(transform3backToOriginal.epsilonEquals(transform3Doriginal, 1e-6));
      }
   }

   @Test(timeout=300000)
   public void testTransformFromJMECoordinatesToZupWith90RotAboutX()
   {
      RigidBodyTransform transform3D;
      RigidBodyTransform transform;
      Vector3d originalVector;

      //*****
      transform3D = new RigidBodyTransform();
      transform3D.setIdentity();

      // 90 degree about JME x
      AxisAngle4d axisAngle = new AxisAngle4d(1.0, 0.0, 0.0, Math.PI/2.0);
      transform3D.setRotationAndZeroTranslation(axisAngle);

      // expected that this is
      transform = JMEGeometryUtils.transformFromJMECoordinatesToZup(transform3D);

      AxisAngle4d axisAngleTransformed = new AxisAngle4d();

      Quat4d quat4d = new Quat4d();
      transform.get(quat4d);
      axisAngleTransformed.set(quat4d);

      // Unit vector in x
      originalVector = new Vector3d(1.0, 0.0, 0.0);
      Vector3d originalVectorTransformedToZup = new Vector3d();

      transform.transform(originalVector, originalVectorTransformedToZup);

      Vector3d expectedAnswer = new Vector3d(0.0, 1.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Y
      originalVector = new Vector3d(0.0, 1.0, 0.0);
      originalVectorTransformedToZup = new Vector3d();

      transform.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3d(1.0, 0.0, 0.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));

      //    Unit vector in Z
      originalVector = new Vector3d(0.0, 0.0, 1.0);
      originalVectorTransformedToZup = new Vector3d();

      transform.transform(originalVector, originalVectorTransformedToZup);

      expectedAnswer = new Vector3d(0.0, 0.0, -1.0);

      assertTrue(expectedAnswer.epsilonEquals(originalVectorTransformedToZup, 1e-6));
   }

   @Test(timeout=300000)
   public void testTransformFromZupToJMECoordinates()
   {
      Vector3f pointOriginal, pointTransformed, expectedAnswer;

      //In world
      pointOriginal = new Vector3f(1.0f, 0.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 0.0f, 1.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 1.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(1.0f, 0.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 0.0f, 1.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 1.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));
   }

   @Test(timeout=300000)
   public void testTransformFromJMECoordinatesToZupPoint()
   {
      Vector3f pointOriginal, pointTransformed, expectedAnswer;

      //In JME
      pointOriginal = new Vector3f(1.0f, 0.0f, 0.0f);

      //In WORLD
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 1.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      pointOriginal = new Vector3f(0.0f, 1.0f, 0.0f);

      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 0.0f, 1.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      pointOriginal = new Vector3f(0.0f, 0.0f, 1.0f);

      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromJMECoordinatesToZup(pointTransformed);
      expectedAnswer = new Vector3f(1.0f, 0.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));
   }


   @Test(timeout=300000)
   public void testTransformFromZupToJMECoordinatesQuaternion()
   {
      Random random = new Random(100L);
      for (int i = 0; i < 100; i++)
      {
         AxisAngle4d axisAngle4d = RandomTools.generateRandomRotation(random);
         Quat4d quat4d = new Quat4d();
         quat4d.set(axisAngle4d);


      }

//      transformFromZupToJMECoordinates(Quaternion rotation)
   }

   @Test(timeout=300000)
   public void testRotationsFromAndToJMEToZupCoordinates()
   {
      //JME to World
      Quaternion quaternion = JMEGeometryUtils.getRotationFromJMEToZupCoordinates();
      Quat4d quat4d = JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(quaternion);

      Matrix3d matrix3d = new Matrix3d();
      matrix3d.set(quat4d);

      Vector3f pointOriginal, pointTransformed, expectedAnswer;

      //In world
      pointOriginal = new Vector3f(1.0f, 0.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 0.0f, 1.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 1.0f, 0.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(1.0f, 0.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

      //In world
      pointOriginal = new Vector3f(0.0f, 0.0f, 1.0f);

      //In JME
      pointTransformed = new Vector3f(pointOriginal);
      JMEGeometryUtils.transformFromZupToJMECoordinates(pointTransformed);
      expectedAnswer = new Vector3f(0.0f, 1.0f, 0.0f);
      assertTrue(areVectorsEqual(expectedAnswer, pointTransformed));

   }

   private static boolean areVectorsEqual(Vector3f vector3fa, Vector3f vector3fb)
   {
      if (Math.abs(vector3fa.getX() - vector3fb.getX()) > 1e-6)
         return false;
      if (Math.abs(vector3fa.getY() - vector3fb.getY()) > 1e-6)
         return false;
      if (Math.abs(vector3fa.getZ() - vector3fb.getZ()) > 1e-6)
         return false;

      return true;
   }

}
